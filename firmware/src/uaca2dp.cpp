#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "usb_device_uac.h"         // ESP USB audio device (UAC) driver
#include "BluetoothA2DPSource.h"    // Bluetooth A2DP source library (pschatzmann's ESP32-A2DP)

// Configuration constants
#define AUDIO_SAMPLE_RATE   48000   // 48 kHz sample rate
#define AUDIO_CHANNELS      2       // stereo
#define AUDIO_BITS_PER_SAMPLE 16    // 16-bit PCM
#define RINGBUF_SIZE        (8 * 1024)  // 8 KB ring buffer for audio data

// Global state for audio control
static RingbufHandle_t audio_ringbuf = NULL;
static bool uac_mute_flag = false;
static uint32_t uac_volume_level = 100;  // Volume level (0-100% by default)

// Bluetooth A2DP source object (for sending audio to headphones)
static BluetoothA2DPSource a2dp_source;

// Callback for USB Audio Class speaker output (host sending audio data to device).
// This function is called whenever the host provides new PCM audio samples for output.
static esp_err_t uac_output_cb(uint8_t *buf, size_t len, void *cb_ctx) {
    // Copy the received audio samples into the ring buffer for the Bluetooth task to consume.
    if (audio_ringbuf != NULL && buf != NULL && len > 0) {
        BaseType_t ok = xRingbufferSend(audio_ringbuf, buf, len, 0);
        if (!ok) {
            // Ring buffer overflow: not enough space.
            // Drop the oldest audio data to make room (to avoid stalling the USB host).
            size_t recv_len;
            uint8_t *recv_buf = (uint8_t*) xRingbufferReceiveUpTo(audio_ringbuf, &recv_len, 0, len);
            if (recv_buf != NULL) {
                vRingbufferReturnItem(audio_ringbuf, recv_buf);
            }
            // Try again to push new data after freeing some space
            xRingbufferSend(audio_ringbuf, buf, len, 0);
        }
    }
    return ESP_OK;  // Indicate that the data has been handled
}

// Callback for USB Audio Class mute control.
static void uac_device_set_mute_cb(uint32_t mute, void *cb_ctx) {
    uac_mute_flag = (mute != 0);
    printf("USB Host set Mute: %s\n", uac_mute_flag ? "ON" : "OFF");
    // If mute is ON, we will drop or silence audio in the BT audio callback.
    // If OFF, we resume normal audio forwarding.
}

// Callback for USB Audio Class volume control.
static void uac_device_set_volume_cb(uint32_t volume, void *cb_ctx) {
    // `volume` is an unsigned value from the host. Typically this might be in percent (0-100)
    // or in some device-specific range (0-255 or 0-127 etc.). We'll handle common ranges.
    printf("USB Host set Volume: %u\n", volume);
    uac_volume_level = volume;
    // Normalize volume to 0-127 range for Bluetooth if needed:contentReference[oaicite:10]{index=10}.
    uint8_t bt_volume = 0;
    if (volume <= 100) {
        // Volume seems like 0-100%
        bt_volume = (uint8_t)((volume * 127) / 100);
    } else if (volume <= 127) {
        // Already in 0-127 range
        bt_volume = (uint8_t)volume;
    } else {
        // Possibly 0-255 range
        if (volume > 255) volume = 255;
        bt_volume = (uint8_t)((volume * 127) / 255);
    }
    // Set A2DP volume (this will send AVRCP absolute volume to the headset, if supported):contentReference[oaicite:11]{index=11}.
    a2dp_source.set_volume(bt_volume);
}

// Bluetooth A2DP data callback: provides audio data to the Bluetooth library when it needs more samples to send.
int32_t get_bt_audio_data(uint8_t *data, int32_t len) {
    if (!data || len <= 0) {
        return 0;
    }
    // If muted, output silence.
    if (uac_mute_flag || uac_volume_level == 0) {
        memset(data, 0, len);
        return len;
    }
    // Fetch audio from ring buffer
    size_t bytes_to_read = len;
    size_t bytes_read = 0;
    while (bytes_read < len) {
        size_t chunk_len;
        uint8_t *chunk = (uint8_t*) xRingbufferReceiveUpTo(audio_ringbuf, &chunk_len, 0, bytes_to_read);
        if (chunk == NULL) {
            // No audio available (buffer underrun). If host is still streaming, this could indicate a timing mismatch.
            // Fill remaining buffer with silence to avoid pops.
            memset(data + bytes_read, 0, len - bytes_read);
            bytes_read = len;
            break;
        }
        // Copy the chunk into the output buffer
        memcpy(data + bytes_read, chunk, chunk_len);
        bytes_read += chunk_len;
        bytes_to_read = len - bytes_read;
        // Return the chunk memory to the ring buffer
        vRingbufferReturnItem(audio_ringbuf, chunk);
    }
    // Apply volume scaling (simple linear scale based on uac_volume_level if it's 1-100).
    if (uac_volume_level < 100 && uac_volume_level > 0 && bytes_read > 0) {
        // Scale down the amplitude according to volume percentage.
        // We interpret uac_volume_level as 0-100.
        uint16_t *samples = (uint16_t*) data;
        size_t sample_count = bytes_read / 2;  // number of 16-bit samples
        for (size_t i = 0; i < sample_count; ++i) {
            int32_t sample = samples[i];
            // Apply volume percentage (simple multiplication)
            sample = (sample * (int)uac_volume_level) / 100;
            // Clip to 16-bit range just in case
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            samples[i] = (int16_t) sample;
        }
    }
    return bytes_read;
}

// Bluetooth AVRCP passthrough (remote control) callback.
// Called when the connected Bluetooth sink (headphones) sends a button press (play/pause/etc.).
void avrc_passthru_cb(uint8_t key, bool isReleased) {
    if (!isReleased) {
        // We act only on button release to avoid repeating actions
        return;
    }
    printf("BT AVRCP command received: 0x%02X\n", key);
    switch (key) {
        case 0x44: // PLAY pressed:contentReference[oaicite:12]{index=12}
        case 0x46: // PAUSE pressed
            // Toggle pause/play state (in this simple example, just mute/unmute as a "pause")
            uac_mute_flag = !uac_mute_flag;
            printf("Toggling pause, mute now: %s\n", uac_mute_flag ? "ON (paused)" : "OFF (playing)");
            break;
        case 0x4B: // FORWARD (next track)
            printf("AVRCP: Next track\n");
            // (No actual track control implemented; placeholder)
            break;
        case 0x4C: // BACKWARD (previous track)
            printf("AVRCP: Previous track\n");
            // (No actual track control implemented; placeholder)
            break;
        case 0x48: // REWIND
        case 0x49: // FAST FORWARD
            printf("AVRCP: Seek command (0x%02X)\n", key);
            break;
        default:
            printf("AVRCP: Unhandled command 0x%02X\n", key);
            break;
    }
}

// The main application entry point (ESP-IDF style)
extern "C" void app_main(void) {
    // Create the audio ring buffer to hold PCM data between USB and BT tasks
    audio_ringbuf = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_ALLOWSPLIT);
    if (audio_ringbuf == NULL) {
        printf("Failed to create audio ring buffer\n");
        return;
    }

    // Configure the USB UAC device with callbacks:contentReference[oaicite:13]{index=13}:contentReference[oaicite:14]{index=14}.
    uac_device_config_t uac_config = {
        .output_cb = uac_output_cb,             // Speaker output from host
        .input_cb = NULL,                      // Microphone input not used (NULL to disable):contentReference[oaicite:15]{index=15}
        .set_mute_cb = uac_device_set_mute_cb,  // Mute control callback
        .set_volume_cb = uac_device_set_volume_cb, // Volume control callback
        .cb_ctx = NULL
    };
    if (uac_device_init(&uac_config) != ESP_OK) {
        printf("Failed to initialize USB UAC device\n");
        return;
    }
    printf("USB Audio device initialized (48kHz stereo speaker)...\n");

    // Initialize and start the Bluetooth A2DP source
    // Set up the data callback that provides PCM data to the Bluetooth transmitter:contentReference[oaicite:16]{index=16}.
    a2dp_source.set_auto_reconnect(true);  // auto-reconnect to headphones if connection drops
    a2dp_source.set_stream_reader(nullptr, false); // disable any default I2S output, we handle data manually
    a2dp_source.set_data_callback(get_bt_audio_data);
    // Set up remote control (AVRCP) callback to handle play/pause/volume from headphone:contentReference[oaicite:17]{index=17}.
    a2dp_source.set_avrc_passthru_command_callback(avrc_passthru_cb);

    // Start Bluetooth and attempt to connect to the headphones. 
    // Replace "MyHeadphones" with the Bluetooth name of your headset or speaker.
    const char *deviceName = "MyHeadphones";  
    printf("Starting Bluetooth A2DP source, looking for device \"%s\"...\n", deviceName);
    a2dp_source.start(deviceName);
    // Note: The A2DP library will handle Bluetooth initialization and pairing. 
    // Ensure the headphone is in pairing mode or already bonded.

    printf("Bluetooth A2DP source started. Waiting for headphone connection...\n");
    // After connection, the ESP32 will stream audio received via USB to the Bluetooth headphones.
}
