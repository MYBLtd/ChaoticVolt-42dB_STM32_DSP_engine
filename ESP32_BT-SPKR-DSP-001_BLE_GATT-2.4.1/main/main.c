/*
 * ESP32 A2DP Sink + BLE GATT Controller for External DSP
 * V4 Architecture: A2DP audio → I2S → STM32 DSP, BLE GATT control + UART forwarding
 *
 * Core 0: BT controller + Bluedroid + A2DP + BLE GATT + I2S write + UART + OTA + NVS
 *
 * Audio path: Phone (SBC) → ESP32 A2DP sink → I2S → STM32 → PCM5102A
 * Control path: BLE GATT → UART → STM32
 *
 * For ESP-IDF v6.0-beta1 or later
 *
 * Author: Robin Kluit
 * Date: 2026-01-20
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"

/* Project modules */
#include "ble_gatt_dsp.h"
#include "nvs_settings.h"
#include "ota_manager.h"

/* Uncomment to enable I2S sine test mode (bypasses all Bluetooth) */
// #define I2S_SINE_TEST

static const char *TAG = "BT_SPEAKER";

/* Device name base - will be appended with MAC suffix */
#define BT_DEVICE_NAME_BASE "42 Decibels"
#define BT_DEVICE_NAME_MAX_LEN 32

/* Dynamic device name with MAC suffix (e.g., "42 Decibels-A7B3") */
static char s_device_name[BT_DEVICE_NAME_MAX_LEN] = BT_DEVICE_NAME_BASE;

/* I2S GPIO Configuration (shared clocks to STM32 + PCM5102A) */
#define I2S_BCK_PIN     GPIO_NUM_26
#define I2S_WS_PIN      GPIO_NUM_25
#define I2S_DATA_PIN    GPIO_NUM_22

/* I2S configuration */
#define I2S_SAMPLE_RATE     44100
#define I2S_BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT

/* Ring buffer for A2DP → I2S decoupling */
#define RINGBUF_SIZE        (32 * 1024)

/* Pre-buffer: accumulate this much audio before starting I2S output.
 * 44100 Hz * 2ch * 2 bytes * 0.050s = 8820 bytes ≈ 50ms */
#define PREBUF_BYTES        8820

/* Watchdog timeout in seconds */
#define WDT_TIMEOUT_SEC     30

/* I2S channel handle */
static i2s_chan_handle_t i2s_tx_handle = NULL;

/* Ring buffer handle */
static RingbufHandle_t s_ringbuf = NULL;

/* Connection state tracking */
static bool s_a2dp_connected = false;
static bool s_audio_started = false;
static esp_bd_addr_t s_peer_bda = {0};

/* Current sample rate */
static uint32_t s_current_sample_rate = I2S_SAMPLE_RATE;

/* Forward declarations */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

/*
 * Build device name with MAC address suffix for unique identification
 * Format: "42 Decibels-XXXX" where XXXX is last 4 hex chars of MAC
 */
static void build_device_name(void)
{
    const uint8_t *mac = esp_bt_dev_get_address();
    if (mac != NULL) {
        snprintf(s_device_name, BT_DEVICE_NAME_MAX_LEN, "%s-%02X%02X",
                 BT_DEVICE_NAME_BASE, mac[4], mac[5]);
        ESP_LOGI(TAG, "Device name: %s", s_device_name);
    } else {
        ESP_LOGW(TAG, "MAC address not available, using base name");
    }
}

/*
 * Get the device name (for use by BLE module)
 */
const char *bt_get_device_name(void)
{
    return s_device_name;
}

/*
 * Initialize I2S for audio output
 * ESP32 is I2S master — BCK/LRCK shared to STM32 and PCM5102A
 */
static esp_err_t i2s_init(void)
{
    ESP_LOGI(TAG, "Initializing I2S...");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 8;       /* 8 DMA descriptors (default: 6) */
    chan_cfg.dma_frame_num = 480;    /* 480 frames per descriptor (default: 240) */
    chan_cfg.auto_clear = true;      /* Output silence on underrun */

    esp_err_t ret = i2s_new_channel(&chan_cfg, &i2s_tx_handle, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_BITS_PER_SAMPLE, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(i2s_tx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S std mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(i2s_tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2S initialized: BCK=GPIO%d, WS=GPIO%d, DOUT=GPIO%d @ %dHz",
             I2S_BCK_PIN, I2S_WS_PIN, I2S_DATA_PIN, I2S_SAMPLE_RATE);
    return ESP_OK;
}

/*
 * Reconfigure I2S sample rate based on A2DP stream parameters
 */
static esp_err_t i2s_reconfigure(uint32_t sample_rate)
{
    if (i2s_tx_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (sample_rate == s_current_sample_rate) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Reconfiguring I2S to %lu Hz", (unsigned long)sample_rate);

    esp_err_t ret = i2s_channel_disable(i2s_tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable I2S: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    ret = i2s_channel_reconfig_std_clock(i2s_tx_handle, &clk_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconfigure I2S clock: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(i2s_tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S: %s", esp_err_to_name(ret));
        return ret;
    }

    s_current_sample_rate = sample_rate;
    return ESP_OK;
}

/*
 * GAP callback for handling Bluetooth connection events and pairing
 */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(TAG, "PIN request, using default '0000'");
        esp_bt_pin_code_t pin_code = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "Confirm request for numeric comparison, confirming...");
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "Passkey notification: %06lu", param->key_notif.passkey);
        break;

    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "Passkey request");
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "Power mode changed: %d (0=active, 2=sniff)", param->mode_chg.mode);
        if (param->mode_chg.mode == ESP_BT_PM_MD_SNIFF && s_a2dp_connected) {
            ESP_LOGW(TAG, "Sniff mode while A2DP connected — re-requesting QoS");
            esp_bt_gap_set_qos(s_peer_bda, 40);
        }
        break;

    default:
        ESP_LOGD(TAG, "GAP event: %d", event);
        break;
    }
}

/*
 * A2DP callback for handling audio streaming events
 */
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(TAG, "A2DP connected");
            s_a2dp_connected = true;
            memcpy(s_peer_bda, param->conn_stat.remote_bda, sizeof(esp_bd_addr_t));
            /* Set low poll interval to prevent sniff mode during audio */
            esp_bt_gap_set_qos(s_peer_bda, 40);
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG, "A2DP disconnected");
            s_a2dp_connected = false;
            s_audio_started = false;
        }
        break;

    case ESP_A2D_AUDIO_STATE_EVT:
        if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
            ESP_LOGI(TAG, "Audio stream started");
            s_audio_started = true;
        } else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_SUSPEND) {
            ESP_LOGI(TAG, "Audio stream suspended");
            s_audio_started = false;
        }
        break;

    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(TAG, "Audio configuration received");
        if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            uint32_t sample_rate = 44100;  /* Safe default */
            uint8_t samp_freq = param->audio_cfg.mcc.cie.sbc_info.samp_freq;

            ESP_LOGI(TAG, "SBC samp_freq mask: 0x%02x", samp_freq);

            /* Only accept single-bit config values, not capability masks */
            int bit_count = __builtin_popcount(samp_freq & 0x0F);
            if (bit_count != 1) {
                ESP_LOGW(TAG, "samp_freq has %d bits set (expected 1), defaulting to 44.1kHz", bit_count);
            } else if (samp_freq & 0x01) {
                sample_rate = 48000;
            } else if (samp_freq & 0x02) {
                sample_rate = 44100;
            } else if (samp_freq & 0x04) {
                sample_rate = 32000;
            } else if (samp_freq & 0x08) {
                sample_rate = 16000;
            }

            ESP_LOGI(TAG, "SBC codec, sample rate: %lu Hz", (unsigned long)sample_rate);
            i2s_reconfigure(sample_rate);
        }
        break;

    case ESP_A2D_PROF_STATE_EVT:
        if (param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS) {
            ESP_LOGI(TAG, "A2DP profile initialized");
        }
        break;

    default:
        ESP_LOGD(TAG, "A2DP event: %d", event);
        break;
    }
}

/*
 * A2DP data callback - receives decoded PCM audio, sends to ring buffer
 * Runs on BTC task (Core 0). Non-blocking to avoid stalling BT stack.
 * Audio consumed by I2S writer task on Core 1.
 */
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    if (s_ringbuf == NULL || data == NULL || len == 0) {
        return;
    }

    if (xRingbufferSend(s_ringbuf, data, len, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Ring buffer full, dropping %lu bytes", (unsigned long)len);
    }
}

/*
 * I2S writer task — pinned to Core 1
 * Pre-buffers audio data, then writes continuously to keep DMA always fed.
 * This eliminates DMA underruns that cause crackling.
 */
static void i2s_writer_task(void *arg)
{
    ESP_LOGI(TAG, "I2S writer task started on Core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Waiting for pre-buffer (%d bytes / ~%d ms)...",
             PREBUF_BYTES, PREBUF_BYTES * 1000 / (I2S_SAMPLE_RATE * 4));

    /* Wait until ring buffer has enough data to prevent initial underrun */
    while (1) {
        UBaseType_t items_waiting = 0;
        vRingbufferGetInfo(s_ringbuf, NULL, NULL, NULL, NULL, &items_waiting);
        if (items_waiting >= PREBUF_BYTES) {
            ESP_LOGI(TAG, "Pre-buffer filled (%lu bytes), starting I2S output",
                     (unsigned long)items_waiting);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* Continuously drain ring buffer to I2S — keeps DMA always fed */
    size_t item_size = 0;
    size_t bytes_written = 0;
    while (1) {
        void *data = xRingbufferReceive(s_ringbuf, &item_size, portMAX_DELAY);
        if (data != NULL && item_size > 0) {
            i2s_channel_write(i2s_tx_handle, data, item_size, &bytes_written, portMAX_DELAY);
            vRingbufferReturnItem(s_ringbuf, data);
        }
    }
}


/*
 * AVRCP Controller callback for media control events
 */
static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "AVRC %s (A2DP: %s)",
                 param->conn_stat.connected ? "connected" : "disconnected",
                 s_a2dp_connected ? "connected" : "disconnected");
        break;

    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
        ESP_LOGD(TAG, "AVRC passthrough response");
        break;

    case ESP_AVRC_CT_METADATA_RSP_EVT:
        ESP_LOGD(TAG, "AVRC metadata response");
        break;

    case ESP_AVRC_CT_PLAY_STATUS_RSP_EVT:
        ESP_LOGD(TAG, "AVRC play status response");
        break;

    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        ESP_LOGD(TAG, "AVRC change notification");
        break;

    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
        ESP_LOGD(TAG, "AVRC remote features: 0x%lx", param->rmt_feats.feat_mask);
        break;

    default:
        ESP_LOGD(TAG, "AVRC event: %d", event);
        break;
    }
}

/*
 * Callback when settings change via BLE (for NVS persistence)
 */
static void settings_changed_callback(void)
{
    nvs_dsp_settings_t settings;
    nvs_settings_get(&settings);
    nvs_settings_request_save();
}

/*
 * Callback for OTA status updates (for BLE notifications)
 */
static void ota_status_callback(const ota_status_t *status)
{
    ble_gatt_dsp_notify_ota_status((const uint8_t *)status);
}

/*
 * Initialize Bluetooth stack (dual mode: Classic + BLE)
 */
static esp_err_t bluetooth_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing Bluetooth (dual mode)...");

    /* Initialize BT controller */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Enable dual mode (Classic BT + BLE) */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize Bluedroid */
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Build unique device name with MAC suffix */
    build_device_name();

    /* Set device name for Classic BT */
    esp_bt_gap_set_device_name(s_device_name);

    /* Register GAP callback for pairing (Classic BT) */
    esp_bt_gap_register_callback(bt_app_gap_cb);

    /* Configure Secure Simple Pairing (SSP) for iOS compatibility */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    /* Enable SSP */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    /* Initialize AVRCP controller (must be before A2DP) */
    ret = esp_avrc_ct_register_callback(bt_app_avrc_ct_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AVRC register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_avrc_ct_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AVRC controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize A2DP sink (after AVRC) */
    ret = esp_a2d_register_callback(bt_app_a2d_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP register data callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_a2d_sink_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP sink init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set discoverable and connectable mode for Classic BT */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    ESP_LOGI(TAG, "Classic Bluetooth initialized");

    /* Initialize BLE GATT control service */
    ret = ble_gatt_dsp_init(settings_changed_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE GATT init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BLE GATT service initialized");

    return ESP_OK;
}

/*
 * Watchdog monitoring task - keeps system alive and monitors health
 */
static void watchdog_task(void *arg)
{
    ESP_LOGI(TAG, "Watchdog task started");

    esp_task_wdt_add(NULL);

    while (1) {
        esp_task_wdt_reset();

        if (s_a2dp_connected) {
            /* Read RSSI for connection quality monitoring */
            /* (RSSI read is async — result comes via GAP callback) */
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

#ifdef I2S_SINE_TEST
/*
 * I2S sine wave test — generates 1kHz tone directly to I2S
 * Bypasses Bluetooth entirely to isolate the I2S data path
 */
static void sine_test_task(void *arg)
{
    const int freq = 1000;
    const int samples_per_period = I2S_SAMPLE_RATE / freq;
    const int16_t amplitude = 16000;  /* ~50% of int16_t max */

    /* Pre-compute one period of stereo sine wave */
    int16_t sine_buf[samples_per_period * 2];
    for (int i = 0; i < samples_per_period; i++) {
        int16_t sample = (int16_t)(amplitude * sinf(2.0f * M_PI * i / samples_per_period));
        sine_buf[i * 2]     = sample;  /* Left */
        sine_buf[i * 2 + 1] = sample;  /* Right */
    }

    ESP_LOGI(TAG, "Sine test running: %d Hz, %d samples/period, amplitude=%d",
             freq, samples_per_period, amplitude);

    size_t bytes_written;
    while (1) {
        i2s_channel_write(i2s_tx_handle, sine_buf, sizeof(sine_buf),
                          &bytes_written, portMAX_DELAY);
    }
}
#endif

/*
 * Application entry point
 */
void app_main(void)
{
#ifdef I2S_SINE_TEST
    {
        ESP_LOGW(TAG, "=== I2S SINE TEST MODE ===");
        ESP_LOGW(TAG, "Bluetooth disabled — testing I2S data path only");

        esp_err_t err = i2s_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2S init failed, restarting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }

        xTaskCreatePinnedToCore(sine_test_task, "sine_test", 4096, NULL, 10, NULL, 1);
        return;
    }
#endif

    ESP_LOGI(TAG, "=== ESP32 A2DP Sink + BLE GATT Controller (V4) ===");
    ESP_LOGI(TAG, "A2DP audio → I2S → STM32 DSP, BLE GATT control + UART");
    ESP_LOGI(TAG, "Firmware version: %s", ota_mgr_get_version());

    /* Log heap info */
    ESP_LOGI(TAG, "Heap: free=%lu, largest_block=%lu, DRAM=%lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    /* Initialize NVS (required for Bluetooth and settings storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash erase required");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize NVS settings module */
    ret = nvs_settings_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS settings init failed, using defaults");
    }

    /* Configure Task Watchdog Timer for crash recovery */
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));

    /* Create ring buffer for A2DP → I2S decoupling */
    s_ringbuf = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (s_ringbuf == NULL) {
        ESP_LOGE(TAG, "Failed to create ring buffer, restarting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    /* Initialize I2S for audio output */
    ret = i2s_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S initialization failed, restarting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    /* Initialize Bluetooth (Classic A2DP + BLE GATT) */
    ret = bluetooth_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth initialization failed, restarting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    /* Initialize OTA manager */
    ret = ota_mgr_init(ota_status_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OTA manager init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "OTA manager initialized");
        if (ota_mgr_is_pending_verify()) {
            ESP_LOGW(TAG, "New firmware pending validation - send VALIDATE command via BLE");
        }
    }

    /* Create I2S writer task pinned to Core 1 (with pre-buffering) */
    xTaskCreatePinnedToCore(i2s_writer_task, "i2s_writer", 4096, NULL, 10, NULL, 1);

    /* Create watchdog monitoring task */
    xTaskCreate(watchdog_task, "watchdog", 2048, NULL, 5, NULL);

    /* Log cached settings */
    nvs_dsp_settings_t settings;
    nvs_settings_get(&settings);

    ESP_LOGI(TAG, "System ready");
    ESP_LOGI(TAG, "- Classic BT: A2DP sink waiting for audio connection");
    ESP_LOGI(TAG, "- BLE: GATT control service advertising");
    ESP_LOGI(TAG, "- UART: Forwarding commands to external DSP (STM32)");
    ESP_LOGI(TAG, "- I2S: BCK=GPIO%d, WS=GPIO%d, DOUT=GPIO%d → STM32",
             I2S_BCK_PIN, I2S_WS_PIN, I2S_DATA_PIN);
    ESP_LOGI(TAG, "- OTA: Hybrid BLE+WiFi updates enabled");
    ESP_LOGI(TAG, "- Cached settings: preset=%d, loudness=%s",
             settings.preset_id, settings.loudness ? "ON" : "OFF");
}
