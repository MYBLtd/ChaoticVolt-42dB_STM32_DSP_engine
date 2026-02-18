/*
 * BLE GATT DSP Control Service Implementation
 * FSD-DSP-001: DSP Control via BLE GATT
 *
 * Author: Robin Kluit
 * Date: 2026-01-20
 */

#include "ble_gatt_dsp.h"
#include "nvs_settings.h"
#include "ota_manager.h"
#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_timer.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* UART configuration for serial echo
 * Note: GPIO16/17 are used by PSRAM on ESP32-WROVER modules!
 * Note: GPIO25/26 are used by I2S (WS/BCK)
 * Using GPIO4 (TX) and GPIO5 (RX) which are free */
#define UART_ECHO_PORT      UART_NUM_2
#define UART_ECHO_TX_PIN    GPIO_NUM_4
#define UART_ECHO_RX_PIN    GPIO_NUM_5
#define UART_ECHO_BAUD      115200
#define UART_ECHO_BUF_SIZE  256

static const char *TAG = "BLE_GATT";

/* GATT profile instance */
#define DSP_PROFILE_NUM     1
#define DSP_PROFILE_APP_ID  0

/* Attribute indexes */
enum {
    IDX_SVC,                    /* Service declaration */
    IDX_CTRL_CHAR,              /* Control characteristic declaration */
    IDX_CTRL_VAL,               /* Control characteristic value */
    IDX_STATUS_CHAR,            /* Status characteristic declaration */
    IDX_STATUS_VAL,             /* Status characteristic value */
    IDX_STATUS_CCC,             /* Status Client Characteristic Configuration */
    IDX_GALACTIC_CHAR,          /* GalacticStatus characteristic declaration */
    IDX_GALACTIC_VAL,           /* GalacticStatus characteristic value */
    IDX_GALACTIC_CCC,           /* GalacticStatus Client Characteristic Configuration */
    /* OTA characteristics */
    IDX_OTA_CREDS_CHAR,         /* OTA Credentials characteristic declaration */
    IDX_OTA_CREDS_VAL,          /* OTA Credentials characteristic value */
    IDX_OTA_URL_CHAR,           /* OTA URL characteristic declaration */
    IDX_OTA_URL_VAL,            /* OTA URL characteristic value */
    IDX_OTA_CTRL_CHAR,          /* OTA Control characteristic declaration */
    IDX_OTA_CTRL_VAL,           /* OTA Control characteristic value */
    IDX_OTA_STATUS_CHAR,        /* OTA Status characteristic declaration */
    IDX_OTA_STATUS_VAL,         /* OTA Status characteristic value */
    IDX_OTA_STATUS_CCC,         /* OTA Status Client Characteristic Configuration */
    IDX_NB,                     /* Number of attributes */
};

/* GalacticStatus notification interval (FR-20: 2x per second) */
#define GALACTIC_NOTIFY_INTERVAL_MS  500

/* Service UUID */
static const uint8_t dsp_service_uuid[16] = DSP_SERVICE_UUID_128;

/* Characteristic UUIDs */
static const uint8_t dsp_control_uuid[16] = DSP_CONTROL_CHAR_UUID_128;
static const uint8_t dsp_status_uuid[16] = DSP_STATUS_CHAR_UUID_128;
static const uint8_t dsp_galactic_uuid[16] = DSP_GALACTIC_CHAR_UUID_128;

/* OTA Characteristic UUIDs */
static const uint8_t ota_creds_uuid[16] = OTA_CREDS_CHAR_UUID_128;
static const uint8_t ota_url_uuid[16] = OTA_URL_CHAR_UUID_128;
static const uint8_t ota_ctrl_uuid[16] = OTA_CONTROL_CHAR_UUID_128;
static const uint8_t ota_status_uuid[16] = OTA_STATUS_CHAR_UUID_128;

/* Characteristic properties */
static const uint8_t ctrl_char_prop = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t status_char_prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t galactic_char_prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* OTA Characteristic properties */
static const uint8_t ota_write_char_prop = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t ota_status_char_prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* Client Characteristic Configuration Descriptor default values */
static uint8_t status_ccc[2] = {0x00, 0x00};
static uint8_t galactic_ccc[2] = {0x00, 0x00};
static uint8_t ota_status_ccc[2] = {0x00, 0x00};

/* Control characteristic value (2 bytes: CMD + VAL) */
static uint8_t ctrl_value[2] = {0x00, 0x00};

/* Status characteristic value (4 bytes per Section 10.4) */
static uint8_t status_value[DSP_STATUS_SIZE] = {
    DSP_STATUS_PROTOCOL_VERSION,    /* VER */
    0x00,                           /* PRESET */
    0x00,                           /* LOUDNESS */
    0x01                            /* FLAGS (limiter active) */
};

/* GalacticStatus characteristic value (7 bytes per FR-18) */
static uint8_t galactic_value[DSP_GALACTIC_STATUS_SIZE] = {
    DSP_GALACTIC_PROTOCOL_VERSION,  /* VER: 0x42 */
    0x00,                           /* currentQuantumFlavor (preset) */
    0x01,                           /* shieldStatus (flags) */
    100,                            /* energyCoreLevel (placeholder) */
    50,                             /* distortionFieldStrength (volume placeholder) */
    100,                            /* Energy core (battery placeholder) */
    0                               /* lastContact (seconds) */
};

/* OTA characteristic values */
static uint8_t ota_creds_value[OTA_CREDS_MAX_SIZE] = {0};
static uint8_t ota_url_value[OTA_URL_MAX_SIZE] = {0};
static uint8_t ota_ctrl_value[OTA_CONTROL_SIZE] = {0};
static uint8_t ota_status_value[OTA_STATUS_SIZE] = {0};

/* GATT attribute table */
static const esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    /* Service Declaration */
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_PRI_SERVICE},
            ESP_GATT_PERM_READ,
            sizeof(dsp_service_uuid), sizeof(dsp_service_uuid), (uint8_t *)dsp_service_uuid
        }
    },

    /* Control Characteristic Declaration */
    [IDX_CTRL_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE},
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&ctrl_char_prop
        }
    },

    /* Control Characteristic Value */
    [IDX_CTRL_VAL] = {
        {ESP_GATT_RSP_BY_APP},  /* Manual response for write handling */
        {
            ESP_UUID_LEN_128, (uint8_t *)dsp_control_uuid,
            ESP_GATT_PERM_WRITE,
            sizeof(ctrl_value), sizeof(ctrl_value), ctrl_value
        }
    },

    /* Status Characteristic Declaration */
    [IDX_STATUS_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE},
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&status_char_prop
        }
    },

    /* Status Characteristic Value */
    [IDX_STATUS_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_128, (uint8_t *)dsp_status_uuid,
            ESP_GATT_PERM_READ,
            sizeof(status_value), sizeof(status_value), status_value
        }
    },

    /* Status Client Characteristic Configuration Descriptor */
    [IDX_STATUS_CCC] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(status_ccc), sizeof(status_ccc), status_ccc
        }
    },

    /* GalacticStatus Characteristic Declaration (FR-18) */
    [IDX_GALACTIC_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE},
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&galactic_char_prop
        }
    },

    /* GalacticStatus Characteristic Value */
    [IDX_GALACTIC_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_128, (uint8_t *)dsp_galactic_uuid,
            ESP_GATT_PERM_READ,
            sizeof(galactic_value), sizeof(galactic_value), galactic_value
        }
    },

    /* GalacticStatus Client Characteristic Configuration Descriptor */
    [IDX_GALACTIC_CCC] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(galactic_ccc), sizeof(galactic_ccc), galactic_ccc
        }
    },

    /* ========== OTA Characteristics ========== */

    /* OTA Credentials Characteristic Declaration */
    [IDX_OTA_CREDS_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE},
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&ota_write_char_prop
        }
    },

    /* OTA Credentials Characteristic Value */
    [IDX_OTA_CREDS_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {
            ESP_UUID_LEN_128, (uint8_t *)ota_creds_uuid,
            ESP_GATT_PERM_WRITE,
            sizeof(ota_creds_value), 0, ota_creds_value
        }
    },

    /* OTA URL Characteristic Declaration */
    [IDX_OTA_URL_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE},
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&ota_write_char_prop
        }
    },

    /* OTA URL Characteristic Value */
    [IDX_OTA_URL_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {
            ESP_UUID_LEN_128, (uint8_t *)ota_url_uuid,
            ESP_GATT_PERM_WRITE,
            sizeof(ota_url_value), 0, ota_url_value
        }
    },

    /* OTA Control Characteristic Declaration */
    [IDX_OTA_CTRL_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE},
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&ota_write_char_prop
        }
    },

    /* OTA Control Characteristic Value */
    [IDX_OTA_CTRL_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {
            ESP_UUID_LEN_128, (uint8_t *)ota_ctrl_uuid,
            ESP_GATT_PERM_WRITE,
            sizeof(ota_ctrl_value), 0, ota_ctrl_value
        }
    },

    /* OTA Status Characteristic Declaration */
    [IDX_OTA_STATUS_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE},
            ESP_GATT_PERM_READ,
            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&ota_status_char_prop
        }
    },

    /* OTA Status Characteristic Value */
    [IDX_OTA_STATUS_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_128, (uint8_t *)ota_status_uuid,
            ESP_GATT_PERM_READ,
            sizeof(ota_status_value), sizeof(ota_status_value), ota_status_value
        }
    },

    /* OTA Status Client Characteristic Configuration Descriptor */
    [IDX_OTA_STATUS_CCC] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(ota_status_ccc), sizeof(ota_status_ccc), ota_status_ccc
        }
    },
};

/* Advertising data - contains service UUID and flags
 * Flags: General Discoverable + BR/EDR Not Supported
 * Max 31 bytes: flags(3) + UUID(18) = 21 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = false,              /* Name in scan response to save space */
    .include_txpower = false,
    .min_interval = 0x0006,             /* 7.5ms */
    .max_interval = 0x0010,             /* 20ms */
    .appearance = 0x0841,               /* Speaker appearance */
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(dsp_service_uuid),
    .p_service_uuid = (uint8_t *)dsp_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_DMT_CONTROLLER_SPT | ESP_BLE_ADV_FLAG_DMT_HOST_SPT),
};

/* Scan response data - contains device name only
 * Service UUID is already in adv_data; keep scan response under 31 bytes
 * Name "42 Decibels-XXXX" (18) + TX power (3) + appearance (4) = 25 bytes */
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,               /* Device name here */
    .include_txpower = true,
    .appearance = 0x0841,               /* Speaker appearance */
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_DMT_CONTROLLER_SPT | ESP_BLE_ADV_FLAG_DMT_HOST_SPT),
};

/* Advertising parameters */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = BLE_ADV_INTERVAL_MIN,
    .adv_int_max = BLE_ADV_INTERVAL_MAX,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* BLE state */
typedef struct {
    uint16_t gatts_if;
    uint16_t conn_id;
    uint16_t handle_table[IDX_NB];
    bool connected;
    bool notifications_enabled;
    bool galactic_notifications_enabled;  /* CCCD for GalacticStatus (FR-18) */
    bool ota_notifications_enabled;       /* CCCD for OTA Status */
    int64_t last_contact_us;              /* Timestamp of last BLE interaction (FR-19) */
    TimerHandle_t galactic_notify_timer;  /* FreeRTOS timer for periodic notifications (FR-20) */
    ble_dsp_settings_cb_t settings_cb;
} ble_state_t;

static ble_state_t s_ble = {
    .gatts_if = ESP_GATT_IF_NONE,
    .conn_id = 0xFFFF,
    .connected = false,
    .notifications_enabled = false,
    .galactic_notifications_enabled = false,
    .ota_notifications_enabled = false,
    .last_contact_us = 0,
    .galactic_notify_timer = NULL,
    .settings_cb = NULL,
};

/* UART echo state */
static bool s_uart_echo_initialized = false;

/* Transient DSP state — not persisted in NVS, tracked locally for status notifications.
 * FLAGS byte bit layout (shared by STATUS[3] and shieldStatus in GalacticStatus):
 *   Bit 0 (0x01): Mute
 *   Bit 1 (0x02): Audio Duck
 *   Bit 2 (0x04): Loudness  (also in NVS)
 *   Bit 3 (0x08): Normalizer
 *   Bit 4 (0x10): Bass Boost
 *   Bit 5 (0x20): Bypass
 */
static uint8_t s_dsp_flags = 0x00;  /* All off at startup */
static uint8_t s_dsp_volume = 100;  /* Default 100% */

/* Forward declarations */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);
static void handle_control_write(const uint8_t *data, uint16_t len);
static void update_status_value(void);
static void update_galactic_status_value(void);
static void galactic_notify_timer_callback(TimerHandle_t timer);
static esp_err_t uart_echo_init(void);
static void uart_echo_gatt_command(const char *char_name, const uint8_t *data, uint16_t len);

/*
 * Initialize UART for serial echo of BLE GATT commands
 */
static esp_err_t uart_echo_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_ECHO_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(UART_ECHO_PORT, UART_ECHO_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(UART_ECHO_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(UART_ECHO_PORT, UART_ECHO_TX_PIN, UART_ECHO_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_uart_echo_initialized = true;
    ESP_LOGI(TAG, "UART echo initialized on TX=GPIO%d, RX=GPIO%d @ %d baud",
             UART_ECHO_TX_PIN, UART_ECHO_RX_PIN, UART_ECHO_BAUD);
    return ESP_OK;
}

/*
 * Echo BLE GATT command to serial port
 * Format: "GATT:<char_name>:<hex_bytes>\r\n"
 */
static void uart_echo_gatt_command(const char *char_name, const uint8_t *data, uint16_t len)
{
    if (!s_uart_echo_initialized || data == NULL || len == 0) {
        return;
    }

    /* Build message: "GATT:<char_name>:" + hex bytes + "\r\n" */
    char msg[128];
    int offset = snprintf(msg, sizeof(msg), "GATT:%s:", char_name);

    /* Append hex bytes */
    for (uint16_t i = 0; i < len && offset < (int)(sizeof(msg) - 4); i++) {
        offset += snprintf(msg + offset, sizeof(msg) - offset, "%02X", data[i]);
    }

    /* Append newline */
    offset += snprintf(msg + offset, sizeof(msg) - offset, "\r\n");

    /* Send to UART */
    int written = uart_write_bytes(UART_ECHO_PORT, msg, offset);
    ESP_LOGI(TAG, "UART echo: %s (wrote %d bytes)", msg, written);
}

/*
 * GAP event handler
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGD(TAG, "Advertising data set complete");
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGD(TAG, "Scan response data set complete");
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BLE advertising started");
        } else {
            ESP_LOGE(TAG, "BLE advertising start failed: %d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BLE advertising stopped");
        }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGD(TAG, "Connection params updated: interval=%d, latency=%d, timeout=%d",
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    default:
        break;
    }
}

/*
 * Handle control write commands (Section 10.3)
 */
static void handle_control_write(const uint8_t *data, uint16_t len)
{
    if (len < 2) {
        ESP_LOGW(TAG, "Control write too short: %d bytes", len);
        return;
    }

    uint8_t cmd = data[0];
    uint8_t val = data[1];

    ESP_LOGI(TAG, "Control command: CMD=0x%02X, VAL=0x%02X", cmd, val);

    bool settings_changed = false;

    switch (cmd) {
    case DSP_CMD_SET_PRESET:
        if (val < 4) {
            nvs_settings_update(val, 0xFF);  /* 0xFF = don't change loudness */
            settings_changed = true;
            ESP_LOGI(TAG, "Preset set to: %d (forwarded to UART)", val);
        } else {
            ESP_LOGW(TAG, "Invalid preset value: %d", val);
        }
        break;

    case DSP_CMD_SET_LOUDNESS:
        nvs_settings_update(0xFF, val != 0 ? 1 : 0);  /* 0xFF = don't change preset */
        if (val) s_dsp_flags |= 0x04; else s_dsp_flags &= ~0x04;
        settings_changed = true;
        ESP_LOGI(TAG, "Loudness set to: %s (forwarded to UART)", val ? "ON" : "OFF");
        break;

    case DSP_CMD_GET_STATUS:
        ESP_LOGI(TAG, "Status request received");
        break;

    case DSP_CMD_SET_MUTE:
        if (val) s_dsp_flags |= 0x01; else s_dsp_flags &= ~0x01;
        ESP_LOGI(TAG, "Mute set to: %s (forwarded to UART)", val ? "ON" : "OFF");
        break;

    case DSP_CMD_SET_AUDIO_DUCK:
        if (val) s_dsp_flags |= 0x02; else s_dsp_flags &= ~0x02;
        ESP_LOGI(TAG, "Audio Duck set to: %s (forwarded to UART)", val ? "ON" : "OFF");
        break;

    case DSP_CMD_SET_NORMALIZER:
        if (val) s_dsp_flags |= 0x08; else s_dsp_flags &= ~0x08;
        ESP_LOGI(TAG, "Normalizer set to: %s (forwarded to UART)", val ? "ON" : "OFF");
        break;

    case DSP_CMD_SET_VOLUME:
        s_dsp_volume = val;
        ESP_LOGI(TAG, "Volume set to: %d%% (forwarded to UART)", val);
        break;

    case DSP_CMD_SET_BYPASS:
        if (val) s_dsp_flags |= 0x20; else s_dsp_flags &= ~0x20;
        ESP_LOGI(TAG, "DSP Bypass set to: %s (forwarded to UART)", val ? "ON" : "OFF");
        break;

    case DSP_CMD_SET_BASS_BOOST:
        if (val) s_dsp_flags |= 0x10; else s_dsp_flags &= ~0x10;
        ESP_LOGI(TAG, "Bass Boost set to: %s (forwarded to UART)", val ? "ON" : "OFF");
        break;

    default:
        ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
        break;
    }

    /* Update status and notify */
    update_status_value();
    ble_gatt_dsp_notify_status();

    /* Callback for NVS persistence */
    if (settings_changed && s_ble.settings_cb != NULL) {
        s_ble.settings_cb();
    }
}

/*
 * Update status characteristic value from DSP state
 */
static void update_status_value(void)
{
    nvs_dsp_settings_t settings;
    nvs_settings_get(&settings);

    /* Merge loudness (NVS) into flags bit 2 */
    uint8_t flags = s_dsp_flags;
    if (settings.loudness) flags |= 0x04; else flags &= ~0x04;

    status_value[0] = DSP_STATUS_PROTOCOL_VERSION;
    status_value[1] = settings.preset_id;
    status_value[2] = settings.loudness;
    status_value[3] = flags;  /* All DSP feature flags */

    /* Update the attribute value in GATT database */
    if (s_ble.gatts_if != ESP_GATT_IF_NONE && s_ble.handle_table[IDX_STATUS_VAL] != 0) {
        esp_ble_gatts_set_attr_value(s_ble.handle_table[IDX_STATUS_VAL],
                                     sizeof(status_value), status_value);
    }
}

/*
 * Update GalacticStatus characteristic value (FR-18, FR-19)
 * Includes last contact time calculation
 */
static void update_galactic_status_value(void)
{
    nvs_dsp_settings_t settings;
    nvs_settings_get(&settings);

    /* Calculate last contact age in seconds (FR-19) */
    int64_t now_us = esp_timer_get_time();
    uint32_t age_sec = (now_us - s_ble.last_contact_us) / 1000000;
    if (age_sec > 255) {
        age_sec = 255;  /* Clamp to 8-bit max */
    }

    /* Build shieldStatus from all tracked DSP flags */
    uint8_t shield_status = s_dsp_flags;
    if (settings.loudness) shield_status |= 0x04; else shield_status &= ~0x04;

    galactic_value[0] = DSP_GALACTIC_PROTOCOL_VERSION;  /* Protocol version: 0x42 */
    galactic_value[1] = settings.preset_id;             /* currentQuantumFlavor */
    galactic_value[2] = shield_status;                  /* shieldStatus: all DSP feature flags */
    galactic_value[3] = 100;                            /* energyCoreLevel (placeholder) */
    galactic_value[4] = s_dsp_volume;                   /* distortionFieldStrength (volume 0-100) */
    galactic_value[5] = 100;                            /* battery (placeholder) */
    galactic_value[6] = (uint8_t)age_sec;               /* lastContact */

    /* Update the attribute value in GATT database */
    if (s_ble.gatts_if != ESP_GATT_IF_NONE && s_ble.handle_table[IDX_GALACTIC_VAL] != 0) {
        esp_ble_gatts_set_attr_value(s_ble.handle_table[IDX_GALACTIC_VAL],
                                     sizeof(galactic_value), galactic_value);
    }
}

/*
 * Timer callback for periodic GalacticStatus notifications (FR-20)
 * Called 2x per second (every 500ms)
 */
static void galactic_notify_timer_callback(TimerHandle_t timer)
{
    (void)timer;  /* Unused parameter */

    if (s_ble.connected && s_ble.galactic_notifications_enabled) {
        ble_gatt_dsp_notify_galactic_status();
    }
}

/*
 * GATTS event handler
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "GATT app registered, app_id=%d", param->reg.app_id);
            s_ble.gatts_if = gatts_if;

            /* Set device name for BLE (uses shared name with MAC suffix) */
            esp_ble_gap_set_device_name(bt_get_device_name());

            /* Configure advertising data */
            esp_ble_gap_config_adv_data(&adv_data);
            esp_ble_gap_config_adv_data(&scan_rsp_data);

            /* Create attribute table */
            esp_err_t ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Create attr table failed: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "GATT app register failed, status=%d", param->reg.status);
        }
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            if (param->add_attr_tab.num_handle == IDX_NB) {
                memcpy(s_ble.handle_table, param->add_attr_tab.handles,
                       sizeof(s_ble.handle_table));
                ESP_LOGI(TAG, "Attribute table created, handles=%d", param->add_attr_tab.num_handle);

                /* Start GATT service */
                esp_ble_gatts_start_service(s_ble.handle_table[IDX_SVC]);
            }
        } else {
            ESP_LOGE(TAG, "Create attr table failed, status=%d", param->add_attr_tab.status);
        }
        break;

    case ESP_GATTS_START_EVT:
        if (param->start.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "GATT service started");
            /* Start advertising */
            ble_gatt_dsp_start_advertising();
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "BLE client connected, conn_id=%d", param->connect.conn_id);
        s_ble.conn_id = param->connect.conn_id;
        s_ble.connected = true;

        /* Reset last contact timestamp (FR-19) */
        s_ble.last_contact_us = esp_timer_get_time();

        /* Update connection parameters for better latency (FR-14) */
        esp_ble_conn_update_params_t conn_params = {
            .latency = 0,
            .max_int = 0x20,    /* 40ms */
            .min_int = 0x10,    /* 20ms */
            .timeout = 400,     /* 4s */
        };
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gap_update_conn_params(&conn_params);

        /* Send initial status notification */
        update_status_value();

        /* Start GalacticStatus notification timer (FR-20) */
        if (s_ble.galactic_notify_timer != NULL) {
            xTimerStart(s_ble.galactic_notify_timer, 0);
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "BLE client disconnected, reason=0x%x", param->disconnect.reason);
        s_ble.conn_id = 0xFFFF;
        s_ble.connected = false;
        s_ble.notifications_enabled = false;
        s_ble.galactic_notifications_enabled = false;
        s_ble.ota_notifications_enabled = false;

        /* Stop GalacticStatus notification timer (FR-20) */
        if (s_ble.galactic_notify_timer != NULL) {
            xTimerStop(s_ble.galactic_notify_timer, 0);
        }

        /* Restart advertising (FR-15: BLE disconnect must not affect audio) */
        ble_gatt_dsp_start_advertising();
        break;

    case ESP_GATTS_WRITE_EVT:
        /* Update last contact timestamp on any write (FR-19) */
        s_ble.last_contact_us = esp_timer_get_time();

        if (!param->write.is_prep) {
            /* Handle write to control characteristic */
            if (param->write.handle == s_ble.handle_table[IDX_CTRL_VAL]) {
                uart_echo_gatt_command("CTRL", param->write.value, param->write.len);
                handle_control_write(param->write.value, param->write.len);

                /* Send response if needed */
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                               param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            /* Handle write to status CCC (enable/disable notifications) */
            else if (param->write.handle == s_ble.handle_table[IDX_STATUS_CCC]) {
                uart_echo_gatt_command("STATUS_CCC", param->write.value, param->write.len);
                if (param->write.len == 2) {
                    uint16_t ccc_val = param->write.value[0] | (param->write.value[1] << 8);
                    s_ble.notifications_enabled = (ccc_val == 0x0001);
                    ESP_LOGI(TAG, "Status notifications %s",
                             s_ble.notifications_enabled ? "enabled" : "disabled");
                }
            }
            /* Handle write to GalacticStatus CCC (enable/disable notifications) */
            else if (param->write.handle == s_ble.handle_table[IDX_GALACTIC_CCC]) {
                uart_echo_gatt_command("GALACTIC_CCC", param->write.value, param->write.len);
                if (param->write.len == 2) {
                    uint16_t ccc_val = param->write.value[0] | (param->write.value[1] << 8);
                    s_ble.galactic_notifications_enabled = (ccc_val == 0x0001);
                    ESP_LOGI(TAG, "GalacticStatus notifications %s",
                             s_ble.galactic_notifications_enabled ? "enabled" : "disabled");
                }
            }
            /* Handle write to OTA Credentials characteristic */
            else if (param->write.handle == s_ble.handle_table[IDX_OTA_CREDS_VAL]) {
                uart_echo_gatt_command("OTA_CREDS", param->write.value, param->write.len);
                ESP_LOGI(TAG, "OTA credentials received, len=%d", param->write.len);
                ota_mgr_set_credentials(param->write.value, param->write.len);
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                               param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            /* Handle write to OTA URL characteristic */
            else if (param->write.handle == s_ble.handle_table[IDX_OTA_URL_VAL]) {
                uart_echo_gatt_command("OTA_URL", param->write.value, param->write.len);
                ESP_LOGI(TAG, "OTA URL received, len=%d", param->write.len);
                ota_mgr_set_url(param->write.value, param->write.len);
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                               param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            /* Handle write to OTA Control characteristic */
            else if (param->write.handle == s_ble.handle_table[IDX_OTA_CTRL_VAL]) {
                uart_echo_gatt_command("OTA_CTRL", param->write.value, param->write.len);
                if (param->write.len >= 1) {
                    uint8_t cmd = param->write.value[0];
                    uint8_t val = (param->write.len >= 2) ? param->write.value[1] : 0;
                    ESP_LOGI(TAG, "OTA command: CMD=0x%02X, VAL=0x%02X", cmd, val);
                    ota_mgr_execute_command(cmd, val);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                               param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            /* Handle write to OTA Status CCC (enable/disable notifications) */
            else if (param->write.handle == s_ble.handle_table[IDX_OTA_STATUS_CCC]) {
                uart_echo_gatt_command("OTA_STATUS_CCC", param->write.value, param->write.len);
                if (param->write.len == 2) {
                    uint16_t ccc_val = param->write.value[0] | (param->write.value[1] << 8);
                    s_ble.ota_notifications_enabled = (ccc_val == 0x0001);
                    ESP_LOGI(TAG, "OTA Status notifications %s",
                             s_ble.ota_notifications_enabled ? "enabled" : "disabled");
                }
            }
        }
        break;

    case ESP_GATTS_READ_EVT:
        /* Update last contact timestamp on any read (FR-19) */
        s_ble.last_contact_us = esp_timer_get_time();
        /* Auto-response handles reads, but log for debugging */
        ESP_LOGD(TAG, "Read request, handle=%d", param->read.handle);
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU updated to %d", param->mtu.mtu);
        break;

    default:
        break;
    }
}

/*
 * Public API Implementation
 */

esp_err_t ble_gatt_dsp_init(ble_dsp_settings_cb_t settings_changed_cb)
{
    ESP_LOGI(TAG, "Initializing BLE GATT DSP service");

    s_ble.settings_cb = settings_changed_cb;

    /* Initialise flags from NVS (loudness persists across reboots) */
    nvs_dsp_settings_t boot_settings;
    nvs_settings_get(&boot_settings);
    if (boot_settings.loudness) s_dsp_flags |= 0x04;

    /* Initialize UART for serial echo of GATT commands */
    esp_err_t ret = uart_echo_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART echo init failed (non-fatal): %s", esp_err_to_name(ret));
        /* Continue without serial echo - not critical for operation */
    }

    /* Create GalacticStatus notification timer (FR-20: 2x per second) */
    s_ble.galactic_notify_timer = xTimerCreate(
        "galactic_notify",
        pdMS_TO_TICKS(GALACTIC_NOTIFY_INTERVAL_MS),
        pdTRUE,     /* Auto-reload */
        NULL,       /* Timer ID */
        galactic_notify_timer_callback
    );
    if (s_ble.galactic_notify_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create GalacticStatus notification timer");
        return ESP_ERR_NO_MEM;
    }

    /* Register GAP callback */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register GATTS callback */
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register GATT application */
    ret = esp_ble_gatts_app_register(DSP_PROFILE_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Note: MTU is negotiated during connection, default is 23 bytes
     * For larger payloads, client can request MTU exchange */

    ESP_LOGI(TAG, "BLE GATT DSP service initialized");
    return ESP_OK;
}

esp_err_t ble_gatt_dsp_start_advertising(void)
{
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t ble_gatt_dsp_stop_advertising(void)
{
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stop advertising failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t ble_gatt_dsp_notify_status(void)
{
    if (!s_ble.connected || !s_ble.notifications_enabled) {
        return ESP_OK;  /* Not an error, just nothing to do */
    }

    /* Update status value from DSP state */
    update_status_value();

    /* Send notification */
    esp_err_t ret = esp_ble_gatts_send_indicate(s_ble.gatts_if, s_ble.conn_id,
                                                 s_ble.handle_table[IDX_STATUS_VAL],
                                                 sizeof(status_value), status_value, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Send notification failed: %s", esp_err_to_name(ret));
    } else {
        /* Reset last contact on successful notification */
        s_ble.last_contact_us = esp_timer_get_time();
        ESP_LOGD(TAG, "Status notification sent");
    }

    return ret;
}

esp_err_t ble_gatt_dsp_notify_galactic_status(void)
{
    if (!s_ble.connected || !s_ble.galactic_notifications_enabled) {
        return ESP_OK;  /* Not an error, just nothing to do */
    }

    /* Update GalacticStatus value including last contact time */
    update_galactic_status_value();

    /* Send notification */
    esp_err_t ret = esp_ble_gatts_send_indicate(s_ble.gatts_if, s_ble.conn_id,
                                                 s_ble.handle_table[IDX_GALACTIC_VAL],
                                                 sizeof(galactic_value), galactic_value, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "GalacticStatus notification failed: %s", esp_err_to_name(ret));
    } else {
        /* Reset last contact on successful notification */
        s_ble.last_contact_us = esp_timer_get_time();
        ESP_LOGD(TAG, "GalacticStatus notification sent");
    }

    return ret;
}

bool ble_gatt_dsp_is_connected(void)
{
    return s_ble.connected;
}

uint16_t ble_gatt_dsp_get_conn_handle(void)
{
    return s_ble.conn_id;
}

esp_err_t ble_gatt_dsp_notify_ota_status(const uint8_t *status)
{
    if (!s_ble.connected || !s_ble.ota_notifications_enabled) {
        return ESP_OK;  /* Not an error, just nothing to do */
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Copy status to local value */
    memcpy(ota_status_value, status, OTA_STATUS_SIZE);

    /* Update the attribute value in GATT database */
    if (s_ble.gatts_if != ESP_GATT_IF_NONE && s_ble.handle_table[IDX_OTA_STATUS_VAL] != 0) {
        esp_ble_gatts_set_attr_value(s_ble.handle_table[IDX_OTA_STATUS_VAL],
                                     OTA_STATUS_SIZE, ota_status_value);
    }

    /* Send notification */
    esp_err_t ret = esp_ble_gatts_send_indicate(s_ble.gatts_if, s_ble.conn_id,
                                                 s_ble.handle_table[IDX_OTA_STATUS_VAL],
                                                 OTA_STATUS_SIZE, ota_status_value, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OTA status notification failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "OTA status notification sent: state=%d, progress=%d%%",
                 status[0], status[2]);
    }

    return ret;
}
