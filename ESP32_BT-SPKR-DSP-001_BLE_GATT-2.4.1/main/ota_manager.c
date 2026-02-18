/*
 * OTA Manager Implementation
 * FSD-DSP-001: Hybrid OTA via BLE+WiFi
 *
 * Author: Robin Kluit
 * Date: 2026-01-23
 */

#include "ota_manager.h"
#include "wifi_manager.h"
#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_app_format.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "OTA_MGR";

/* Firmware version */
#define FIRMWARE_VERSION    "2.3.0"

/* OTA task configuration */
#define OTA_TASK_STACK_SIZE     8192
#define OTA_TASK_PRIORITY       5

/* HTTP buffer size */
#define OTA_HTTP_BUFFER_SIZE    1024

/* WiFi connection timeout (ms) */
#define WIFI_CONNECT_TIMEOUT_MS 30000

/* OTA manager context */
typedef struct {
    ota_state_t state;
    ota_error_t error;
    ota_status_cb_t status_cb;
    char ssid[WIFI_SSID_MAX_LEN + 1];
    char password[WIFI_PASSWORD_MAX_LEN + 1];
    char url[OTA_URL_MAX_LEN + 1];
    uint8_t progress;
    uint32_t downloaded_bytes;
    uint32_t total_bytes;
    TaskHandle_t ota_task_handle;
    SemaphoreHandle_t mutex;
    bool cancel_requested;
    bool initialized;
} ota_mgr_ctx_t;

static ota_mgr_ctx_t s_ota = {
    .state = OTA_STATE_IDLE,
    .error = OTA_ERROR_NONE,
    .status_cb = NULL,
    .progress = 0,
    .downloaded_bytes = 0,
    .total_bytes = 0,
    .ota_task_handle = NULL,
    .mutex = NULL,
    .cancel_requested = false,
    .initialized = false,
};

/* Forward declarations */
static void ota_task(void *arg);
static void wifi_event_callback(wifi_mgr_state_t state, int8_t rssi);
static void notify_status_update(void);
static void set_state(ota_state_t state);
static void set_error(ota_error_t error);

/*
 * Set OTA state with thread safety
 */
static void set_state(ota_state_t state)
{
    if (s_ota.mutex != NULL) {
        xSemaphoreTake(s_ota.mutex, portMAX_DELAY);
    }
    s_ota.state = state;
    if (state != OTA_STATE_ERROR) {
        s_ota.error = OTA_ERROR_NONE;
    }
    if (s_ota.mutex != NULL) {
        xSemaphoreGive(s_ota.mutex);
    }
    notify_status_update();
}

/*
 * Set OTA error with thread safety
 */
static void set_error(ota_error_t error)
{
    if (s_ota.mutex != NULL) {
        xSemaphoreTake(s_ota.mutex, portMAX_DELAY);
    }
    s_ota.error = error;
    s_ota.state = OTA_STATE_ERROR;
    if (s_ota.mutex != NULL) {
        xSemaphoreGive(s_ota.mutex);
    }
    notify_status_update();
}

/*
 * Notify status callback
 */
static void notify_status_update(void)
{
    if (s_ota.status_cb != NULL) {
        ota_status_t status;
        ota_mgr_get_status(&status);
        s_ota.status_cb(&status);
    }
}

/*
 * WiFi event callback for OTA process
 */
static void wifi_event_callback(wifi_mgr_state_t state, int8_t rssi)
{
    ESP_LOGI(TAG, "WiFi state: %d, RSSI: %d", state, rssi);

    switch (state) {
    case WIFI_MGR_STATE_CONNECTED:
        if (s_ota.state == OTA_STATE_WIFI_CONNECTING) {
            set_state(OTA_STATE_WIFI_CONNECTED);
        }
        break;

    case WIFI_MGR_STATE_FAILED:
        if (s_ota.state == OTA_STATE_WIFI_CONNECTING) {
            set_error(OTA_ERROR_WIFI_CONNECT);
        }
        break;

    case WIFI_MGR_STATE_DISCONNECTED:
        if (s_ota.state == OTA_STATE_DOWNLOADING) {
            set_error(OTA_ERROR_DOWNLOAD);
        }
        break;

    default:
        break;
    }
}

/*
 * HTTP event handler for progress tracking
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP connected");
        break;

    case HTTP_EVENT_HEADER_SENT:
        break;

    case HTTP_EVENT_ON_HEADER:
        if (strcasecmp(evt->header_key, "Content-Length") == 0) {
            s_ota.total_bytes = atoi(evt->header_value);
            ESP_LOGI(TAG, "Firmware size: %lu bytes", s_ota.total_bytes);
        }
        break;

    case HTTP_EVENT_ON_DATA:
        /* Data received, update progress */
        break;

    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP download finished");
        break;

    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP disconnected");
        break;

    case HTTP_EVENT_REDIRECT:
        ESP_LOGI(TAG, "HTTP redirect");
        break;

    default:
        break;
    }
    return ESP_OK;
}

/*
 * OTA download task
 */
static void ota_task(void *arg)
{
    ESP_LOGI(TAG, "OTA task started");

    /* Initialize WiFi manager */
    esp_err_t ret = wifi_mgr_init(wifi_event_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        set_error(OTA_ERROR_WIFI_CONNECT);
        goto cleanup;
    }

    /* Set WiFi credentials */
    ret = wifi_mgr_set_credentials(s_ota.ssid, s_ota.password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi credentials set failed");
        set_error(OTA_ERROR_NO_CREDS);
        goto cleanup;
    }

    /* Connect to WiFi */
    set_state(OTA_STATE_WIFI_CONNECTING);
    ret = wifi_mgr_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed");
        set_error(OTA_ERROR_WIFI_CONNECT);
        goto cleanup;
    }

    /* Wait for WiFi connection with timeout */
    uint32_t timeout = WIFI_CONNECT_TIMEOUT_MS;
    while (!wifi_mgr_is_connected() && timeout > 0 && !s_ota.cancel_requested) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout -= 100;
    }

    if (s_ota.cancel_requested) {
        ESP_LOGI(TAG, "OTA cancelled by user");
        set_error(OTA_ERROR_CANCELLED);
        goto cleanup;
    }

    if (!wifi_mgr_is_connected()) {
        ESP_LOGE(TAG, "WiFi connection timeout");
        set_error(OTA_ERROR_WIFI_CONNECT);
        goto cleanup;
    }

    /* Start firmware download */
    ESP_LOGI(TAG, "Starting OTA from: %s", s_ota.url);
    set_state(OTA_STATE_DOWNLOADING);

    /* Configure HTTP client */
    esp_http_client_config_t http_config = {
        .url = s_ota.url,
        .event_handler = http_event_handler,
        .buffer_size = OTA_HTTP_BUFFER_SIZE,
        .buffer_size_tx = OTA_HTTP_BUFFER_SIZE,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };

    /* Configure OTA */
    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    esp_https_ota_handle_t ota_handle = NULL;
    ret = esp_https_ota_begin(&ota_config, &ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(ret));
        set_error(OTA_ERROR_HTTP_CONNECT);
        goto cleanup;
    }

    /* Get image size */
    s_ota.total_bytes = esp_https_ota_get_image_size(ota_handle);
    ESP_LOGI(TAG, "Firmware image size: %lu bytes", s_ota.total_bytes);

    /* Download firmware with progress tracking */
    while (1) {
        if (s_ota.cancel_requested) {
            ESP_LOGI(TAG, "OTA cancelled during download");
            esp_https_ota_abort(ota_handle);
            set_error(OTA_ERROR_CANCELLED);
            goto cleanup;
        }

        ret = esp_https_ota_perform(ota_handle);
        if (ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }

        /* Update progress */
        s_ota.downloaded_bytes = esp_https_ota_get_image_len_read(ota_handle);
        if (s_ota.total_bytes > 0) {
            s_ota.progress = (uint8_t)((s_ota.downloaded_bytes * 100) / s_ota.total_bytes);
        }
        notify_status_update();

        ESP_LOGD(TAG, "Download progress: %d%% (%lu/%lu)",
                 s_ota.progress, s_ota.downloaded_bytes, s_ota.total_bytes);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA download failed: %s", esp_err_to_name(ret));
        esp_https_ota_abort(ota_handle);
        set_error(OTA_ERROR_DOWNLOAD);
        goto cleanup;
    }

    /* Verify firmware image */
    set_state(OTA_STATE_VERIFYING);
    ESP_LOGI(TAG, "Verifying firmware image...");

    if (!esp_https_ota_is_complete_data_received(ota_handle)) {
        ESP_LOGE(TAG, "Incomplete firmware image");
        esp_https_ota_abort(ota_handle);
        set_error(OTA_ERROR_VERIFY);
        goto cleanup;
    }

    /* Finish OTA and set boot partition */
    ret = esp_https_ota_finish(ota_handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Firmware validation failed");
            set_error(OTA_ERROR_INVALID_IMAGE);
        } else {
            ESP_LOGE(TAG, "OTA finish failed: %s", esp_err_to_name(ret));
            set_error(OTA_ERROR_WRITE);
        }
        goto cleanup;
    }

    /* OTA successful */
    s_ota.progress = 100;
    set_state(OTA_STATE_SUCCESS);
    ESP_LOGI(TAG, "OTA completed successfully! Ready for reboot.");

cleanup:
    /* Disconnect WiFi */
    wifi_mgr_disconnect();
    wifi_mgr_deinit();

    /* Clear task handle */
    s_ota.ota_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t ota_mgr_init(ota_status_cb_t status_cb)
{
    if (s_ota.initialized) {
        ESP_LOGW(TAG, "OTA manager already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing OTA manager");

    s_ota.status_cb = status_cb;

    /* Create mutex for thread safety */
    s_ota.mutex = xSemaphoreCreateMutex();
    if (s_ota.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Check if booted from new firmware pending validation */
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGW(TAG, "Running firmware pending validation");
            s_ota.state = OTA_STATE_PENDING_VERIFY;
        }
    }

    s_ota.initialized = true;
    ESP_LOGI(TAG, "OTA manager initialized, firmware: %s", FIRMWARE_VERSION);

    return ESP_OK;
}

esp_err_t ota_mgr_set_credentials(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Parse credentials: format is "SSID\0PASSWORD" or "SSID:PASSWORD" */
    /* Find separator (null byte or colon) */
    const char *str = (const char *)data;
    size_t ssid_len = 0;

    for (size_t i = 0; i < len; i++) {
        if (data[i] == '\0' || data[i] == ':') {
            ssid_len = i;
            break;
        }
    }

    if (ssid_len == 0 || ssid_len >= len) {
        /* Try to interpret as just SSID with remaining as password */
        ssid_len = strnlen(str, len);
        if (ssid_len == len) {
            ESP_LOGE(TAG, "Invalid credential format");
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* Copy SSID */
    if (ssid_len > WIFI_SSID_MAX_LEN) {
        ssid_len = WIFI_SSID_MAX_LEN;
    }
    memcpy(s_ota.ssid, data, ssid_len);
    s_ota.ssid[ssid_len] = '\0';

    /* Copy password (skip separator) */
    size_t pwd_offset = ssid_len + 1;
    size_t pwd_len = 0;
    if (pwd_offset < len) {
        pwd_len = len - pwd_offset;
        if (pwd_len > WIFI_PASSWORD_MAX_LEN) {
            pwd_len = WIFI_PASSWORD_MAX_LEN;
        }
        memcpy(s_ota.password, data + pwd_offset, pwd_len);
    }
    s_ota.password[pwd_len] = '\0';

    ESP_LOGI(TAG, "WiFi credentials set: SSID='%s', password length=%d",
             s_ota.ssid, (int)strlen(s_ota.password));

    set_state(OTA_STATE_CREDS_RECEIVED);
    return ESP_OK;
}

esp_err_t ota_mgr_set_url(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Copy URL, ensuring null termination */
    size_t url_len = len;
    if (url_len > OTA_URL_MAX_LEN) {
        url_len = OTA_URL_MAX_LEN;
    }

    memcpy(s_ota.url, data, url_len);
    s_ota.url[url_len] = '\0';

    /* Remove trailing whitespace/null */
    while (url_len > 0 && (s_ota.url[url_len - 1] == '\0' ||
                          s_ota.url[url_len - 1] == ' ' ||
                          s_ota.url[url_len - 1] == '\n' ||
                          s_ota.url[url_len - 1] == '\r')) {
        s_ota.url[--url_len] = '\0';
    }

    ESP_LOGI(TAG, "Firmware URL set: %s", s_ota.url);

    set_state(OTA_STATE_URL_RECEIVED);
    return ESP_OK;
}

esp_err_t ota_mgr_execute_command(uint8_t cmd, uint8_t param)
{
    ESP_LOGI(TAG, "OTA command: 0x%02X, param: 0x%02X", cmd, param);

    switch (cmd) {
    case OTA_CMD_START:
        if (strlen(s_ota.ssid) == 0) {
            ESP_LOGE(TAG, "No WiFi credentials set");
            set_error(OTA_ERROR_NO_CREDS);
            return ESP_ERR_INVALID_STATE;
        }
        if (strlen(s_ota.url) == 0) {
            ESP_LOGE(TAG, "No firmware URL set");
            set_error(OTA_ERROR_NO_URL);
            return ESP_ERR_INVALID_STATE;
        }
        if (s_ota.ota_task_handle != NULL) {
            ESP_LOGW(TAG, "OTA already in progress");
            return ESP_ERR_INVALID_STATE;
        }

        /* Reset state */
        s_ota.progress = 0;
        s_ota.downloaded_bytes = 0;
        s_ota.total_bytes = 0;
        s_ota.cancel_requested = false;

        /* Create OTA task */
        BaseType_t ret = xTaskCreate(ota_task, "ota_task", OTA_TASK_STACK_SIZE,
                                     NULL, OTA_TASK_PRIORITY, &s_ota.ota_task_handle);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create OTA task");
            set_error(OTA_ERROR_DOWNLOAD);
            return ESP_ERR_NO_MEM;
        }
        break;

    case OTA_CMD_CANCEL:
        if (s_ota.ota_task_handle != NULL) {
            ESP_LOGI(TAG, "Cancelling OTA...");
            s_ota.cancel_requested = true;
        } else {
            set_state(OTA_STATE_IDLE);
        }
        break;

    case OTA_CMD_REBOOT:
        if (s_ota.state == OTA_STATE_SUCCESS || s_ota.state == OTA_STATE_PENDING_VERIFY) {
            ESP_LOGI(TAG, "Rebooting to new firmware...");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        } else {
            ESP_LOGW(TAG, "Cannot reboot: OTA not complete");
            return ESP_ERR_INVALID_STATE;
        }
        break;

    case OTA_CMD_GET_VERSION:
        ESP_LOGI(TAG, "Firmware version: %s", FIRMWARE_VERSION);
        notify_status_update();
        break;

    case OTA_CMD_ROLLBACK:
        ESP_LOGI(TAG, "Rolling back to previous firmware...");
        if (esp_ota_mark_app_invalid_rollback_and_reboot() != ESP_OK) {
            ESP_LOGE(TAG, "Rollback failed");
            set_error(OTA_ERROR_ROLLBACK_FAILED);
            return ESP_FAIL;
        }
        break;

    case OTA_CMD_VALIDATE:
        if (s_ota.state == OTA_STATE_PENDING_VERIFY) {
            ESP_LOGI(TAG, "Marking firmware as valid");
            esp_err_t ret = esp_ota_mark_app_valid_cancel_rollback();
            if (ret == ESP_OK) {
                set_state(OTA_STATE_IDLE);
                ESP_LOGI(TAG, "Firmware validated successfully");
            } else {
                ESP_LOGE(TAG, "Failed to validate firmware: %s", esp_err_to_name(ret));
                return ret;
            }
        } else {
            ESP_LOGW(TAG, "No pending verification");
        }
        break;

    default:
        ESP_LOGW(TAG, "Unknown OTA command: 0x%02X", cmd);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t ota_mgr_get_status(ota_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ota.mutex != NULL) {
        xSemaphoreTake(s_ota.mutex, portMAX_DELAY);
    }

    status->state = s_ota.state;
    status->error = s_ota.error;
    status->progress = s_ota.progress;
    status->downloaded_kb = (uint16_t)(s_ota.downloaded_bytes / 1024);
    status->total_kb = (uint16_t)(s_ota.total_bytes / 1024);
    status->rssi = wifi_mgr_is_connected() ? wifi_mgr_get_rssi() : 0;

    if (s_ota.mutex != NULL) {
        xSemaphoreGive(s_ota.mutex);
    }

    return ESP_OK;
}

ota_state_t ota_mgr_get_state(void)
{
    return s_ota.state;
}

bool ota_mgr_is_active(void)
{
    return s_ota.state != OTA_STATE_IDLE &&
           s_ota.state != OTA_STATE_ERROR &&
           s_ota.state != OTA_STATE_SUCCESS &&
           s_ota.state != OTA_STATE_PENDING_VERIFY;
}

bool ota_mgr_is_pending_verify(void)
{
    return s_ota.state == OTA_STATE_PENDING_VERIFY;
}

const char *ota_mgr_get_version(void)
{
    return FIRMWARE_VERSION;
}
