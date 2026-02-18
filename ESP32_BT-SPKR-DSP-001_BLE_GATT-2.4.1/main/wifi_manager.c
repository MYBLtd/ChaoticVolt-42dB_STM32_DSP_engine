/*
 * WiFi Manager Implementation
 * FSD-DSP-001: Hybrid OTA via BLE+WiFi
 *
 * Author: Robin Kluit
 * Date: 2026-01-23
 */

#include "wifi_manager.h"
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "WIFI_MGR";

/* Event bits for connection status */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* Maximum connection retry attempts */
#define WIFI_MAX_RETRY      5

/* WiFi manager state */
typedef struct {
    wifi_mgr_state_t state;
    wifi_mgr_event_cb_t event_cb;
    char ssid[WIFI_SSID_MAX_LEN + 1];
    char password[WIFI_PASSWORD_MAX_LEN + 1];
    EventGroupHandle_t event_group;
    int retry_count;
    bool initialized;
    esp_netif_t *sta_netif;
} wifi_mgr_ctx_t;

static wifi_mgr_ctx_t s_wifi = {
    .state = WIFI_MGR_STATE_IDLE,
    .event_cb = NULL,
    .retry_count = 0,
    .initialized = false,
    .sta_netif = NULL,
};

/* Forward declarations */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

/*
 * Notify callback of state change
 */
static void notify_state_change(wifi_mgr_state_t new_state)
{
    s_wifi.state = new_state;
    if (s_wifi.event_cb != NULL) {
        int8_t rssi = wifi_mgr_get_rssi();
        s_wifi.event_cb(new_state, rssi);
    }
}

/*
 * WiFi and IP event handler
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi station started");
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Connected to AP");
            s_wifi.retry_count = 0;
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "Disconnected from AP, reason: %d", event->reason);

            if (s_wifi.state == WIFI_MGR_STATE_CONNECTING) {
                if (s_wifi.retry_count < WIFI_MAX_RETRY) {
                    s_wifi.retry_count++;
                    ESP_LOGI(TAG, "Retrying connection (%d/%d)", s_wifi.retry_count, WIFI_MAX_RETRY);
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(TAG, "Connection failed after %d attempts", WIFI_MAX_RETRY);
                    xEventGroupSetBits(s_wifi.event_group, WIFI_FAIL_BIT);
                    notify_state_change(WIFI_MGR_STATE_FAILED);
                }
            } else {
                notify_state_change(WIFI_MGR_STATE_DISCONNECTED);
            }
            break;
        }

        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
        case IP_EVENT_STA_GOT_IP: {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            xEventGroupSetBits(s_wifi.event_group, WIFI_CONNECTED_BIT);
            notify_state_change(WIFI_MGR_STATE_CONNECTED);
            break;
        }

        case IP_EVENT_STA_LOST_IP:
            ESP_LOGW(TAG, "Lost IP address");
            notify_state_change(WIFI_MGR_STATE_DISCONNECTED);
            break;

        default:
            break;
        }
    }
}

esp_err_t wifi_mgr_init(wifi_mgr_event_cb_t event_cb)
{
    if (s_wifi.initialized) {
        ESP_LOGW(TAG, "WiFi manager already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing WiFi manager");

    s_wifi.event_cb = event_cb;

    /* Create event group for synchronization */
    s_wifi.event_group = xEventGroupCreate();
    if (s_wifi.event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Create default event loop if not already created */
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create default WiFi station */
    s_wifi.sta_netif = esp_netif_create_default_wifi_sta();
    if (s_wifi.sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi station netif");
        return ESP_FAIL;
    }

    /* Initialize WiFi with default config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register event handlers */
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set WiFi mode to station */
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start WiFi */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    s_wifi.initialized = true;
    s_wifi.state = WIFI_MGR_STATE_DISCONNECTED;

    ESP_LOGI(TAG, "WiFi manager initialized");
    return ESP_OK;
}

esp_err_t wifi_mgr_deinit(void)
{
    if (!s_wifi.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing WiFi manager");

    /* Disconnect if connected */
    if (s_wifi.state == WIFI_MGR_STATE_CONNECTED ||
        s_wifi.state == WIFI_MGR_STATE_CONNECTING) {
        esp_wifi_disconnect();
    }

    /* Stop and deinit WiFi */
    esp_wifi_stop();
    esp_wifi_deinit();

    /* Destroy netif */
    if (s_wifi.sta_netif != NULL) {
        esp_netif_destroy_default_wifi(s_wifi.sta_netif);
        s_wifi.sta_netif = NULL;
    }

    /* Delete event group */
    if (s_wifi.event_group != NULL) {
        vEventGroupDelete(s_wifi.event_group);
        s_wifi.event_group = NULL;
    }

    s_wifi.initialized = false;
    s_wifi.state = WIFI_MGR_STATE_IDLE;

    ESP_LOGI(TAG, "WiFi manager deinitialized");
    return ESP_OK;
}

esp_err_t wifi_mgr_set_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(ssid) > WIFI_SSID_MAX_LEN) {
        ESP_LOGE(TAG, "SSID too long (max %d)", WIFI_SSID_MAX_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    if (password != NULL && strlen(password) > WIFI_PASSWORD_MAX_LEN) {
        ESP_LOGE(TAG, "Password too long (max %d)", WIFI_PASSWORD_MAX_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    /* Store credentials */
    strncpy(s_wifi.ssid, ssid, WIFI_SSID_MAX_LEN);
    s_wifi.ssid[WIFI_SSID_MAX_LEN] = '\0';

    if (password != NULL) {
        strncpy(s_wifi.password, password, WIFI_PASSWORD_MAX_LEN);
        s_wifi.password[WIFI_PASSWORD_MAX_LEN] = '\0';
    } else {
        s_wifi.password[0] = '\0';
    }

    ESP_LOGI(TAG, "WiFi credentials set for SSID: %s", s_wifi.ssid);
    return ESP_OK;
}

esp_err_t wifi_mgr_connect(void)
{
    if (!s_wifi.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (strlen(s_wifi.ssid) == 0) {
        ESP_LOGE(TAG, "No credentials set");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_wifi.state == WIFI_MGR_STATE_CONNECTED) {
        ESP_LOGI(TAG, "Already connected");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Connecting to WiFi: %s", s_wifi.ssid);

    /* Configure WiFi station */
    wifi_config_t wifi_config = {0};
    /* Use memcpy with explicit size limits to avoid truncation warnings */
    size_t ssid_len = strlen(s_wifi.ssid);
    if (ssid_len > sizeof(wifi_config.sta.ssid) - 1) {
        ssid_len = sizeof(wifi_config.sta.ssid) - 1;
    }
    memcpy(wifi_config.sta.ssid, s_wifi.ssid, ssid_len);

    size_t pwd_len = strlen(s_wifi.password);
    if (pwd_len > sizeof(wifi_config.sta.password) - 1) {
        pwd_len = sizeof(wifi_config.sta.password) - 1;
    }
    memcpy(wifi_config.sta.password, s_wifi.password, pwd_len);

    wifi_config.sta.threshold.authmode = strlen(s_wifi.password) > 0 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Clear event bits */
    xEventGroupClearBits(s_wifi.event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    /* Reset retry counter */
    s_wifi.retry_count = 0;
    s_wifi.state = WIFI_MGR_STATE_CONNECTING;

    /* Notify callback */
    if (s_wifi.event_cb != NULL) {
        s_wifi.event_cb(WIFI_MGR_STATE_CONNECTING, 0);
    }

    /* Start connection */
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi connection: %s", esp_err_to_name(ret));
        s_wifi.state = WIFI_MGR_STATE_FAILED;
        return ret;
    }

    return ESP_OK;
}

esp_err_t wifi_mgr_disconnect(void)
{
    if (!s_wifi.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Disconnecting from WiFi");

    s_wifi.state = WIFI_MGR_STATE_DISCONNECTED;
    esp_err_t ret = esp_wifi_disconnect();

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi disconnect failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

wifi_mgr_state_t wifi_mgr_get_state(void)
{
    return s_wifi.state;
}

int8_t wifi_mgr_get_rssi(void)
{
    if (s_wifi.state != WIFI_MGR_STATE_CONNECTED) {
        return 0;
    }

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }

    return 0;
}

bool wifi_mgr_is_connected(void)
{
    return s_wifi.state == WIFI_MGR_STATE_CONNECTED;
}
