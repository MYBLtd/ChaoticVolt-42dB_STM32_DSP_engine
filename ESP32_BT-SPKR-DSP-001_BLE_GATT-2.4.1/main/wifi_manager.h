/*
 * WiFi Manager for OTA Updates
 * FSD-DSP-001: Hybrid OTA via BLE+WiFi
 *
 * Provides temporary WiFi connection for firmware downloads.
 * WiFi is only used during OTA process, then disconnected.
 *
 * Author: Robin Kluit
 * Date: 2026-01-23
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* WiFi credential limits */
#define WIFI_SSID_MAX_LEN       32
#define WIFI_PASSWORD_MAX_LEN   64

/* WiFi connection states */
typedef enum {
    WIFI_MGR_STATE_IDLE,           /* WiFi not initialized */
    WIFI_MGR_STATE_DISCONNECTED,   /* WiFi ready but not connected */
    WIFI_MGR_STATE_CONNECTING,     /* Connecting to AP */
    WIFI_MGR_STATE_CONNECTED,      /* Connected with IP address */
    WIFI_MGR_STATE_FAILED,         /* Connection failed */
} wifi_mgr_state_t;

/* WiFi event callback type */
typedef void (*wifi_mgr_event_cb_t)(wifi_mgr_state_t state, int8_t rssi);

/*
 * Initialize WiFi manager
 * Sets up WiFi station mode but does not connect
 *
 * @param event_cb Callback for WiFi state changes (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t wifi_mgr_init(wifi_mgr_event_cb_t event_cb);

/*
 * Deinitialize WiFi manager
 * Stops WiFi and frees resources
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_mgr_deinit(void);

/*
 * Set WiFi credentials for connection
 *
 * @param ssid WiFi network name (max 32 bytes)
 * @param password WiFi password (max 64 bytes, can be NULL for open networks)
 * @return ESP_OK on success
 */
esp_err_t wifi_mgr_set_credentials(const char *ssid, const char *password);

/*
 * Connect to WiFi using stored credentials
 * Non-blocking, will trigger callback on state change
 *
 * @return ESP_OK if connection started
 */
esp_err_t wifi_mgr_connect(void);

/*
 * Disconnect from WiFi
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_mgr_disconnect(void);

/*
 * Get current WiFi connection state
 *
 * @return Current state
 */
wifi_mgr_state_t wifi_mgr_get_state(void);

/*
 * Get current WiFi RSSI (signal strength)
 *
 * @return RSSI in dBm, or 0 if not connected
 */
int8_t wifi_mgr_get_rssi(void);

/*
 * Check if WiFi is connected with valid IP
 *
 * @return true if connected
 */
bool wifi_mgr_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */
