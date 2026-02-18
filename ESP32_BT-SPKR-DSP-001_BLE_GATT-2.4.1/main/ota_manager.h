/*
 * OTA Manager for Hybrid BLE+WiFi Updates
 * FSD-DSP-001: Over-The-Air Firmware Updates
 *
 * Receives credentials/URL via BLE, downloads firmware via WiFi.
 * Implements state machine for OTA process with progress reporting.
 *
 * Author: Robin Kluit
 * Date: 2026-01-23
 */

#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* OTA URL maximum length */
#define OTA_URL_MAX_LEN     256

/* OTA States */
typedef enum {
    OTA_STATE_IDLE              = 0x00,  /* Ready for OTA */
    OTA_STATE_CREDS_RECEIVED    = 0x01,  /* WiFi credentials received */
    OTA_STATE_URL_RECEIVED      = 0x02,  /* Firmware URL received */
    OTA_STATE_WIFI_CONNECTING   = 0x03,  /* Connecting to WiFi */
    OTA_STATE_WIFI_CONNECTED    = 0x04,  /* WiFi connected */
    OTA_STATE_DOWNLOADING       = 0x05,  /* Downloading firmware */
    OTA_STATE_VERIFYING         = 0x06,  /* Verifying firmware */
    OTA_STATE_SUCCESS           = 0x07,  /* OTA complete, ready for reboot */
    OTA_STATE_PENDING_VERIFY    = 0x08,  /* Booted new firmware, awaiting validation */
    OTA_STATE_ERROR             = 0xFF,  /* Error occurred */
} ota_state_t;

/* OTA Error Codes */
typedef enum {
    OTA_ERROR_NONE              = 0x00,  /* No error */
    OTA_ERROR_WIFI_CONNECT      = 0x01,  /* WiFi connection failed */
    OTA_ERROR_HTTP_CONNECT      = 0x02,  /* HTTP connection failed */
    OTA_ERROR_HTTP_RESPONSE     = 0x03,  /* HTTP error response */
    OTA_ERROR_DOWNLOAD          = 0x04,  /* Download failed */
    OTA_ERROR_VERIFY            = 0x05,  /* Verification failed */
    OTA_ERROR_WRITE             = 0x06,  /* Flash write failed */
    OTA_ERROR_NO_CREDS          = 0x07,  /* No WiFi credentials */
    OTA_ERROR_NO_URL            = 0x08,  /* No firmware URL */
    OTA_ERROR_INVALID_IMAGE     = 0x09,  /* Invalid firmware image */
    OTA_ERROR_CANCELLED         = 0x0A,  /* OTA cancelled by user */
    OTA_ERROR_ROLLBACK_FAILED   = 0x0B,  /* Rollback failed */
} ota_error_t;

/* OTA Commands (received via BLE) */
#define OTA_CMD_START           0x10  /* Start OTA process */
#define OTA_CMD_CANCEL          0x11  /* Cancel OTA */
#define OTA_CMD_REBOOT          0x12  /* Reboot to new firmware */
#define OTA_CMD_GET_VERSION     0x13  /* Get current firmware version */
#define OTA_CMD_ROLLBACK        0x14  /* Rollback to previous firmware */
#define OTA_CMD_VALIDATE        0x15  /* Mark new firmware as valid */

/*
 * OTA Status structure (8 bytes for BLE notification)
 * Format: [STATE][ERROR][PROGRESS%][DOWNLOADED_KB_L][DOWNLOADED_KB_H][TOTAL_KB_L][TOTAL_KB_H][RSSI]
 */
typedef struct {
    uint8_t state;           /* Current OTA state */
    uint8_t error;           /* Error code (0 if no error) */
    uint8_t progress;        /* Download progress 0-100% */
    uint16_t downloaded_kb;  /* Downloaded size in KB */
    uint16_t total_kb;       /* Total firmware size in KB */
    int8_t rssi;             /* WiFi RSSI (signal strength) */
} __attribute__((packed)) ota_status_t;

/* OTA status callback type */
typedef void (*ota_status_cb_t)(const ota_status_t *status);

/*
 * Initialize OTA manager
 *
 * @param status_cb Callback for OTA status updates (for BLE notifications)
 * @return ESP_OK on success
 */
esp_err_t ota_mgr_init(ota_status_cb_t status_cb);

/*
 * Set WiFi credentials for OTA download
 * Format: SSID (32 bytes) + Password (64 bytes) + Separator (1 byte) + Padding (1 byte) = 98 bytes
 *
 * @param data Credential data (SSID + separator + password)
 * @param len Data length
 * @return ESP_OK on success
 */
esp_err_t ota_mgr_set_credentials(const uint8_t *data, uint16_t len);

/*
 * Set firmware download URL
 *
 * @param data URL string (null-terminated or length-specified)
 * @param len Data length
 * @return ESP_OK on success
 */
esp_err_t ota_mgr_set_url(const uint8_t *data, uint16_t len);

/*
 * Execute OTA command
 *
 * @param cmd Command byte (OTA_CMD_*)
 * @param param Optional parameter byte
 * @return ESP_OK on success
 */
esp_err_t ota_mgr_execute_command(uint8_t cmd, uint8_t param);

/*
 * Get current OTA status
 *
 * @param status Pointer to status structure to fill
 * @return ESP_OK on success
 */
esp_err_t ota_mgr_get_status(ota_status_t *status);

/*
 * Get current OTA state
 *
 * @return Current state
 */
ota_state_t ota_mgr_get_state(void);

/*
 * Check if OTA is in progress
 *
 * @return true if OTA is active (not idle or error)
 */
bool ota_mgr_is_active(void);

/*
 * Check if new firmware is pending validation
 * Should be called on boot to handle pending verification
 *
 * @return true if pending validation
 */
bool ota_mgr_is_pending_verify(void);

/*
 * Get firmware version string
 *
 * @return Version string (e.g., "2.0.0")
 */
const char *ota_mgr_get_version(void);

#ifdef __cplusplus
}
#endif

#endif /* OTA_MANAGER_H */
