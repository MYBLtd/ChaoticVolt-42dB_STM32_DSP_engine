/*
 * BLE GATT DSP Control Service
 * FSD-DSP-001: DSP Control via BLE GATT
 *
 * Implements Section 10 of the FSD:
 * - DSP_CONTROL service with custom 128-bit UUID
 * - CONTROL_WRITE characteristic (Write, Write Without Response)
 * - STATUS_NOTIFY characteristic (Read, Notify)
 * - 2-byte command protocol
 *
 * Author: Robin Kluit
 * Date: 2026-01-20
 */

#ifndef BLE_GATT_DSP_H
#define BLE_GATT_DSP_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BLE GATT UUIDs (Section 10.2)
 * Custom 128-bit UUIDs for DSP Control Service
 *
 * Base UUID: xxxxxxxx-1234-5678-9ABC-DEF012345678
 * Service:   DSP_CTRL (0x0001)
 * Control:   CTRL_WR  (0x0002)
 * Status:    STAT_NTF (0x0003)
 */

/* DSP Control Service UUID: 00000001-1234-5678-9ABC-DEF012345678 */
#define DSP_SERVICE_UUID_128 { \
    0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, \
    0x78, 0x56, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00 \
}

/* Control Write Characteristic UUID: 00000002-1234-5678-9ABC-DEF012345678 */
#define DSP_CONTROL_CHAR_UUID_128 { \
    0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, \
    0x78, 0x56, 0x34, 0x12, 0x02, 0x00, 0x00, 0x00 \
}

/* Status Notify Characteristic UUID: 00000003-1234-5678-9ABC-DEF012345678 */
#define DSP_STATUS_CHAR_UUID_128 { \
    0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, \
    0x78, 0x56, 0x34, 0x12, 0x03, 0x00, 0x00, 0x00 \
}

/* GalacticStatus Characteristic UUID: 00000004-1234-5678-9ABC-DEF012345678 */
#define DSP_GALACTIC_CHAR_UUID_128 { \
    0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, \
    0x78, 0x56, 0x34, 0x12, 0x04, 0x00, 0x00, 0x00 \
}

/* OTA Credentials Characteristic UUID: 00000005-1234-5678-9ABC-DEF012345678 */
#define OTA_CREDS_CHAR_UUID_128 { \
    0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, \
    0x78, 0x56, 0x34, 0x12, 0x05, 0x00, 0x00, 0x00 \
}

/* OTA URL Characteristic UUID: 00000006-1234-5678-9ABC-DEF012345678 */
#define OTA_URL_CHAR_UUID_128 { \
    0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, \
    0x78, 0x56, 0x34, 0x12, 0x06, 0x00, 0x00, 0x00 \
}

/* OTA Control Characteristic UUID: 00000007-1234-5678-9ABC-DEF012345678 */
#define OTA_CONTROL_CHAR_UUID_128 { \
    0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, \
    0x78, 0x56, 0x34, 0x12, 0x07, 0x00, 0x00, 0x00 \
}

/* OTA Status Characteristic UUID: 00000008-1234-5678-9ABC-DEF012345678 */
#define OTA_STATUS_CHAR_UUID_128 { \
    0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, \
    0x78, 0x56, 0x34, 0x12, 0x08, 0x00, 0x00, 0x00 \
}

/*
 * Control Protocol (Section 10.3)
 * Format: [CMD (1 byte)] [VAL (1 byte)]
 */
#define DSP_CMD_SET_PRESET      0x01    /* VAL: 0-3 (preset ID) */
#define DSP_CMD_SET_LOUDNESS    0x02    /* VAL: 0/1 (off/on) */
#define DSP_CMD_GET_STATUS      0x03    /* VAL: 0 (triggers notify) */
#define DSP_CMD_SET_MUTE        0x04    /* VAL: 0/1 (unmute/mute) */
#define DSP_CMD_SET_AUDIO_DUCK  0x05    /* VAL: 0/1 (off/on) - FR-21: panic button, reduces volume */
#define DSP_CMD_SET_NORMALIZER  0x06    /* VAL: 0/1 (off/on) - FR-22: dynamic range compression */
#define DSP_CMD_SET_VOLUME      0x07    /* VAL: 0-100 (volume trim) - FR-24: device volume control */
#define DSP_CMD_SET_BYPASS      0x08    /* VAL: 0/1 (off/on) - Skip EQ, keep safety (debug) */
#define DSP_CMD_SET_BASS_BOOST  0x09    /* VAL: 0/1 (off/on) - Bass boost (+8dB @ 100Hz) */

/*
 * OTA Commands (Section 10.5)
 * Sent to OTA Control characteristic (0x0007)
 */
#define OTA_CMD_START           0x10    /* Start OTA process */
#define OTA_CMD_CANCEL          0x11    /* Cancel OTA */
#define OTA_CMD_REBOOT          0x12    /* Reboot to new firmware */
#define OTA_CMD_GET_VERSION     0x13    /* Get firmware version */
#define OTA_CMD_ROLLBACK        0x14    /* Rollback to previous firmware */
#define OTA_CMD_VALIDATE        0x15    /* Mark new firmware as valid */

/*
 * OTA Characteristic Sizes
 */
#define OTA_CREDS_MAX_SIZE      98      /* SSID (32) + separator (1) + password (64) + padding (1) */
#define OTA_URL_MAX_SIZE        258     /* URL (256) + length prefix (2) */
#define OTA_CONTROL_SIZE        2       /* CMD (1) + param (1) */
#define OTA_STATUS_SIZE         8       /* State, error, progress, sizes, RSSI */

/*
 * Status Payload (Section 10.4)
 * Format: [VER][PRESET][LOUDNESS][FLAGS]
 */
#define DSP_STATUS_PROTOCOL_VERSION 0x01
#define DSP_STATUS_SIZE             4

/*
 * GalacticStatus Payload (FR-18)
 * Format: [VER][PRESET][FLAGS][ENERGY][VOLUME][BATTERY][LAST_CONTACT]
 * Byte 0: Protocol version (0x42)
 * Byte 1: currentQuantumFlavor (preset 0-3)
 * Byte 2: shieldStatus (flags: mute, audio duck, loudness, normalizer)
 * Byte 3: energyCoreLevel (0-100, placeholder)
 * Byte 4: distortionFieldStrength (volume 0-100)
 * Byte 5: Energy core (battery 0-100, placeholder)
 * Byte 6: lastContact (seconds since last BLE interaction, 0-255)
 */
#define DSP_GALACTIC_PROTOCOL_VERSION  0x42
#define DSP_GALACTIC_STATUS_SIZE       7

/*
 * BLE advertising configuration
 */
#define BLE_ADV_INTERVAL_MIN    0xA0    /* 100ms */
#define BLE_ADV_INTERVAL_MAX    0x140   /* 200ms */

/*
 * Get the device name (with MAC suffix for unique identification)
 * Defined in main.c, shared between Classic BT and BLE
 */
extern const char *bt_get_device_name(void);

/*
 * Callback function type for settings changes
 * Called when a BLE write command changes settings
 */
typedef void (*ble_dsp_settings_cb_t)(void);

/*
 * Initialize BLE GATT DSP control service
 *
 * @param settings_changed_cb Callback when settings change (for NVS persistence)
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_dsp_init(ble_dsp_settings_cb_t settings_changed_cb);

/*
 * Start BLE advertising
 * Call after initialization to make device discoverable
 *
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_dsp_start_advertising(void);

/*
 * Stop BLE advertising
 *
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_dsp_stop_advertising(void);

/*
 * Send status notification to connected client
 * Called after settings change or when client requests status
 *
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_dsp_notify_status(void);

/*
 * Send GalacticStatus notification to connected client (FR-18, FR-20)
 * Contains 7-byte payload with extended status information
 *
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_dsp_notify_galactic_status(void);

/*
 * Send OTA status notification to connected client
 * Contains 8-byte payload with OTA progress information
 *
 * @param status OTA status data (8 bytes)
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_dsp_notify_ota_status(const uint8_t *status);

/*
 * Check if a BLE client is connected
 *
 * @return true if connected
 */
bool ble_gatt_dsp_is_connected(void);

/*
 * Get BLE connection handle (for internal use)
 *
 * @return Connection handle, or 0xFFFF if not connected
 */
uint16_t ble_gatt_dsp_get_conn_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_GATT_DSP_H */
