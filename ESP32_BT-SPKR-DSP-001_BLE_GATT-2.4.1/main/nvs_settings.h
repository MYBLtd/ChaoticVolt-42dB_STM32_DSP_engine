/*
 * NVS Settings Module
 * FSD-DSP-001: Persistent Storage
 *
 * Implements:
 * - FR-12: Persistent storage with write debouncing
 * - Section 12: Fields to store and write policy
 *
 * Author: Robin Kluit
 * Date: 2026-01-20
 */

#ifndef NVS_SETTINGS_H
#define NVS_SETTINGS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

/*
 * Settings structure (Section 12.1)
 */
typedef struct {
    uint8_t preset_id;      /* Current preset (0-3) */
    uint8_t loudness;       /* Loudness enabled (0/1) */
    uint8_t bass_level;     /* Optional: bass boost (0-3) */
    uint8_t treble_level;   /* Optional: treble level (0-2) */
    uint8_t config_version; /* Configuration version for migrations */
} nvs_dsp_settings_t;

/* Current config version */
#define NVS_CONFIG_VERSION  1

/* Debounce time in milliseconds (Section 12.2) */
#define NVS_DEBOUNCE_MS     1500

/*
 * Initialize NVS settings module
 * Loads stored settings or initializes defaults
 *
 * @return ESP_OK on success
 */
esp_err_t nvs_settings_init(void);

/*
 * Load settings from NVS
 *
 * @param settings Pointer to settings structure to fill
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not stored
 */
esp_err_t nvs_settings_load(nvs_dsp_settings_t *settings);

/*
 * Request save of current settings
 * This starts/resets the debounce timer (FR-12)
 * Actual save happens after debounce period without new requests
 */
void nvs_settings_request_save(void);

/*
 * Force immediate save of settings
 * Bypasses debounce (use sparingly)
 *
 * @return ESP_OK on success
 */
esp_err_t nvs_settings_save_now(void);

/*
 * Get current settings (cached in memory)
 *
 * @param settings Pointer to settings structure to fill
 */
void nvs_settings_get(nvs_dsp_settings_t *settings);

/*
 * Update settings in memory and request save
 *
 * @param preset New preset ID
 * @param loudness New loudness state
 */
void nvs_settings_update(uint8_t preset, uint8_t loudness);

/*
 * Check if a save is pending (debounce active)
 *
 * @return true if save is scheduled
 */
bool nvs_settings_save_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* NVS_SETTINGS_H */
