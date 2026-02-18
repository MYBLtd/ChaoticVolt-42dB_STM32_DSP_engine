/*
 * NVS Settings Implementation
 * FSD-DSP-001: Persistent Storage with Debouncing
 *
 * Author: Robin Kluit
 * Date: 2026-01-20
 */

#include "nvs_settings.h"
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static const char *TAG = "NVS_SETTINGS";

/* NVS namespace and keys */
#define NVS_NAMESPACE       "dsp_settings"
#define NVS_KEY_PRESET      "preset"
#define NVS_KEY_LOUDNESS    "loudness"
#define NVS_KEY_BASS        "bass"
#define NVS_KEY_TREBLE      "treble"
#define NVS_KEY_VERSION     "version"

/* Module state */
typedef struct {
    nvs_dsp_settings_t settings;
    nvs_handle_t nvs_handle;
    TimerHandle_t debounce_timer;
    bool save_pending;
    bool initialized;
} nvs_state_t;

static nvs_state_t s_nvs = {
    .initialized = false,
    .save_pending = false,
};

/* Forward declarations */
static void debounce_timer_callback(TimerHandle_t timer);
static esp_err_t do_save(void);

/*
 * Default settings
 */
static void set_defaults(nvs_dsp_settings_t *settings)
{
    settings->preset_id = 0;  /* Default preset */
    settings->loudness = 0;
    settings->bass_level = 0;
    settings->treble_level = 0;
    settings->config_version = NVS_CONFIG_VERSION;
}

/*
 * Debounce timer callback - actually saves settings
 */
static void debounce_timer_callback(TimerHandle_t timer)
{
    (void)timer;
    ESP_LOGI(TAG, "Debounce complete, saving settings");
    do_save();
    s_nvs.save_pending = false;
}

/*
 * Perform actual NVS write
 */
static esp_err_t do_save(void)
{
    if (!s_nvs.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    ret = nvs_set_u8(s_nvs.nvs_handle, NVS_KEY_PRESET, s_nvs.settings.preset_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save preset: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u8(s_nvs.nvs_handle, NVS_KEY_LOUDNESS, s_nvs.settings.loudness);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save loudness: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u8(s_nvs.nvs_handle, NVS_KEY_BASS, s_nvs.settings.bass_level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save bass: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u8(s_nvs.nvs_handle, NVS_KEY_TREBLE, s_nvs.settings.treble_level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save treble: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u8(s_nvs.nvs_handle, NVS_KEY_VERSION, s_nvs.settings.config_version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save version: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Commit changes to flash */
    ret = nvs_commit(s_nvs.nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Settings saved: preset=%d, loudness=%d",
             s_nvs.settings.preset_id, s_nvs.settings.loudness);

    return ESP_OK;
}

/*
 * Public API Implementation
 */

esp_err_t nvs_settings_init(void)
{
    ESP_LOGI(TAG, "Initializing NVS settings");

    /* Open NVS namespace */
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs.nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Try to load existing settings */
    ret = nvs_settings_load(&s_nvs.settings);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No stored settings, using defaults");
        set_defaults(&s_nvs.settings);
        /* Save defaults to NVS */
        do_save();
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load settings: %s", esp_err_to_name(ret));
        set_defaults(&s_nvs.settings);
    } else {
        /* Check config version and migrate if needed */
        if (s_nvs.settings.config_version != NVS_CONFIG_VERSION) {
            ESP_LOGW(TAG, "Config version mismatch (stored=%d, current=%d), resetting",
                     s_nvs.settings.config_version, NVS_CONFIG_VERSION);
            set_defaults(&s_nvs.settings);
            do_save();
        }
    }

    /* Create debounce timer */
    s_nvs.debounce_timer = xTimerCreate("nvs_debounce",
                                         pdMS_TO_TICKS(NVS_DEBOUNCE_MS),
                                         pdFALSE,  /* One-shot */
                                         NULL,
                                         debounce_timer_callback);
    if (s_nvs.debounce_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create debounce timer");
        return ESP_ERR_NO_MEM;
    }

    s_nvs.initialized = true;

    ESP_LOGI(TAG, "NVS settings initialized: preset=%d, loudness=%d",
             s_nvs.settings.preset_id, s_nvs.settings.loudness);

    return ESP_OK;
}

esp_err_t nvs_settings_load(nvs_dsp_settings_t *settings)
{
    if (settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t value;

    /* Load preset */
    ret = nvs_get_u8(s_nvs.nvs_handle, NVS_KEY_PRESET, &value);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ret;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read preset: %s", esp_err_to_name(ret));
        return ret;
    }
    settings->preset_id = value;

    /* Load loudness */
    ret = nvs_get_u8(s_nvs.nvs_handle, NVS_KEY_LOUDNESS, &value);
    if (ret == ESP_OK) {
        settings->loudness = value;
    } else {
        settings->loudness = 0;
    }

    /* Load bass */
    ret = nvs_get_u8(s_nvs.nvs_handle, NVS_KEY_BASS, &value);
    if (ret == ESP_OK) {
        settings->bass_level = value;
    } else {
        settings->bass_level = 0;
    }

    /* Load treble */
    ret = nvs_get_u8(s_nvs.nvs_handle, NVS_KEY_TREBLE, &value);
    if (ret == ESP_OK) {
        settings->treble_level = value;
    } else {
        settings->treble_level = 0;
    }

    /* Load version */
    ret = nvs_get_u8(s_nvs.nvs_handle, NVS_KEY_VERSION, &value);
    if (ret == ESP_OK) {
        settings->config_version = value;
    } else {
        settings->config_version = 0;
    }

    return ESP_OK;
}

void nvs_settings_request_save(void)
{
    if (!s_nvs.initialized) {
        return;
    }

    /* Reset/start debounce timer */
    if (xTimerIsTimerActive(s_nvs.debounce_timer)) {
        xTimerReset(s_nvs.debounce_timer, 0);
    } else {
        xTimerStart(s_nvs.debounce_timer, 0);
    }

    s_nvs.save_pending = true;
    ESP_LOGD(TAG, "Save requested, debounce timer started");
}

esp_err_t nvs_settings_save_now(void)
{
    if (!s_nvs.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Stop debounce timer if active */
    if (xTimerIsTimerActive(s_nvs.debounce_timer)) {
        xTimerStop(s_nvs.debounce_timer, 0);
    }

    s_nvs.save_pending = false;
    return do_save();
}

void nvs_settings_get(nvs_dsp_settings_t *settings)
{
    if (settings == NULL) {
        return;
    }
    memcpy(settings, &s_nvs.settings, sizeof(nvs_dsp_settings_t));
}

void nvs_settings_update(uint8_t preset, uint8_t loudness)
{
    /* Update preset if not 0xFF (sentinel for "no change") */
    if (preset != 0xFF) {
        if (preset >= 4) {
            preset = 0;  /* Default preset */
        }
        s_nvs.settings.preset_id = preset;
    }

    /* Update loudness if not 0xFF (sentinel for "no change") */
    if (loudness != 0xFF) {
        s_nvs.settings.loudness = loudness ? 1 : 0;
    }

    /* Request debounced save */
    nvs_settings_request_save();
}

bool nvs_settings_save_pending(void)
{
    return s_nvs.save_pending;
}
