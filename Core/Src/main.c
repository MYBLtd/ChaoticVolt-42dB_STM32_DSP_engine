/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 42dB DSP Engine - I2S Audio Processor with BLE Control
 * @author         : Robin Kluit
 * @date           : 2026-02-04
 ******************************************************************************
 * @attention
 *
 * DSP audio processor: I2S input -> DSP processing -> I2S output
 * Control interface via ESP32 BLE GATT
 *
 * Peripherals:
 *   USART3 (Debug Console - ST-LINK VCP): PD8(TX), PD9(RX) @ 115200
 *   USART2 (ESP32 BLE Control): PD5(TX), PD6(RX) @ 115200
 *   SAI2_A (I2S Input from BM83): PD13(SCK), PD12(FS), PD11(SD) - Slave RX
 *   SAI2_B (I2S Output to DAC): PA0(SD) - Slave TX synced to SAI2_A
 *
 ******************************************************************************
 */

#include "stm32h7xx_hal.h"
#include "build_number.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Audio Configuration -------------------------------------------------------*/
#define AUDIO_SAMPLE_RATE       44100   /* Match ESP32 default */
#define AUDIO_BIT_DEPTH         16      /* 16-bit audio */
#define AUDIO_CHANNELS          2
#define AUDIO_BUFFER_SAMPLES    256     /* Samples per channel per half-buffer */
#define AUDIO_BUFFER_SIZE       (AUDIO_BUFFER_SAMPLES * AUDIO_CHANNELS)  /* Total samples per half */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;  // ESP32 communication
UART_HandleTypeDef huart3;  // Debug console (VCP)
SAI_HandleTypeDef hsai_rx;  // SAI2_A - I2S Input (Slave RX)
SAI_HandleTypeDef hsai_tx;  // SAI2_B - I2S Output (Slave TX)
DMA_HandleTypeDef hdma_sai_rx;  // DMA for SAI2_A RX
DMA_HandleTypeDef hdma_sai_tx;  // DMA for SAI2_B TX
DAC_HandleTypeDef hdac1;    // DAC1 - IN-13 VU meter (PA4)

/* Audio Buffers - placed in D2 SRAM for DMA1/DMA2 access --------------------*/
/* IMPORTANT: DMA1/DMA2 on STM32H7 can ONLY access D2 SRAM (0x30000000)! */
/* Double buffer: DMA fills one half while we process the other */
/* 16-bit samples to match ESP32 I2S format */
__attribute__((section(".RAM_D2"))) int16_t audio_rx_buffer[AUDIO_BUFFER_SIZE * 2];
__attribute__((section(".RAM_D2"))) int16_t audio_tx_buffer[AUDIO_BUFFER_SIZE * 2];

/* Audio state */
volatile uint8_t audio_rx_half_complete = 0;
volatile uint8_t audio_rx_full_complete = 0;
volatile uint32_t audio_buffer_count = 0;
volatile uint32_t audio_overrun_count = 0;

/* Ring buffer for received data */
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;
uint8_t rx_byte;  // Single byte for UART interrupt

/* Line buffer for GATT commands from ESP32 */
#define LINE_BUFFER_SIZE 128
char line_buffer[LINE_BUFFER_SIZE];
uint16_t line_pos = 0;

/* GATT command handler - called when complete line received */
static void Process_GATT_Command(const char *line);

/* ==========================================================================
 * DSP Processing State
 * ========================================================================== */

/* DSP feature flags */
static volatile uint8_t dsp_loudness_enabled = 0;
static volatile uint8_t dsp_mute_enabled = 0;
static volatile uint8_t dsp_bypass_enabled = 0;
static volatile uint8_t dsp_bass_boost_enabled = 0;
static volatile uint8_t dsp_duck_enabled = 0;
static volatile uint8_t dsp_normalizer_enabled = 0;

/* ==========================================================================
 * Presets (EQ profiles)
 * 0 = OFFICE: Neutral with slight high clarity
 * 1 = FULL: Enhanced bass and treble ("fun" curve)
 * 2 = NIGHT: Reduced bass, volume capped at 60%
 * 3 = SPEECH: Voice clarity, high-pass + mid boost
 * ========================================================================== */
typedef enum {
    PRESET_OFFICE = 0,
    PRESET_FULL   = 1,
    PRESET_NIGHT  = 2,
    PRESET_SPEECH = 3
} DspPreset;

static volatile uint8_t current_preset = PRESET_OFFICE;
#define PRESET_NIGHT_VOLUME_CAP 60  /* Max trim for NIGHT mode */

/* ==========================================================================
 * Volume / Device Trim
 * ==========================================================================
 * Device Trim is separate from phone volume (hardware buttons).
 * Used for: headroom management, safe caps, ducking, protection.
 *
 * Mapping (perceptual/logarithmic):
 *   100 →  0 dB (unity)
 *    80 → -6 dB
 *    60 → -12 dB
 *    40 → -20 dB
 *    20 → -35 dB
 *     0 → -60 dB (near-mute)
 * ========================================================================== */

static volatile uint8_t device_trim_value = 100; /* 0-100, default unity (0dB) */
static float current_gain = 0.3548f;             /* Current gain (smoothed) */
static float target_gain = 0.3548f;              /* Target gain from trim value */

/* Fixed headroom to accommodate all DSP effects without volume jumps
 * -9dB provides room for:
 *   - Loudness: +6dB bass boost
 *   - Bass Boost: +6-8dB
 *   - Combined effects with some margin
 * This headroom is ALWAYS applied, so enabling/disabling effects
 * doesn't cause perceived volume changes (effects "fill" the headroom)
 */
#define FIXED_HEADROOM_DB   -9.0f
#define FIXED_HEADROOM_LIN  0.3548f  /* 10^(-9/20) */

/* Smooth ramping coefficient (~10ms ramp @ 44.1kHz with 256 sample buffers) */
#define GAIN_RAMP_COEFF     0.05f

/**
 * @brief  Convert trim value (0-100) to linear gain
 *         Uses piecewise linear approximation of logarithmic curve
 *         Includes fixed headroom for DSP effects
 */
static float trim_to_gain(uint8_t trim)
{
    /* Piecewise linear dB mapping, then convert to linear */
    float db;
    if (trim >= 100) {
        db = 0.0f;
    } else if (trim >= 80) {
        /* 100→0dB, 80→-6dB: slope = -6/20 = -0.3 dB per unit */
        db = (trim - 100) * 0.3f;
    } else if (trim >= 60) {
        /* 80→-6dB, 60→-12dB: slope = -6/20 = -0.3 dB per unit */
        db = -6.0f + (trim - 80) * 0.3f;
    } else if (trim >= 40) {
        /* 60→-12dB, 40→-20dB: slope = -8/20 = -0.4 dB per unit */
        db = -12.0f + (trim - 60) * 0.4f;
    } else if (trim >= 20) {
        /* 40→-20dB, 20→-35dB: slope = -15/20 = -0.75 dB per unit */
        db = -20.0f + (trim - 40) * 0.75f;
    } else {
        /* 20→-35dB, 0→-60dB: slope = -25/20 = -1.25 dB per unit */
        db = -35.0f + (trim - 20) * 1.25f;
    }

    /* Add fixed headroom and convert dB to linear */
    db += FIXED_HEADROOM_DB;
    return powf(10.0f, db / 20.0f);
}

/**
 * @brief  Calculate target gain from trim value
 *         Headroom is fixed, not dependent on active effects
 *         NIGHT preset applies volume cap
 */
static void update_target_gain(void)
{
    uint8_t effective_trim = device_trim_value;

    /* Apply volume cap for NIGHT mode */
    if (current_preset == PRESET_NIGHT && effective_trim > PRESET_NIGHT_VOLUME_CAP) {
        effective_trim = PRESET_NIGHT_VOLUME_CAP;
    }

    target_gain = trim_to_gain(effective_trim);
}

/* Biquad filter state for loudness (stereo - L and R channels) */
/* Low-shelf filter: boost bass frequencies (+6dB @ 100Hz) */
typedef struct {
    float x1, x2;  /* Input history */
    float y1, y2;  /* Output history */
} BiquadState;

static BiquadState loudness_state_L = {0};
static BiquadState loudness_state_R = {0};

/* Biquad coefficients for low-shelf filter
 * Calculated for: Fs=44100Hz, Fc=150Hz, Gain=+6dB, Q=0.707
 * Using Audio EQ Cookbook formula (Robert Bristow-Johnson) */
static const float loudness_b0 = 1.005260f;
static const float loudness_b1 = -1.974408f;
static const float loudness_b2 = 0.969786f;
static const float loudness_a1 = -1.974567f;
static const float loudness_a2 = 0.974887f;

/* ==========================================================================
 * Bass Boost Filter (+8dB @ 100Hz)
 * ========================================================================== */
static BiquadState bass_state_L = {0};
static BiquadState bass_state_R = {0};

/* Bass Boost: Low-shelf +8dB @ 100Hz, S=0.7 */
static const float bass_b0 = 1.005677f;
static const float bass_b1 = -1.980528f;
static const float bass_b2 = 0.975169f;
static const float bass_a1 = -1.980624f;
static const float bass_a2 = 0.980750f;

/* ==========================================================================
 * Duck Mode (-12dB gain reduction)
 * ========================================================================== */
#define DUCK_GAIN_LIN       0.251189f  /* 10^(-12/20) = ~25% volume */
static float duck_current_gain = 1.0f;
static float duck_target_gain = 1.0f;

/* ==========================================================================
 * Normalizer / DRC (Dynamic Range Compression)
 * Tuned for transparent leveling without "pumping"
 * Threshold: -12dB (only compress loud parts)
 * Ratio: 2:1 (gentle compression)
 * Attack: 10ms, Release: 80ms (faster recovery)
 * Makeup: +3dB (subtle boost)
 * ========================================================================== */
#define DRC_THRESHOLD_LIN   0.251189f  /* 10^(-12/20) = -12dB */
#define DRC_RATIO           2.0f       /* Gentle 2:1 ratio */
#define DRC_ATTACK_COEF     0.002268f  /* ~10ms attack */
#define DRC_RELEASE_COEF    0.000284f  /* ~80ms release */
#define DRC_MAKEUP_LIN      1.412538f  /* +3dB makeup gain */

static float drc_envelope = 0.0f;      /* Current envelope level */
static float drc_gain_smooth = 1.0f;   /* Smoothed gain reduction */

/* ==========================================================================
 * IN-13 Nixie Bargraph VU Meter (DAC1 Channel 1 on PA4)
 * Measured control range with MPSA42 + 10K pull-down:
 *   770mV (DAC ~956)  = 0% bar (tube barely on)
 *   1840mV (DAC ~2283) = 100% bar (full length)
 * Below 956: tube off (avoid unreliable glow region)
 * ========================================================================== */
#define VU_DAC_MIN      956     /* 770mV - tube starts */
#define VU_DAC_MAX      2283    /* 1840mV - full bar */
#define VU_DAC_OFF      0       /* Below minimum: tube completely off */
#define VU_PRIME_MS     50      /* Prime burst duration at startup */

static float vu_rms_smooth = 0.0f;     /* Smoothed RMS level */
static uint8_t vu_primed = 0;          /* Flag: prime burst done */

/* ==========================================================================
 * Preset EQ Coefficients
 * ========================================================================== */

/* Preset filter states (up to 2 stages per preset) */
static BiquadState preset_eq1_L = {0};
static BiquadState preset_eq1_R = {0};
static BiquadState preset_eq2_L = {0};
static BiquadState preset_eq2_R = {0};

/* OFFICE: High-shelf +1.5dB @ 6kHz (subtle clarity) */
static const float preset_office_b0 = 1.128557f;
static const float preset_office_b1 = -0.947809f;
static const float preset_office_b2 = 0.264846f;
static const float preset_office_a1 = -0.762145f;
static const float preset_office_a2 = 0.207739f;

/* FULL: Low-shelf +4dB @ 120Hz */
static const float preset_full_bass_b0 = 1.003338f;
static const float preset_full_bass_b1 = -1.974175f;
static const float preset_full_bass_b2 = 0.971200f;
static const float preset_full_bass_a1 = -1.974242f;
static const float preset_full_bass_a2 = 0.974472f;
/* FULL: High-shelf +3dB @ 8kHz */
static const float preset_full_treble_b0 = 1.237922f;
static const float preset_full_treble_b1 = -0.692061f;
static const float preset_full_treble_b2 = 0.184899f;
static const float preset_full_treble_a1 = -0.383231f;
static const float preset_full_treble_a2 = 0.113991f;

/* NIGHT: Low-shelf -3dB @ 150Hz (reduced bass) */
static const float preset_night_b0 = 0.996892f;
static const float preset_night_b1 = -1.960843f;
static const float preset_night_b2 = 0.964328f;
static const float preset_night_a1 = -1.960765f;
static const float preset_night_a2 = 0.961297f;

/* SPEECH: High-pass @ 150Hz */
static const float preset_speech_hp_b0 = 0.985000f;
static const float preset_speech_hp_b1 = -1.969999f;
static const float preset_speech_hp_b2 = 0.985000f;
static const float preset_speech_hp_a1 = -1.969774f;
static const float preset_speech_hp_a2 = 0.970224f;
/* SPEECH: Mid boost +3dB @ 2.5kHz */
static const float preset_speech_mid_b0 = 1.036752f;
static const float preset_speech_mid_b1 = -1.707474f;
static const float preset_speech_mid_b2 = 0.785074f;
static const float preset_speech_mid_a1 = -1.707474f;
static const float preset_speech_mid_a2 = 0.821826f;

/* Apply biquad filter to a single sample (generic version) */
static inline float biquad_process_coef(BiquadState *state, float input,
                                        float b0, float b1, float b2,
                                        float a1, float a2)
{
    float output = b0 * input
                 + b1 * state->x1
                 + b2 * state->x2
                 - a1 * state->y1
                 - a2 * state->y2;

    /* Shift history */
    state->x2 = state->x1;
    state->x1 = input;
    state->y2 = state->y1;
    state->y1 = output;

    return output;
}

/* Loudness filter wrapper */
static inline float loudness_process(BiquadState *state, float input)
{
    return biquad_process_coef(state, input,
                               loudness_b0, loudness_b1, loudness_b2,
                               loudness_a1, loudness_a2);
}

/* Bass Boost filter wrapper */
static inline float bass_process(BiquadState *state, float input)
{
    return biquad_process_coef(state, input,
                               bass_b0, bass_b1, bass_b2,
                               bass_a1, bass_a2);
}

/* Preset EQ processing - applies preset-specific filters */
static inline void preset_process(float *left, float *right)
{
    switch (current_preset)
    {
        case PRESET_OFFICE:
            /* Single stage: subtle high-shelf */
            *left  = biquad_process_coef(&preset_eq1_L, *left,
                        preset_office_b0, preset_office_b1, preset_office_b2,
                        preset_office_a1, preset_office_a2);
            *right = biquad_process_coef(&preset_eq1_R, *right,
                        preset_office_b0, preset_office_b1, preset_office_b2,
                        preset_office_a1, preset_office_a2);
            break;

        case PRESET_FULL:
            /* Two stages: bass boost + treble boost */
            *left  = biquad_process_coef(&preset_eq1_L, *left,
                        preset_full_bass_b0, preset_full_bass_b1, preset_full_bass_b2,
                        preset_full_bass_a1, preset_full_bass_a2);
            *right = biquad_process_coef(&preset_eq1_R, *right,
                        preset_full_bass_b0, preset_full_bass_b1, preset_full_bass_b2,
                        preset_full_bass_a1, preset_full_bass_a2);
            *left  = biquad_process_coef(&preset_eq2_L, *left,
                        preset_full_treble_b0, preset_full_treble_b1, preset_full_treble_b2,
                        preset_full_treble_a1, preset_full_treble_a2);
            *right = biquad_process_coef(&preset_eq2_R, *right,
                        preset_full_treble_b0, preset_full_treble_b1, preset_full_treble_b2,
                        preset_full_treble_a1, preset_full_treble_a2);
            break;

        case PRESET_NIGHT:
            /* Single stage: reduced bass */
            *left  = biquad_process_coef(&preset_eq1_L, *left,
                        preset_night_b0, preset_night_b1, preset_night_b2,
                        preset_night_a1, preset_night_a2);
            *right = biquad_process_coef(&preset_eq1_R, *right,
                        preset_night_b0, preset_night_b1, preset_night_b2,
                        preset_night_a1, preset_night_a2);
            break;

        case PRESET_SPEECH:
            /* Two stages: high-pass + mid boost */
            *left  = biquad_process_coef(&preset_eq1_L, *left,
                        preset_speech_hp_b0, preset_speech_hp_b1, preset_speech_hp_b2,
                        preset_speech_hp_a1, preset_speech_hp_a2);
            *right = biquad_process_coef(&preset_eq1_R, *right,
                        preset_speech_hp_b0, preset_speech_hp_b1, preset_speech_hp_b2,
                        preset_speech_hp_a1, preset_speech_hp_a2);
            *left  = biquad_process_coef(&preset_eq2_L, *left,
                        preset_speech_mid_b0, preset_speech_mid_b1, preset_speech_mid_b2,
                        preset_speech_mid_a1, preset_speech_mid_a2);
            *right = biquad_process_coef(&preset_eq2_R, *right,
                        preset_speech_mid_b0, preset_speech_mid_b1, preset_speech_mid_b2,
                        preset_speech_mid_a1, preset_speech_mid_a2);
            break;
    }
}

/* Reset preset filter states (call when changing presets) */
static void reset_preset_filters(void)
{
    memset(&preset_eq1_L, 0, sizeof(preset_eq1_L));
    memset(&preset_eq1_R, 0, sizeof(preset_eq1_R));
    memset(&preset_eq2_L, 0, sizeof(preset_eq2_L));
    memset(&preset_eq2_R, 0, sizeof(preset_eq2_R));
}

/* DRC/Compressor process - returns smoothed gain factor */
static inline float drc_process(float input_level)
{
    /* Peak detection with attack/release envelope */
    float abs_level = (input_level < 0) ? -input_level : input_level;

    if (abs_level > drc_envelope) {
        /* Attack: fast rise */
        drc_envelope += DRC_ATTACK_COEF * (abs_level - drc_envelope);
    } else {
        /* Release: slow fall */
        drc_envelope += DRC_RELEASE_COEF * (abs_level - drc_envelope);
    }

    /* Calculate target gain */
    float target_drc_gain;
    if (drc_envelope > DRC_THRESHOLD_LIN) {
        /* Above threshold: apply compression */
        float excess_ratio = drc_envelope / DRC_THRESHOLD_LIN;
        float compressed = DRC_THRESHOLD_LIN * powf(excess_ratio, 1.0f / DRC_RATIO);
        target_drc_gain = (compressed / drc_envelope) * DRC_MAKEUP_LIN;
    } else {
        /* Below threshold: just apply makeup gain */
        target_drc_gain = DRC_MAKEUP_LIN;
    }

    /* Smooth the gain changes to avoid artifacts */
    drc_gain_smooth += 0.01f * (target_drc_gain - drc_gain_smooth);

    return drc_gain_smooth;
}

/* Forward declaration needed for functions below */
void Error_Handler(void);

/**
 * @brief  Initialize DAC1 Channel 1 (PA4) for IN-13 VU meter
 */
static void MX_DAC1_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Prime the IN-13 tube with a 100% burst
 *         Ensures gas discharge starts from bottom (via auxiliary cathode)
 */
static void VU_Prime(void)
{
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, VU_DAC_MAX);
    HAL_Delay(VU_PRIME_MS);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, VU_DAC_OFF);
    vu_primed = 1;
}

/**
 * @brief  Update VU meter from audio buffer RMS level
 *         Called from Audio_Process (ISR context) - only updates smoothed RMS
 * @param  rx_buf: Audio buffer (stereo interleaved)
 * @param  samples: Number of samples (L+R)
 */
static void VU_UpdateRMS(int16_t *rx_buf, uint16_t samples)
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < samples; i += 2)
    {
        float s = ((float)rx_buf[i] + (float)rx_buf[i + 1]) * 0.5f;
        sum += s * s;
    }
    float rms = sqrtf(sum / (float)(samples / 2));

    /* Asymmetric smoothing: fast attack, moderate release */
    if (rms > vu_rms_smooth)
        vu_rms_smooth = rms * 0.3f + vu_rms_smooth * 0.7f;
    else
        vu_rms_smooth = rms * 0.10f + vu_rms_smooth * 0.90f;
}

/**
 * @brief  Write smoothed RMS value to DAC (call from main loop, not ISR)
 */
static void VU_WriteDac(void)
{
    if (!vu_primed) return;

    /* Music RMS is typically ~3000-10000 (not 32768 peak).
     * Use -20dB (0.1 = ~3277) as reference for full scale,
     * with sqrt compression for a more natural VU response. */
    float normalized = vu_rms_smooth / 10000.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    /* Below a threshold: turn tube completely off to avoid random spots */
    if (normalized < 0.01f)
    {
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, VU_DAC_OFF);
        return;
    }

    /* Sqrt compression: spreads mid-range, feels more natural */
    normalized = sqrtf(normalized);

    uint16_t dac_val = VU_DAC_MIN + (uint16_t)(normalized * (float)(VU_DAC_MAX - VU_DAC_MIN));
    if (dac_val > VU_DAC_MAX) dac_val = VU_DAC_MAX;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);
}

/* DMA drift detection and auto-recovery */
#define DMA_DRIFT_THRESHOLD     20      /* Max allowed drift in samples */
#define DMA_DRIFT_CHECK_INTERVAL 500    /* Check every 500ms */
static int16_t dma_initial_offset = 0;  /* Captured after startup */
static uint8_t dma_offset_captured = 0; /* Flag: initial offset valid */
static volatile uint32_t dma_resync_count = 0;  /* Total resyncs */

/**
 * @brief  Restart SAI/DMA to resynchronize RX and TX
 *         Called when DMA drift exceeds threshold
 */
static void SAI_DMA_Resync(void)
{
    /* Stop both DMA streams */
    HAL_SAI_DMAStop(&hsai_rx);
    HAL_SAI_DMAStop(&hsai_tx);

    /* Clear errors and flush FIFOs */
    __HAL_SAI_CLEAR_FLAG(&hsai_rx, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET | SAI_FLAG_WCKCFG);
    __HAL_SAI_CLEAR_FLAG(&hsai_tx, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET | SAI_FLAG_WCKCFG);
    SAI2_Block_A->CR2 |= SAI_xCR2_FFLUSH;
    SAI2_Block_B->CR2 |= SAI_xCR2_FFLUSH;

    /* Clear buffers */
    memset(audio_tx_buffer, 0, sizeof(audio_tx_buffer));

    /* Restart DMA */
    HAL_SAI_Receive_DMA(&hsai_rx, (uint8_t *)audio_rx_buffer, AUDIO_BUFFER_SIZE * 2);
    HAL_SAI_Transmit_DMA(&hsai_tx, (uint8_t *)audio_tx_buffer, AUDIO_BUFFER_SIZE * 2);

    /* Reset drift tracking */
    dma_offset_captured = 0;
    dma_resync_count++;
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SAI2_Init(void);
static void MX_DAC1_Init(void);
void Error_Handler(void);

/* Audio processing */
static void Audio_Process(int16_t *rx_buf, int16_t *tx_buf, uint16_t samples);

/* Debug printf redirection to USART3 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();          // DMA must be initialized before peripherals that use it
    MX_USART3_UART_Init();  // Debug console first

    /* Boot message with build number */
    printf("\r\n\r\n*** 42dB DSP Engine (Build #%d) ***\r\n", BUILD_NUMBER);

    printf("[OK] HAL initialized\r\n");
    printf("[OK] Clock configured\r\n");
    printf("[OK] GPIO initialized\r\n");
    printf("[OK] DMA initialized\r\n");
    printf("[OK] USART3 (debug console) initialized\r\n");

    MX_USART2_UART_Init();  // ESP32 BLE control
    printf("[OK] USART2 (ESP32 BLE) initialized\r\n");

    MX_SAI2_Init();  // I2S Audio
    printf("[OK] SAI2 (I2S Audio) initialized\r\n");
    printf("[NOTE] Use 74HCT04 buffer for I2S signals (5V tolerant)\r\n");

    MX_DAC1_Init();  // IN-13 VU meter
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    VU_Prime();
    printf("[OK] DAC1 (IN-13 VU meter on PA4) initialized and primed\r\n");

    /* Print startup banner */
    printf("\r\n");
    printf("========================================\r\n");
    printf("42dB DSP Engine v0.2\r\n");
    printf("========================================\r\n");
    printf("ESP32 BLE: PD5(TX), PD6(RX) @ 115200\r\n");
    printf("Debug:     PD8(TX), PD9(RX) @ 115200\r\n");
    printf("I2S Input: PD13(SCK), PD12(FS), PD11(SD)\r\n");
    printf("I2S Output: PA0(SD)\r\n");
    printf("Audio:     %d Hz, %d ch, %d samples/buf\r\n",
           AUDIO_SAMPLE_RATE, AUDIO_CHANNELS, AUDIO_BUFFER_SAMPLES);
    printf("========================================\r\n\r\n");

    /* Start receiving data from ESP32 via interrupt */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    printf("[OK] USART2 RX interrupt enabled\r\n");

    /* Clear audio buffers */
    memset(audio_rx_buffer, 0, sizeof(audio_rx_buffer));
    memset(audio_tx_buffer, 0, sizeof(audio_tx_buffer));

    /* Clear any pending SAI errors and flush FIFOs before starting */
    __HAL_SAI_CLEAR_FLAG(&hsai_rx, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET | SAI_FLAG_WCKCFG);
    __HAL_SAI_CLEAR_FLAG(&hsai_tx, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET | SAI_FLAG_WCKCFG);

    /* Flush FIFOs */
    SAI2_Block_A->CR2 |= SAI_xCR2_FFLUSH;
    SAI2_Block_B->CR2 |= SAI_xCR2_FFLUSH;

    /* Pre-fill TX buffer with silence to avoid garbage at startup */
    memset(audio_tx_buffer, 0, sizeof(audio_tx_buffer));

    /* Start RX DMA first to receive audio data from ESP32/BM83 */
    HAL_StatusTypeDef rx_status = HAL_SAI_Receive_DMA(&hsai_rx, (uint8_t *)audio_rx_buffer,
                                                       AUDIO_BUFFER_SIZE * 2);

    /* Start TX DMA - synchronized to RX for same clock */
    HAL_StatusTypeDef tx_status = HAL_SAI_Transmit_DMA(&hsai_tx, (uint8_t *)audio_tx_buffer,
                                                        AUDIO_BUFFER_SIZE * 2);

    if (rx_status == HAL_OK && tx_status == HAL_OK)
    {
        printf("[OK] SAI2 RX+TX DMA started (passthrough mode)\r\n");
    }
    else
    {
        printf("[WARN] DMA start failed: RX=%d TX=%d\r\n", rx_status, tx_status);
    }

    printf("\r\n[INFO] PASSTHROUGH MODE - Audio from ESP32 to DAC\r\n");
    printf("       RX: BCLK->PD13, LRCK->PD12, SD->PD11\r\n");
    printf("       TX: SD->PA0 (shares BCLK/LRCK from ESP32)\r\n\r\n");

    /* Debug: print SAI and GPIO register status */
    printf("[DEBUG] SAI2_A CR1=0x%08lX SR=0x%08lX\r\n",
           SAI2_Block_A->CR1, SAI2_Block_A->SR);
    printf("[DEBUG] SAI2_B CR1=0x%08lX SR=0x%08lX\r\n",
           SAI2_Block_B->CR1, SAI2_Block_B->SR);
    /* GPIO mode and alternate function for PD11,12,13 */
    printf("[DEBUG] GPIOD MODER=0x%08lX AFR[1]=0x%08lX\r\n",
           GPIOD->MODER, GPIOD->AFR[1]);
    /* GPIO for PA0 (SAI2_B output) */
    printf("[DEBUG] GPIOA MODER=0x%08lX AFR[0]=0x%08lX\r\n",
           GPIOA->MODER, GPIOA->AFR[0]);
    /* Check SAI2 clock source */
    printf("[DEBUG] RCC D2CCIP1R=0x%08lX (SAI23SEL bits 8:6)\r\n",
           RCC->D2CCIP1R);
    /* Read GPIO input values to verify signals */
    printf("[DEBUG] GPIOD IDR=0x%04lX (PD11=%d PD12=%d PD13=%d)\r\n",
           GPIOD->IDR,
           (GPIOD->IDR >> 11) & 1,
           (GPIOD->IDR >> 12) & 1,
           (GPIOD->IDR >> 13) & 1);

    /* Variables for periodic status */
    uint32_t last_status_time = 0;
    uint32_t last_buffer_count = 0;
    uint32_t debug_counter = 0;
    uint32_t last_drift_check = 0;

    /* Infinite loop */
    while (1)
    {
        /* Audio processing happens in RX DMA callbacks for lowest latency */

        /* DMA drift detection - check every 500ms */
        uint32_t now = HAL_GetTick();
        if (now - last_drift_check >= DMA_DRIFT_CHECK_INTERVAL && audio_buffer_count > 100)
        {
            /* Read both NDTR as close together as possible */
            uint32_t rx_ndtr = DMA1_Stream0->NDTR;
            uint32_t tx_ndtr = DMA1_Stream1->NDTR;
            int16_t current_offset = (int16_t)(tx_ndtr - rx_ndtr);

            if (!dma_offset_captured)
            {
                /* Capture initial offset after startup/resync */
                dma_initial_offset = current_offset;
                dma_offset_captured = 1;
            }
            else
            {
                /* Check drift from initial offset (handle wrap-around) */
                int16_t drift = current_offset - dma_initial_offset;
                if (drift > 512) drift -= 1024;
                if (drift < -512) drift += 1024;

                if (drift > DMA_DRIFT_THRESHOLD || drift < -DMA_DRIFT_THRESHOLD)
                {
                    printf("[RESYNC] DMA drift=%d (threshold=%d), restarting SAI/DMA (resync #%lu)\r\n",
                           drift, DMA_DRIFT_THRESHOLD, dma_resync_count + 1);
                    SAI_DMA_Resync();
                }
            }
            last_drift_check = now;
        }

        /* Print status every second */
        if (now - last_status_time >= 1000)
        {
            uint32_t buffers_per_sec = audio_buffer_count - last_buffer_count;
            if (audio_buffer_count > 0)
            {
                printf("[AUDIO] Buffers/sec: %lu, Total: %lu, Overruns: %lu, Resyncs: %lu\r\n",
                       buffers_per_sec, audio_buffer_count, audio_overrun_count, dma_resync_count);
            }
            /* Debug: show SAI and DMA status every 5 seconds */
            debug_counter++;
            if ((debug_counter % 5) == 0)
            {
                /* Read NDTR close together for accurate offset */
                uint32_t rx_ndtr = DMA1_Stream0->NDTR;
                uint32_t tx_ndtr = DMA1_Stream1->NDTR;
                int16_t offset = (int16_t)(tx_ndtr - rx_ndtr);
                int16_t drift = offset - dma_initial_offset;
                if (drift > 512) drift -= 1024;
                if (drift < -512) drift += 1024;

                printf("[DEBUG] SAI_A: SR=0x%lX CR1=0x%08lX\r\n",
                       SAI2_Block_A->SR, SAI2_Block_A->CR1);
                printf("[DEBUG] SAI_B: SR=0x%lX CR1=0x%08lX\r\n",
                       SAI2_Block_B->SR, SAI2_Block_B->CR1);
                printf("[DEBUG] DMA RX NDTR=%lu, TX NDTR=%lu, offset=%d, drift=%d\r\n",
                       rx_ndtr, tx_ndtr, offset, drift);
                extern volatile uint32_t audio_process_count;
                printf("[DEBUG] RX[0..3]: %d %d %d %d\r\n",
                       audio_rx_buffer[0], audio_rx_buffer[1],
                       audio_rx_buffer[2], audio_rx_buffer[3]);
                printf("[DEBUG] TX[0..3]: %d %d %d %d\r\n",
                       audio_tx_buffer[0], audio_tx_buffer[1],
                       audio_tx_buffer[2], audio_tx_buffer[3]);
                printf("[DEBUG] Process=%lu\r\n", audio_process_count);
            }
            last_buffer_count = audio_buffer_count;
            last_status_time = now;
        }

        /* Process ESP32 UART data - accumulate into line buffer */
        while (rx_head != rx_tail)
        {
            uint8_t data = rx_buffer[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

            if (data == '\n')
            {
                /* End of line - process command */
                line_buffer[line_pos] = '\0';
                if (line_pos > 0 && line_buffer[line_pos - 1] == '\r')
                {
                    line_buffer[line_pos - 1] = '\0';  /* Remove \r */
                }
                if (line_pos > 0)
                {
                    Process_GATT_Command(line_buffer);
                }
                line_pos = 0;
            }
            else if (line_pos < LINE_BUFFER_SIZE - 1)
            {
                line_buffer[line_pos++] = (char)data;
            }
        }

        /* Update IN-13 VU meter DAC output */
        VU_WriteDac();

        /* Small delay to prevent tight loop when no audio */
        HAL_Delay(1);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 120;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART2 Initialization Function (ESP32 Communication)
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART3 Initialization Function (Debug Console / VCP)
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief DMA Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* Audio DMA at lower priority (2) so UART can interrupt */
    /* DMA1_Stream0_IRQn - SAI2_A RX */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    /* DMA1_Stream1_IRQn - SAI2_B TX */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    /* UART2 interrupt at highest priority (0) for reliable command reception */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief SAI2 Initialization Function (I2S Audio Input/Output)
 *
 * SAI2_A: Slave Receiver - receives I2S from BM83
 *   - Pins: PD13 (SCK), PD12 (FS), PD11 (SD)
 *   - Mode: Slave, receives clock from BM83 (external master)
 *
 * SAI2_B: Slave Transmitter - outputs I2S to DAC
 *   - Pin: PA0 (SD)
 *   - Mode: Synchronous slave to SAI2_A (shares clock)
 *
 * @param None
 * @retval None
 */
static void MX_SAI2_Init(void)
{
    /* -------- SAI2 Block A - Slave Receiver (I2S Input) -------- */
    /* Using FREE protocol for full control over I2S timing */
    hsai_rx.Instance = SAI2_Block_A;
    hsai_rx.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_rx.Init.AudioMode = SAI_MODESLAVE_RX;
    hsai_rx.Init.DataSize = SAI_DATASIZE_16;
    hsai_rx.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_rx.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
    hsai_rx.Init.Synchro = SAI_ASYNCHRONOUS;
    hsai_rx.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    hsai_rx.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
    hsai_rx.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
    hsai_rx.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
    hsai_rx.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_rx.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_rx.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_rx.Init.TriState = SAI_OUTPUT_NOTRELEASED;

    /* Frame: 32 bits total (16 left + 16 right), I2S Philips style */
    hsai_rx.FrameInit.FrameLength = 32;
    hsai_rx.FrameInit.ActiveFrameLength = 16;
    hsai_rx.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
    hsai_rx.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
    hsai_rx.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

    /* Slots: 2 x 16-bit */
    hsai_rx.SlotInit.FirstBitOffset = 0;
    hsai_rx.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
    hsai_rx.SlotInit.SlotNumber = 2;
    hsai_rx.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

    if (HAL_SAI_Init(&hsai_rx) != HAL_OK)
    {
        Error_Handler();
    }

    /* Disable ALL error interrupts - in slave mode we trust external sync */
    __HAL_SAI_DISABLE_IT(&hsai_rx, SAI_IT_OVRUDR | SAI_IT_AFSDET | SAI_IT_LFSDET | SAI_IT_WCKCFG);
    __HAL_SAI_DISABLE_IT(&hsai_tx, SAI_IT_OVRUDR | SAI_IT_AFSDET | SAI_IT_LFSDET | SAI_IT_WCKCFG);

    /* -------- SAI2 Block B - Slave Transmitter (I2S Output) -------- */
    /* Synchronized to Block A */
    hsai_tx.Instance = SAI2_Block_B;
    hsai_tx.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai_tx.Init.AudioMode = SAI_MODESLAVE_TX;
    hsai_tx.Init.DataSize = SAI_DATASIZE_16;
    hsai_tx.Init.FirstBit = SAI_FIRSTBIT_MSB;
    hsai_tx.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
    hsai_tx.Init.Synchro = SAI_SYNCHRONOUS;
    hsai_tx.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
    hsai_tx.Init.NoDivider = SAI_MASTERDIVIDER_DISABLE;
    hsai_tx.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
    hsai_tx.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
    hsai_tx.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_tx.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_tx.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_tx.Init.TriState = SAI_OUTPUT_NOTRELEASED;

    /* Frame: same as RX (Philips) */
    hsai_tx.FrameInit.FrameLength = 32;
    hsai_tx.FrameInit.ActiveFrameLength = 16;
    hsai_tx.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
    hsai_tx.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
    hsai_tx.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

    /* Slots: same as RX */
    hsai_tx.SlotInit.FirstBitOffset = 0;
    hsai_tx.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
    hsai_tx.SlotInit.SlotNumber = 2;
    hsai_tx.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

    if (HAL_SAI_Init(&hsai_tx) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    /* GPIO clocks are enabled in HAL_xxx_MspInit() callbacks */
}

/**
 * @brief  Audio processing function - called for each buffer
 *         Currently just passes through (echo) for testing
 * @param  rx_buf: Pointer to received audio samples (16-bit)
 * @param  tx_buf: Pointer to transmit audio samples (16-bit)
 * @param  samples: Number of samples (L+R interleaved)
 * @retval None
 */
volatile uint32_t audio_process_count = 0;

/* Simple sine wave lookup table (256 entries, one full cycle) */
static const int16_t sine_table[256] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179,
    7962, 8739, 9512, 10278, 11039, 11793, 12539, 13279, 14010, 14732,
    15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403,
    22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571,
    30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521,
    32609, 32678, 32728, 32757, 32767, 32757, 32728, 32678, 32609, 32521,
    32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790,
    26319, 25832, 25329, 24811, 24279, 23731, 23170, 22594, 22005, 21403,
    20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732,
    14010, 13279, 12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179,
    6393, 5602, 4808, 4011, 3212, 2410, 1608, 804, 0, -804,
    -1608, -2410, -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739,
    -9512, -10278, -11039, -11793, -12539, -13279, -14010, -14732, -15446, -16151,
    -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683,
    -28105, -28510, -28898, -29268, -29621, -29956, -30273, -30571, -30852, -31113,
    -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609, -32678,
    -32728, -32757, -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571, -30273, -29956,
    -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832,
    -25329, -24811, -24279, -23731, -23170, -22594, -22005, -21403, -20787, -20159,
    -19519, -18868, -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179, -6393, -5602,
    -4808, -4011, -3212, -2410, -1608, -804
};
static uint32_t sine_phase = 0;
static volatile uint8_t dsp_sine_test = 0;  /* Sine test mode: generates 1kHz tone internally */

/**
 * @brief  Parse hex string to bytes
 * @param  hex: Input hex string (e.g., "0164")
 * @param  out: Output byte array
 * @param  max_len: Maximum bytes to parse
 * @retval Number of bytes parsed
 */
static int hex_to_bytes(const char *hex, uint8_t *out, int max_len)
{
    int len = 0;
    while (*hex && *(hex + 1) && len < max_len)
    {
        char byte_str[3] = {hex[0], hex[1], '\0'};
        out[len++] = (uint8_t)strtol(byte_str, NULL, 16);
        hex += 2;
    }
    return len;
}

/**
 * @brief  Process GATT command received from ESP32
 *         Format: GATT:<characteristic>:<hex_data>
 *         Example: GATT:CTRL:0102 (Set preset to NIGHT)
 * @param  line: Complete line without \r\n
 */
static void Process_GATT_Command(const char *line)
{
    /* Check if it's a GATT command */
    if (strncmp(line, "GATT:", 5) != 0)
    {
        printf("[ESP32] %s\r\n", line);
        return;
    }

    /* Parse: GATT:<char>:<hex> */
    const char *char_start = line + 5;
    const char *colon = strchr(char_start, ':');
    if (!colon)
    {
        printf("[ESP32] Invalid GATT format: %s\r\n", line);
        return;
    }

    /* Extract characteristic name */
    int char_len = colon - char_start;
    char char_name[32];
    if (char_len >= (int)sizeof(char_name)) char_len = sizeof(char_name) - 1;
    strncpy(char_name, char_start, char_len);
    char_name[char_len] = '\0';

    /* Extract hex data */
    const char *hex_data = colon + 1;
    uint8_t cmd_bytes[16];
    int cmd_len = hex_to_bytes(hex_data, cmd_bytes, sizeof(cmd_bytes));

    /* Log the command */
    printf("[GATT] %s:", char_name);
    for (int i = 0; i < cmd_len; i++)
    {
        printf(" %02X", cmd_bytes[i]);
    }
    printf("\r\n");

    /* Handle specific commands - accept any characteristic that ends with "CTRL" or "TRL"
     * to handle occasional UART byte loss */
    if ((strstr(char_name, "CTRL") != NULL || strstr(char_name, "TRL") != NULL) && cmd_len >= 2)
    {
        uint8_t cmd_type = cmd_bytes[0];
        uint8_t cmd_value = cmd_bytes[1];

        switch (cmd_type)
        {
            case 0x01:  /* Set Preset */
                {
                    const char *preset_names[] = {"OFFICE", "FULL", "NIGHT", "SPEECH"};
                    if (cmd_value <= PRESET_SPEECH) {
                        current_preset = cmd_value;
                        reset_preset_filters();
                        update_target_gain();  /* Recalculate gain (NIGHT mode has volume cap) */
                        printf("[DSP] Preset: %s\r\n", preset_names[cmd_value]);
                    } else {
                        printf("[DSP] Invalid preset: %d\r\n", cmd_value);
                    }
                }
                break;
            case 0x02:  /* Set Loudness */
                printf("[DSP] Loudness: %s\r\n", cmd_value ? "ON" : "OFF");
                dsp_loudness_enabled = cmd_value ? 1 : 0;
                if (!cmd_value) {
                    /* Reset filter state when disabling to avoid clicks */
                    memset(&loudness_state_L, 0, sizeof(loudness_state_L));
                    memset(&loudness_state_R, 0, sizeof(loudness_state_R));
                }
                break;
            case 0x04:  /* Set Mute */
                printf("[DSP] Mute: %s\r\n", cmd_value ? "ON" : "OFF");
                dsp_mute_enabled = cmd_value ? 1 : 0;
                break;
            case 0x05:  /* Set Audio Duck */
                printf("[DSP] Duck: %s\r\n", cmd_value ? "ON (-12dB)" : "OFF");
                dsp_duck_enabled = cmd_value ? 1 : 0;
                duck_target_gain = cmd_value ? DUCK_GAIN_LIN : 1.0f;
                break;
            case 0x06:  /* Set Normalizer/DRC */
                printf("[DSP] Normalizer: %s\r\n", cmd_value ? "ON" : "OFF");
                dsp_normalizer_enabled = cmd_value ? 1 : 0;
                if (!cmd_value) {
                    /* Reset DRC envelope when disabling */
                    drc_envelope = 0.0f;
                }
                break;
            case 0x07:  /* Set Volume / Device Trim */
                {
                    uint8_t trim = cmd_value > 100 ? 100 : cmd_value;
                    device_trim_value = trim;
                    update_target_gain();  /* NIGHT mode cap applied here */
                    printf("[DSP] Trim: %d%% (target gain: %.3f)\r\n",
                           trim, target_gain);
                }
                break;
            case 0x08:  /* Set Bypass */
                printf("[DSP] Bypass: %s\r\n", cmd_value ? "ON" : "OFF");
                dsp_bypass_enabled = cmd_value ? 1 : 0;
                break;
            case 0x09:  /* Set Bass Boost */
                printf("[DSP] Bass Boost: %s\r\n", cmd_value ? "ON (+8dB)" : "OFF");
                dsp_bass_boost_enabled = cmd_value ? 1 : 0;
                if (!cmd_value) {
                    /* Reset filter state when disabling to avoid clicks */
                    memset(&bass_state_L, 0, sizeof(bass_state_L));
                    memset(&bass_state_R, 0, sizeof(bass_state_R));
                }
                break;
            case 0x0A:  /* Sine Test Mode */
                printf("[DSP] Sine Test: %s (1kHz tone, ignores input)\r\n", cmd_value ? "ON" : "OFF");
                dsp_sine_test = cmd_value ? 1 : 0;
                sine_phase = 0;
                break;
            default:
                printf("[DSP] Unknown command: 0x%02X\r\n", cmd_type);
                break;
        }
    }
}

static void Audio_Process(int16_t *rx_buf, int16_t *tx_buf, uint16_t samples)
{
    audio_process_count++;

    /* Update VU meter RMS from input signal (pre-DSP for true level) */
    VU_UpdateRMS(rx_buf, samples);

    /* Sine test mode: generate 1kHz tone internally, ignore input */
    if (dsp_sine_test)
    {
        /* 1kHz at 44100Hz: step through 256-entry table at 256*1000/44100 ≈ 5.8 per sample
         * Using fixed-point: phase_increment = (256 * 1000 * 256) / 44100 ≈ 1486 (8.8 fixed) */
        for (uint16_t i = 0; i < samples; i += 2)
        {
            int16_t val = sine_table[(sine_phase >> 8) & 0xFF] >> 2;  /* -12dB */
            tx_buf[i]     = val;
            tx_buf[i + 1] = val;
            sine_phase += 1486;  /* ~1kHz */
        }
        return;
    }

    /* Bypass mode: direct passthrough without any processing */
    if (dsp_bypass_enabled)
    {
        memcpy(tx_buf, rx_buf, samples * sizeof(int16_t));
        return;
    }

    /* Process stereo samples (L, R, L, R, ...) */
    for (uint16_t i = 0; i < samples; i += 2)
    {
        /* Smooth gain ramping - approach target gain gradually */
        current_gain += (target_gain - current_gain) * GAIN_RAMP_COEFF;

        /* Smooth duck gain ramping */
        duck_current_gain += (duck_target_gain - duck_current_gain) * GAIN_RAMP_COEFF;

        /* Get input samples as float (-1.0 to 1.0 range) */
        float left_in  = (float)rx_buf[i]     / 32768.0f;
        float right_in = (float)rx_buf[i + 1] / 32768.0f;

        float left_out  = left_in;
        float right_out = right_in;

        /* ==========================================================
         * DSP Chain (order matters!)
         * 1. Preset EQ (OFFICE/FULL/NIGHT/SPEECH)
         * 2. Loudness overlay (+6dB @ 150Hz)
         * 3. Bass Boost (+8dB @ 100Hz)
         * 4. Normalizer/DRC
         * 5. Volume/Trim
         * 6. Duck
         * 7. Mute
         * 8. Limiter (soft clip)
         * ========================================================== */

        /* 1. Preset EQ */
        preset_process(&left_out, &right_out);

        /* 2. Loudness overlay: Low-shelf bass boost +6dB @ 150Hz */
        if (dsp_loudness_enabled)
        {
            left_out  = loudness_process(&loudness_state_L, left_out);
            right_out = loudness_process(&loudness_state_R, right_out);
        }

        /* 3. Bass Boost: Low-shelf +8dB @ 100Hz */
        if (dsp_bass_boost_enabled)
        {
            left_out  = bass_process(&bass_state_L, left_out);
            right_out = bass_process(&bass_state_R, right_out);
        }

        /* 3. Normalizer/DRC: Dynamic range compression */
        if (dsp_normalizer_enabled)
        {
            /* Use max of L/R for envelope detection (linked stereo) */
            float peak = (left_out > right_out) ? left_out : right_out;
            if (-left_out > peak) peak = -left_out;
            if (-right_out > peak) peak = -right_out;

            float drc_gain = drc_process(peak);
            left_out  *= drc_gain;
            right_out *= drc_gain;
        }

        /* 4. Apply volume/trim with smooth ramping */
        left_out  *= current_gain;
        right_out *= current_gain;

        /* 5. Duck: Temporary volume reduction (-12dB) */
        left_out  *= duck_current_gain;
        right_out *= duck_current_gain;

        /* 6. Mute: Zero output */
        if (dsp_mute_enabled)
        {
            left_out  = 0.0f;
            right_out = 0.0f;
        }

        /* 7. Limiter: Soft clipping to prevent harsh distortion */
        if (left_out > 1.0f) left_out = 1.0f;
        if (left_out < -1.0f) left_out = -1.0f;
        if (right_out > 1.0f) right_out = 1.0f;
        if (right_out < -1.0f) right_out = -1.0f;

        /* Convert back to 16-bit integer */
        tx_buf[i]     = (int16_t)(left_out  * 32767.0f);
        tx_buf[i + 1] = (int16_t)(right_out * 32767.0f);
    }
}

/**
 * @brief  SAI Rx Half Complete callback - copy to TX for passthrough
 * @param  hsai: SAI handle
 * @retval None
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_A)
    {
        Audio_Process(audio_rx_buffer, audio_tx_buffer, AUDIO_BUFFER_SIZE);
        audio_buffer_count++;
    }
}

/**
 * @brief  SAI Rx Complete callback - copy to TX for passthrough
 * @param  hsai: SAI handle
 * @retval None
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_A)
    {
        Audio_Process(&audio_rx_buffer[AUDIO_BUFFER_SIZE],
                     &audio_tx_buffer[AUDIO_BUFFER_SIZE], AUDIO_BUFFER_SIZE);
        audio_buffer_count++;
    }
}

/**
 * @brief  SAI Tx Half Complete callback - RX callback handles passthrough
 * @param  hsai: SAI handle
 * @retval None
 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    (void)hsai;
}

/**
 * @brief  SAI Tx Complete callback - RX callback handles passthrough
 * @param  hsai: SAI handle
 * @retval None
 */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    (void)hsai;
}

/**
 * @brief  SAI Error callback
 * @param  hsai: SAI handle
 * @retval None
 */
volatile uint32_t sai_error_count_a = 0;
volatile uint32_t sai_error_count_b = 0;
volatile uint32_t sai_last_error_a = 0;
volatile uint32_t sai_last_error_b = 0;

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_A)
    {
        sai_error_count_a++;
        sai_last_error_a = hsai->ErrorCode;
        /* Just clear flags, don't restart DMA from interrupt */
        __HAL_SAI_CLEAR_FLAG(hsai, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET);
    }
    else if (hsai->Instance == SAI2_Block_B)
    {
        sai_error_count_b++;
        sai_last_error_b = hsai->ErrorCode;
        __HAL_SAI_CLEAR_FLAG(hsai, SAI_FLAG_OVRUDR | SAI_FLAG_AFSDET | SAI_FLAG_LFSDET);
    }
}

/**
 * @brief  Rx Transfer completed callback
 * @param  huart UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Store byte in ring buffer */
        rx_buffer[rx_head] = rx_byte;
        rx_head = (rx_head + 1) % RX_BUFFER_SIZE;

        /* Continue receiving */
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/**
 * @brief  UART Error callback - required for continuous reception
 * @param  huart UART handle
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Abort resets the HAL state machine properly */
        HAL_UART_Abort(huart);

        /* Clear all error flags */
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                              UART_CLEAR_NEF | UART_CLEAR_OREF);

        /* Re-arm reception */
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
