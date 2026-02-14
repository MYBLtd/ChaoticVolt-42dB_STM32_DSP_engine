# Technical Documentation

Detailed implementation notes for the 42dB DSP Engine on STM32H753.

## System Architecture

```
                    ┌───────────────────────────────────────────────────────────────────┐
                    │                      STM32H753ZI DSP Engine                       │
                    │                                                                   │
                    │  ┌─────────────┐     ┌─────────────────────────────┐             │
I2S from ESP32 ────>│  │   SAI2_A    │────>│      Audio_Process()        │             │
(via 74HCT04)       │  │  Slave RX   │     │                             │             │
                    │  │  DMA1_St0   │     │  Preset EQ ──> Loudness     │             │
                    │  └─────────────┘     │       │            │        │             │
                    │                      │       v            v        │             │
                    │                      │  Bass Boost ──> Normalizer  │             │
                    │                      │       │            │        │             │
                    │                      │       v            v        │             │
                    │                      │   Volume ──> Duck ──> Mute  │             │
                    │  ┌─────────────┐     │       │                     │             │
I2S to PCM5102A <───│  │   SAI2_B    │<────│   Limiter                   │             │
                    │  │  Slave TX   │     │                             │             │
                    │  │  DMA1_St1   │     └─────────────────────────────┘             │
                    │  └─────────────┘                   ▲                             │
                    │                                    │                             │
                    │  ┌─────────────┐     ┌─────────────┴──────────┐                  │
UART from ESP32 ───>│  │   USART2    │────>│  Process_GATT_Command() │                  │
(GATT commands)     │  │ IRQ Prio 0  │     │   - Parse command       │                  │
                    │  └─────────────┘     │   - Update DSP flags    │                  │
                    │                      └────────────────────────┘                  │
                    │                                                                   │
                    │  ┌─────────────┐                                                  │
Debug output <──────│  │   USART3    │   ← printf() via _write() override               │
(ST-LINK VCP)       │  └─────────────┘                                                  │
                    │                                                                   │
                    │  ┌─────────────┐     ┌─────────────────────────┐                  │
IN-13 VU meter <────│  │    DAC1     │<────│  VU_UpdateRMS() (ISR)   │                  │
(via MPSA42)        │  │  PA4 output │     │  VU_WriteDac() (main)   │                  │
                    │  └─────────────┘     └─────────────────────────┘                  │
                    │                                                                   │
                    └───────────────────────────────────────────────────────────────────┘
```

## Peripheral Configuration

### USART2 (ESP32 BLE Control)

| Setting | Value | Notes |
|---------|-------|-------|
| Instance | USART2 | APB1 peripheral |
| TX Pin | PD5 (AF7) | CN9 pin 6, to ESP32 GPIO5 |
| RX Pin | PD6 (AF7) | CN9 pin 4, from ESP32 GPIO4 |
| Baud Rate | 115200 | 8N1 |
| Mode | Interrupt-driven | Ring buffer in ISR |
| NVIC Priority | 0 | Highest - critical for reliability |

### USART3 (Debug Console)

| Setting | Value | Notes |
|---------|-------|-------|
| Instance | USART3 | APB1 peripheral |
| TX Pin | PD8 (AF7) | ST-LINK VCP |
| RX Pin | PD9 (AF7) | ST-LINK VCP |
| Baud Rate | 115200 | 8N1 |

### SAI2 (I2S Audio)

**SAI2_A - I2S Input (Slave Receiver from ESP32)**

| Setting | Value | Notes |
|---------|-------|-------|
| Instance | SAI2_Block_A | APB2 peripheral |
| SCK Pin | PD13 (AF10) | Bit Clock from ESP32 (via 74HCT04) |
| FS Pin | PD12 (AF10) | Frame Sync / LRCK from ESP32 (via 74HCT04) |
| SD Pin | PD11 (AF10) | Serial Data from ESP32 (via 74HCT04) |
| Mode | Slave RX | Clock provided by ESP32 |
| Protocol | FREE (manual config) | Full control over I2S timing |
| Data Size | 16-bit | Matches ESP32 output |
| Frame Length | 32 bits | 16L + 16R |
| ClockStrobing | Rising Edge | Sample on rising BCLK edge |
| FSOffset | Before First Bit | I2S Philips standard |
| FSPolarity | Active Low | Left channel when FS low |
| Sample Rate | 44100 Hz | From ESP32 A2DP |
| DMA | DMA1_Stream0 | Circular mode, half/full callbacks |

**SAI2_B - I2S Output (Slave Transmitter to DAC)**

| Setting | Value | Notes |
|---------|-------|-------|
| Instance | SAI2_Block_B | APB2 peripheral |
| SD Pin | PA0 (AF10) | Serial Data to PCM5102A DIN |
| Mode | Slave TX | Synchronized to SAI2_A |
| ClockStrobing | Falling Edge | Output on falling BCLK edge |
| Synchro | SAI_SYNCHRONOUS | Internal sync to Block A |
| DMA | DMA1_Stream1 | Circular mode |

**Clock Edge Explanation:**

The ESP32 outputs data on falling BCLK edge (valid during low period). To capture this correctly:
- SAI2_A samples on RISING edge (data stable from previous falling edge)
- SAI2_B outputs on FALLING edge (same timing as original)

```
         ┌──┐  ┌──┐  ┌──┐  ┌──┐
BCLK  ───┘  └──┘  └──┘  └──┘  └──
            ↓     ↓     ↓         ESP32 outputs data on falling edge
            ↑     ↑     ↑         STM32 samples data on rising edge
```

## DMA Configuration

### Memory Placement (Critical!)

On STM32H7, DMA1 and DMA2 can ONLY access D2 SRAM (0x30000000-0x30047FFF). Buffers in other RAM regions will cause DMA transfer failures.

```c
/* Linker script section for D2 SRAM */
.RAM_D2 (NOLOAD) :
{
    . = ALIGN(4);
    *(.RAM_D2)
    *(.RAM_D2*)
    . = ALIGN(4);
} >RAM_D2

/* Buffer placement in code */
__attribute__((section(".RAM_D2"))) int16_t audio_rx_buffer[AUDIO_BUFFER_SIZE * 2];
__attribute__((section(".RAM_D2"))) int16_t audio_tx_buffer[AUDIO_BUFFER_SIZE * 2];
```

### Buffer Strategy

Double-buffering with circular DMA:

```
DMA position:  ├──── First Half ────┼──── Second Half ────┤
               0                   512                   1024

HalfCpltCallback: Process samples 0-511 → TX buffer 0-511
     CpltCallback: Process samples 512-1023 → TX buffer 512-1023
```

- Buffer size: 256 samples × 2 channels × 2 halves = 1024 samples
- Each sample: 16-bit (2 bytes)
- Total buffer: 2048 bytes per direction

### Latency Calculation

```
Samples per half-buffer: 256
Sample rate: 44100 Hz
Latency per half: 256 / 44100 = 5.8 ms
Total round-trip latency: ~12 ms (RX half + processing + TX half)
```

## Interrupt Priority Configuration

This was a critical fix for reliable UART reception. When audio DMA had the same or higher priority than UART, bytes were occasionally lost during heavy audio processing.

```c
/* In MX_DMA_Init() */
HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);      /* Highest priority */
HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0); /* Audio RX - lower */
HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0); /* Audio TX - lower */
```

Priority 0 = highest (can preempt everything)
Priority 2 = lower (can be preempted by priority 0-1)

## UART Reception

### Ring Buffer Implementation

```c
#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_head = 0;  /* Write position (ISR) */
volatile uint16_t rx_tail = 0;  /* Read position (main loop) */
uint8_t rx_byte;                /* Single byte for HAL IT receive */
```

### ISR Flow

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    /* Store byte in ring buffer */
    rx_buffer[rx_head] = rx_byte;
    rx_head = (rx_head + 1) % RX_BUFFER_SIZE;

    /* Re-arm for next byte */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}
```

### Error Recovery

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Abort(huart);  /* Reset HAL state machine */
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                          UART_CLEAR_NEF | UART_CLEAR_OREF);
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);  /* Re-arm */
}
```

### Line Buffer Processing

Commands accumulate in `line_buffer[]` until `\n` is received:

```c
while (rx_head != rx_tail) {
    uint8_t data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

    if (data == '\n') {
        line_buffer[line_pos] = '\0';
        Process_GATT_Command(line_buffer);
        line_pos = 0;
    } else {
        line_buffer[line_pos++] = data;
    }
}
```

## GATT Command Protocol

### Format

```
GATT:CTRL:<hex_bytes>\r\n
```

Example: `GATT:CTRL:0201\r\n` = Enable Loudness

### Tolerant Parsing

Due to occasional UART byte loss, the parser accepts both "CTRL" and "TRL":

```c
if ((strstr(char_name, "CTRL") != NULL || strstr(char_name, "TRL") != NULL) && cmd_len >= 2)
```

### Command Handlers

| CMD | Handler | Action |
|-----|---------|--------|
| 0x01 | SET_PRESET | Set `current_preset`, call `reset_preset_filters()` + `update_target_gain()` |
| 0x02 | SET_LOUDNESS | Set `dsp_loudness_enabled`, clear filter state on disable |
| 0x04 | SET_MUTE | Set `dsp_mute_enabled` |
| 0x05 | SET_DUCK | Set `dsp_duck_enabled`, update `duck_target_gain` |
| 0x06 | SET_NORMALIZER | Set `dsp_normalizer_enabled`, reset envelope on disable |
| 0x07 | SET_VOLUME | Set `device_trim_value`, call `update_target_gain()` |
| 0x08 | SET_BYPASS | Set `dsp_bypass_enabled` |
| 0x09 | SET_BASS_BOOST | Set `dsp_bass_boost_enabled`, clear filter state on disable |

## DSP Implementation

### Biquad Filter Structure

```c
typedef struct {
    float x1, x2;  /* Input history: x[n-1], x[n-2] */
    float y1, y2;  /* Output history: y[n-1], y[n-2] */
} BiquadState;
```

### Biquad Processing (Direct Form I)

```c
float output = b0 * input
             + b1 * state->x1
             + b2 * state->x2
             - a1 * state->y1
             - a2 * state->y2;

state->x2 = state->x1;
state->x1 = input;
state->y2 = state->y1;
state->y1 = output;
```

### Coefficient Calculation

Using Robert Bristow-Johnson's Audio EQ Cookbook formulas.

**Low-Shelf Filter:**
```
A  = sqrt(10^(dB_gain/20))
w0 = 2*pi*Fc/Fs
alpha = sin(w0)/(2*Q)

b0 =    A*((A+1) - (A-1)*cos(w0) + 2*sqrt(A)*alpha)
b1 =  2*A*((A-1) - (A+1)*cos(w0))
b2 =    A*((A+1) - (A-1)*cos(w0) - 2*sqrt(A)*alpha)
a0 =        (A+1) + (A-1)*cos(w0) + 2*sqrt(A)*alpha
a1 =   -2*((A-1) + (A+1)*cos(w0))
a2 =        (A+1) + (A-1)*cos(w0) - 2*sqrt(A)*alpha

/* Normalize by a0 */
b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;
```

### Implemented Filters

| Name | Type | Frequency | Gain | Q/Slope |
|------|------|-----------|------|---------|
| Loudness | Low-shelf | 150 Hz | +6 dB | Q=0.707 |
| Bass Boost | Low-shelf | 100 Hz | +8 dB | S=0.7 |
| OFFICE preset | High-shelf | 6 kHz | +1.5 dB | - |
| FULL bass | Low-shelf | 120 Hz | +4 dB | - |
| FULL treble | High-shelf | 8 kHz | +3 dB | - |
| NIGHT | Low-shelf | 150 Hz | -3 dB | - |
| SPEECH HP | High-pass | 150 Hz | - | Q=0.707 |
| SPEECH mid | Peaking | 2.5 kHz | +3 dB | Q=1.0 |

### Normalizer/DRC Implementation

```c
#define DRC_THRESHOLD_LIN   0.251189f  /* -12 dB */
#define DRC_RATIO           2.0f       /* Gentle compression */
#define DRC_ATTACK_COEF     0.002268f  /* ~10 ms */
#define DRC_RELEASE_COEF    0.000284f  /* ~80 ms */
#define DRC_MAKEUP_LIN      1.412538f  /* +3 dB */

static float drc_envelope = 0.0f;
static float drc_gain_smooth = 1.0f;

float drc_process(float input_level) {
    float abs_level = fabsf(input_level);

    /* Envelope follower with attack/release */
    if (abs_level > drc_envelope) {
        drc_envelope += DRC_ATTACK_COEF * (abs_level - drc_envelope);
    } else {
        drc_envelope += DRC_RELEASE_COEF * (abs_level - drc_envelope);
    }

    /* Calculate target gain */
    float target_drc_gain;
    if (drc_envelope > DRC_THRESHOLD_LIN) {
        float excess_ratio = drc_envelope / DRC_THRESHOLD_LIN;
        float compressed = DRC_THRESHOLD_LIN * powf(excess_ratio, 1.0f / DRC_RATIO);
        target_drc_gain = (compressed / drc_envelope) * DRC_MAKEUP_LIN;
    } else {
        target_drc_gain = DRC_MAKEUP_LIN;
    }

    /* Smooth gain changes */
    drc_gain_smooth += 0.01f * (target_drc_gain - drc_gain_smooth);

    return drc_gain_smooth;
}
```

### Volume/Trim Implementation

**Perceptual dB Mapping:**

Human perception is logarithmic. Linear fade sounds wrong. This piecewise approximation gives natural volume control:

```c
float trim_to_gain(uint8_t trim) {
    float db;
    if (trim >= 100)      db = 0.0f;
    else if (trim >= 80)  db = (trim - 100) * 0.3f;       /* 100→0dB, 80→-6dB */
    else if (trim >= 60)  db = -6.0f + (trim - 80) * 0.3f;  /* 80→-6dB, 60→-12dB */
    else if (trim >= 40)  db = -12.0f + (trim - 60) * 0.4f; /* 60→-12dB, 40→-20dB */
    else if (trim >= 20)  db = -20.0f + (trim - 40) * 0.75f; /* 40→-20dB, 20→-35dB */
    else                  db = -35.0f + (trim - 20) * 1.25f; /* 20→-35dB, 0→-60dB */

    db += FIXED_HEADROOM_DB;  /* -9 dB always applied */
    return powf(10.0f, db / 20.0f);
}
```

**Fixed Headroom Approach:**

Early implementation dynamically adjusted headroom based on active effects. This caused perceptible volume drops when enabling loudness/bass boost.

Fix: Always apply -9dB headroom. Effects "fill" this headroom rather than requiring additional reduction.

```c
#define FIXED_HEADROOM_DB   -9.0f
#define FIXED_HEADROOM_LIN  0.3548f  /* 10^(-9/20) */
```

**NIGHT Mode Volume Cap:**

Applied in `update_target_gain()`, not by modifying `device_trim_value`:

```c
void update_target_gain(void) {
    uint8_t effective_trim = device_trim_value;

    if (current_preset == PRESET_NIGHT && effective_trim > PRESET_NIGHT_VOLUME_CAP) {
        effective_trim = PRESET_NIGHT_VOLUME_CAP;  /* 60% */
    }

    target_gain = trim_to_gain(effective_trim);
}
```

This ensures switching away from NIGHT restores full volume.

### Smooth Gain Ramping

Prevents clicks/pops when gain changes:

```c
#define GAIN_RAMP_COEFF 0.05f

/* In Audio_Process() */
current_gain += (target_gain - current_gain) * GAIN_RAMP_COEFF;
duck_current_gain += (duck_target_gain - duck_current_gain) * GAIN_RAMP_COEFF;
```

At 44.1kHz with 256-sample buffers (~172 buffers/sec), this gives ~10ms ramp time constant.

### Audio Processing Chain

```c
void Audio_Process(int16_t *rx_buf, int16_t *tx_buf, uint16_t samples) {
    if (dsp_bypass_enabled) {
        memcpy(tx_buf, rx_buf, samples * sizeof(int16_t));
        return;
    }

    for (uint16_t i = 0; i < samples; i += 2) {
        /* Smooth gain ramping */
        current_gain += (target_gain - current_gain) * GAIN_RAMP_COEFF;
        duck_current_gain += (duck_target_gain - duck_current_gain) * GAIN_RAMP_COEFF;

        /* Convert to float [-1.0, 1.0] */
        float left  = rx_buf[i]     / 32768.0f;
        float right = rx_buf[i + 1] / 32768.0f;

        /* 1. Preset EQ */
        preset_process(&left, &right);

        /* 2. Loudness */
        if (dsp_loudness_enabled) {
            left  = loudness_process(&loudness_state_L, left);
            right = loudness_process(&loudness_state_R, right);
        }

        /* 3. Bass Boost */
        if (dsp_bass_boost_enabled) {
            left  = bass_process(&bass_state_L, left);
            right = bass_process(&bass_state_R, right);
        }

        /* 4. Normalizer/DRC */
        if (dsp_normalizer_enabled) {
            float peak = fmaxf(fabsf(left), fabsf(right));
            float drc_gain = drc_process(peak);
            left  *= drc_gain;
            right *= drc_gain;
        }

        /* 5. Volume/Trim with smooth ramping */
        left  *= current_gain;
        right *= current_gain;

        /* 6. Duck */
        left  *= duck_current_gain;
        right *= duck_current_gain;

        /* 7. Mute */
        if (dsp_mute_enabled) {
            left  = 0.0f;
            right = 0.0f;
        }

        /* 8. Limiter (hard clip) */
        left  = fmaxf(-1.0f, fminf(1.0f, left));
        right = fmaxf(-1.0f, fminf(1.0f, right));

        /* Convert back to int16 */
        tx_buf[i]     = (int16_t)(left  * 32767.0f);
        tx_buf[i + 1] = (int16_t)(right * 32767.0f);
    }
}
```

## IN-13 Nixie Bargraph VU Meter

### DAC Configuration

| Setting | Value | Notes |
|---------|-------|-------|
| Instance | DAC1 | APB1 peripheral |
| Output Pin | PA4 (DAC1_OUT1) | Analog mode, no pull |
| Trigger | None (software) | Written from main loop |
| Output Buffer | Enabled | Low impedance drive |
| Channel | 1 | 12-bit right-aligned |

### Drive Circuit

The IN-13 requires 0-10mA for 0-100% bar length. An MPSA42 NPN transistor (300V, hFE ~50-100) operates as a linear current sink controlled by the DAC.

```
+140V ─── 4K7 (1W) ─── IN-13 Anode
                         │
                   100KΩ ─── IN-13 Aux Cathode
                         │
                   IN-13 Cathode
                         │
                   MPSA42 Collector
                   MPSA42 Base ─── 1KΩ ─── PA4
                         │              │
                       10KΩ (pull-down to GND)
                         │
                   MPSA42 Emitter ─── 270Ω ─── GND
```

**Measured calibration points:**
- 770mV at PA4 (DAC value 956) = 0% bar (tube barely visible)
- 1840mV at PA4 (DAC value 2283) = 100% bar (full 120mm)
- Below 770mV: tube off (unreliable glow region)

The 10KΩ base pull-down is required because the MPSA42 has sufficient gain that noise or finger-touch on the base causes false triggering.

### RMS Level Detection

```c
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
        vu_rms_smooth = rms * 0.3f + vu_rms_smooth * 0.7f;   /* ~2ms attack */
    else
        vu_rms_smooth = rms * 0.10f + vu_rms_smooth * 0.90f;  /* ~60ms release */
}
```

**Design decisions:**
- RMS computed from **input** signal (pre-DSP) to show true audio level
- Called from DMA ISR (Audio_Process) — only float math, no HAL calls
- DAC write happens in main loop (~1ms interval) to avoid ISR overhead
- Mono mix (L+R average) for single-channel VU display

### DAC Scaling

Music RMS is typically 3000-10000 (not 32768 peak). Direct normalization to 32768 would only use ~30% of the tube range.

```c
float normalized = vu_rms_smooth / 10000.0f;  /* Full scale at typical loud music */
if (normalized > 1.0f) normalized = 1.0f;

/* Sqrt compression: spreads mid-range for natural VU response */
normalized = sqrtf(normalized);

uint16_t dac_val = VU_DAC_MIN + (uint16_t)(normalized * (VU_DAC_MAX - VU_DAC_MIN));
```

### Startup Prime Burst

Gas discharge tubes need ionization priming to ensure the glow starts from the bottom (near the auxiliary cathode). Without priming, random glowing spots can appear in the middle of the tube.

```c
static void VU_Prime(void)
{
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, VU_DAC_MAX);
    HAL_Delay(50);  /* 50ms at full current */
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, VU_DAC_OFF);
}
```

## Clock Configuration

- HSE: 8 MHz (bypass mode, from ST-LINK)
- PLL: 480 MHz system clock
- AHBCLKDivider: /2 → 240 MHz
- APB1: 120 MHz (USART2, USART3)
- APB2: 120 MHz (SAI2)
- SAI2 Clock: External (slave mode, from ESP32)

## Memory Map

| Region | Address | Size | Usage |
|--------|---------|------|-------|
| FLASH | 0x08000000 | 2 MB | Code + constants |
| DTCMRAM | 0x20000000 | 128 KB | Stack, fast variables, .bss, .data |
| AXI SRAM (D1) | 0x24000000 | 512 KB | Heap, large buffers |
| D2 SRAM | 0x30000000 | 288 KB | **DMA buffers (required!)** |
| D3 SRAM | 0x38000000 | 64 KB | Available |
| ITCMRAM | 0x00000000 | 64 KB | Time-critical code |

## Troubleshooting

### UART Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| No data received | Wiring | Check ESP32 TX → STM32 PD6 |
| One byte then stops | Missing error callback | Implement HAL_UART_ErrorCallback |
| Occasional byte loss | Priority | Set UART priority 0, DMA priority 2 |
| "GATT:TRL" instead of "GATT:CTRL" | Normal | Parser tolerates this |

### I2S/SAI Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| No audio received | No clock | Check ESP32 playing audio, 74HCT04 power |
| Buffers/sec = 0 | No BCLK | Check wiring, scope BCLK signal |
| Distorted audio | Format mismatch | Verify 16-bit Philips format |
| Wrong channels | FSPolarity | Check Active Low setting |
| Glitchy audio | ClockStrobing | RX must use Rising Edge |
| No DAC output | SD pin | Verify PA0 → PCM5102A DIN |

### DSP Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| Effect produces silence | Bad coefficients | Verify biquad math, check a0 normalization |
| Volume drops on effect enable | Dynamic headroom | Use fixed -9dB headroom |
| DRC pumping | Parameters | Use gentle settings (2:1, -12dB threshold) |
| NIGHT volume stays low | Permanent trim change | Apply cap in update_target_gain() only |
| Clicks on effect toggle | Abrupt gain change | Reset filter states, use gain ramping |

## References

- [Audio EQ Cookbook (Bristow-Johnson)](https://www.w3.org/2011/audio/audio-eq-cookbook.html)
- [STM32H753 Reference Manual (RM0433)](https://www.st.com/resource/en/reference_manual/rm0433)
- [Nucleo-H753ZI User Manual (UM2407)](https://www.st.com/resource/en/user_manual/um2407)
- [STM32H7 HAL Driver Documentation (UM2217)](https://www.st.com/resource/en/user_manual/um2217)
- [STM32H7 SAI Application Note (AN5543)](https://www.st.com/resource/en/application_note/an5543)
- [STM32H7 DMA Application Note (AN4031)](https://www.st.com/resource/en/application_note/an4031)
