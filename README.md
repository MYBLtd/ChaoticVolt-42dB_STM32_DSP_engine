# 42dB DSP Engine for STM32H753

Real-time audio DSP processor running on STM32H753ZI (Nucleo-144), receiving I2S audio from ESP32 Bluetooth A2DP sink, with BLE GATT control interface.

**Version:** 0.3.0 | **Build:** 42 | **Date:** 2026-02-06

## Overview

This project implements a DSP audio engine that sits between a Bluetooth audio source (ESP32 running A2DP sink) and an I2S DAC. The STM32H753 receives audio over I2S, applies real-time DSP processing (EQ, loudness, compression, etc.), and outputs to a PCM5102A DAC.

Control commands are received via BLE GATT on the ESP32, which relays them over UART to the STM32.

```
┌──────────┐   Bluetooth   ┌─────────┐   I2S      ┌────────────┐   I2S     ┌──────────┐
│  Phone   │──────A2DP────>│  ESP32  │──────────>│  STM32H753 │─────────>│ PCM5102A │
│          │               │         │            │ DSP Engine │          │   DAC    │
│  42dB    │   BLE GATT    │         │   UART     │            │          │          │
│   App    │──────────────>│         │──────────>│            │          │          │
└──────────┘               └─────────┘            └────────────┘          └──────────┘
```

## Features

- **Real-time DSP Processing** at 44.1kHz, 16-bit stereo
- **Presets**: OFFICE, FULL, NIGHT (volume capped), SPEECH
- **Loudness**: Low-shelf +6dB @ 150Hz bass boost
- **Bass Boost**: Low-shelf +8dB @ 100Hz for small speakers
- **Normalizer/DRC**: Dynamic range compression (2:1, threshold -12dB)
- **Volume/Trim**: Device-side volume with perceptual dB curve
- **Duck Mode**: Panic button -12dB instant reduction
- **Mute**: Zero output
- **Bypass**: Skip all DSP (direct passthrough)
- **Fixed Headroom**: -9dB always applied, prevents volume jumps when toggling effects

## Hardware Requirements

### Microcontroller
- **STM32H753ZI Nucleo-144** (or any STM32H753 with sufficient pins)

### Audio Components
- **ESP32-WROOM-32** running Bluetooth A2DP sink firmware
- **PCM5102A** I2S DAC module (or equivalent)
- **74HCT04** hex inverter IC (for I2S signal buffering)

### Why 74HCT04 Buffer?

The STM32 GPIO inputs load the ESP32 I2S signals too much when connected directly. The 74HCT04 (double-inverter per signal = non-inverting buffer) provides:
- High input impedance (doesn't load ESP32)
- 5V output levels (STM32 is 5V tolerant)
- Clean signal edges

## Wiring

### I2S Audio Path

```
ESP32 I2S Output:
ESP32 BCLK (GPIO26) ──┬── PCM5102A BCK (direct)
                      └── 74HCT04 (2 stages) ── STM32 PD13 (SAI2_A SCK)

ESP32 LRCK (GPIO25) ──┬── PCM5102A LRCK (direct)
                      └── 74HCT04 (2 stages) ── STM32 PD12 (SAI2_A FS)

ESP32 DATA (GPIO22) ───── 74HCT04 (2 stages) ── STM32 PD11 (SAI2_A SD)

STM32 PA0 (SAI2_B SD) ─── PCM5102A DIN
```

**Important PCM5102A Notes:**
- SCK pin MUST be grounded (internal PLL mode)
- FMT pin: LOW = I2S Philips format
- XSMT pin: HIGH = unmuted

### UART Control Path (BLE GATT Relay)

```
ESP32 GPIO4 (TX) ──── STM32 PD6 (USART2 RX)
ESP32 GPIO5 (RX) ──── STM32 PD5 (USART2 TX)
GND ───────────────── GND
```

### Power

```
5V ──┬── ESP32 VIN
     ├── 74HCT04 VCC
     └── PCM5102A VIN

3.3V ── STM32 (from Nucleo regulator)
```

## Pin Summary

| Function | STM32 Pin | Nucleo Header | Notes |
|----------|-----------|---------------|-------|
| SAI2_A SCK (I2S BCLK in) | PD13 | CN9-10 | From 74HCT04 buffer |
| SAI2_A FS (I2S LRCK in) | PD12 | CN9-12 | From 74HCT04 buffer |
| SAI2_A SD (I2S Data in) | PD11 | CN9-14 | From 74HCT04 buffer |
| SAI2_B SD (I2S Data out) | PA0 | CN10-29 | To PCM5102A DIN |
| USART2 TX (to ESP32) | PD5 | CN9-6 | ESP32 UART RX |
| USART2 RX (from ESP32) | PD6 | CN9-4 | ESP32 UART TX |
| USART3 TX (Debug VCP) | PD8 | ST-LINK | Serial console |
| USART3 RX (Debug VCP) | PD9 | ST-LINK | Serial console |

## Building and Flashing

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- ST-Link utilities (`st-flash`)
- Make

### Build

```bash
make
```

Build number auto-increments in `Core/Inc/build_number.h`.

### Flash

```bash
./flash.sh
# or manually:
st-flash --reset write build/42dB_DSP_Engine.bin 0x8000000
```

### Debug Console

```bash
# Linux
screen /dev/ttyACM0 115200

# macOS
screen /dev/cu.usbmodem* 115200
```

## GATT Command Protocol

Commands are relayed from ESP32 over UART in format:
```
GATT:CTRL:<hex_bytes>\r\n
```

### Command Reference

| CMD | Name | Value | Description |
|-----|------|-------|-------------|
| `0x01` | SET_PRESET | `0x00-0x03` | OFFICE / FULL / NIGHT / SPEECH |
| `0x02` | SET_LOUDNESS | `0x00`/`0x01` | Loudness OFF/ON |
| `0x04` | SET_MUTE | `0x00`/`0x01` | Unmute/Mute |
| `0x05` | SET_DUCK | `0x00`/`0x01` | Duck OFF/ON (-12dB) |
| `0x06` | SET_NORMALIZER | `0x00`/`0x01` | DRC OFF/ON |
| `0x07` | SET_VOLUME | `0x00-0x64` | Volume 0-100% |
| `0x08` | SET_BYPASS | `0x00`/`0x01` | DSP Bypass OFF/ON |
| `0x09` | SET_BASS_BOOST | `0x00`/`0x01` | Bass Boost OFF/ON |

### Example Commands

```
GATT:CTRL:0100  → Preset OFFICE
GATT:CTRL:0101  → Preset FULL
GATT:CTRL:0102  → Preset NIGHT (volume capped at 60%)
GATT:CTRL:0103  → Preset SPEECH
GATT:CTRL:0201  → Loudness ON
GATT:CTRL:0401  → Mute ON
GATT:CTRL:0501  → Duck ON (-12dB)
GATT:CTRL:0601  → Normalizer ON
GATT:CTRL:073C  → Volume 60% (0x3C = 60)
GATT:CTRL:0801  → Bypass ON (passthrough)
GATT:CTRL:0901  → Bass Boost ON
```

## DSP Architecture

### Signal Chain

```
I2S Input
    │
    ▼
┌─────────────────┐
│  Preset EQ      │  ← OFFICE/FULL/NIGHT/SPEECH biquad filters
└────────┬────────┘
         │
    ▼
┌─────────────────┐
│  Loudness       │  ← Low-shelf +6dB @ 150Hz (when enabled)
└────────┬────────┘
         │
    ▼
┌─────────────────┐
│  Bass Boost     │  ← Low-shelf +8dB @ 100Hz (when enabled)
└────────┬────────┘
         │
    ▼
┌─────────────────┐
│  Normalizer/DRC │  ← 2:1 ratio, -12dB threshold (when enabled)
└────────┬────────┘
         │
    ▼
┌─────────────────┐
│  Volume + Trim  │  ← -9dB fixed headroom + perceptual curve
└────────┬────────┘
         │
    ▼
┌─────────────────┐
│  Duck           │  ← -12dB reduction (when enabled)
└────────┬────────┘
         │
    ▼
┌─────────────────┐
│  Mute           │  ← Zero output (when enabled)
└────────┬────────┘
         │
    ▼
┌─────────────────┐
│  Limiter        │  ← Hard clip at ±1.0 (safety)
└────────┬────────┘
         │
    ▼
I2S Output
```

### Preset EQ Curves

| Preset | Description | Filter 1 | Filter 2 |
|--------|-------------|----------|----------|
| OFFICE | Subtle clarity | High-shelf +1.5dB @ 6kHz | - |
| FULL | Enhanced bass+treble | Low-shelf +4dB @ 120Hz | High-shelf +3dB @ 8kHz |
| NIGHT | Reduced bass, vol cap 60% | Low-shelf -3dB @ 150Hz | - |
| SPEECH | Voice clarity | High-pass @ 150Hz | Peak +3dB @ 2.5kHz |

### Loudness Filter

Low-shelf boost for enhancing bass at low listening volumes:
- **Frequency:** 150 Hz
- **Gain:** +6 dB
- **Q:** 0.707 (Butterworth)

### Bass Boost Filter

Additional low-frequency boost for small speakers:
- **Frequency:** 100 Hz
- **Gain:** +8 dB
- **Slope:** 0.7

### Normalizer/DRC Parameters

Transparent dynamic range compression:
- **Threshold:** -12 dB
- **Ratio:** 2:1 (gentle)
- **Attack:** 10 ms
- **Release:** 80 ms
- **Makeup Gain:** +3 dB

### Volume/Trim Curve

Perceptual (logarithmic) mapping:

| Trim | dB (before headroom) |
|------|---------------------|
| 100 | 0 dB |
| 80 | -6 dB |
| 60 | -12 dB |
| 40 | -20 dB |
| 20 | -35 dB |
| 0 | -60 dB (near-mute) |

Fixed -9dB headroom is always applied to prevent clipping when effects are enabled.

## Technical Details

### Audio Configuration

- Sample Rate: 44100 Hz
- Bit Depth: 16-bit
- Channels: Stereo (interleaved L/R)
- Buffer: 256 samples per channel per half-buffer
- Latency: ~5.8 ms per buffer half

### SAI Configuration (I2S)

**SAI2_A (RX - input from ESP32):**
- Mode: Slave RX (clock from ESP32)
- ClockStrobing: Rising edge (sample data on rising BCLK)
- Protocol: I2S Philips (FS before first bit)
- Sync: Asynchronous

**SAI2_B (TX - output to DAC):**
- Mode: Slave TX (synchronized to SAI2_A)
- ClockStrobing: Falling edge (output data on falling BCLK)
- Protocol: I2S Philips
- Sync: Synchronous with SAI2_A

### Interrupt Priorities

Critical for reliable UART reception during audio processing:

| Interrupt | Priority | Notes |
|-----------|----------|-------|
| USART2 (BLE commands) | 0 | Highest - never miss commands |
| DMA1_Stream0 (Audio RX) | 2 | Lower - can be preempted |
| DMA1_Stream1 (Audio TX) | 2 | Lower - can be preempted |

### Memory Placement

DMA1/DMA2 on STM32H7 can ONLY access D2 SRAM (0x30000000). Audio buffers must be placed there:

```c
__attribute__((section(".RAM_D2"))) int16_t audio_rx_buffer[...];
__attribute__((section(".RAM_D2"))) int16_t audio_tx_buffer[...];
```

## Project Structure

```
42dB_H753_DSP_engine/
├── Core/
│   ├── Inc/
│   │   ├── build_number.h      # Auto-incremented build number
│   │   ├── stm32h7xx_hal_conf.h
│   │   └── stm32h7xx_it.h
│   └── Src/
│       ├── main.c              # Main application + DSP processing
│       ├── stm32h7xx_hal_msp.c # Peripheral GPIO/clock init
│       ├── stm32h7xx_it.c      # Interrupt handlers
│       └── system_stm32h7xx.c  # System initialization
├── Drivers/
│   ├── CMSIS/                  # ARM CMSIS headers
│   └── STM32H7xx_HAL_Driver/   # ST HAL library
├── STM32H753XX_FLASH.ld        # Linker script (D2 SRAM section)
├── Makefile
├── flash.sh                    # Flashing script
├── README.md                   # This file
└── TECHNICAL.md                # Detailed technical documentation
```

## Changelog

### v0.3.0 (2026-02-06)
- Implemented all DSP features: presets, loudness, bass boost, normalizer, duck, mute, bypass
- Fixed volume cap for NIGHT preset (non-destructive)
- Tuned DRC for transparent compression (no pumping)
- Fixed headroom approach (-9dB) prevents volume jumps

### v0.2.0 (2026-02-04)
- I2S passthrough working with 74HCT04 buffer
- UART command reception with interrupt priority fix
- Basic loudness and mute

### v0.1.0 (2026-02-03)
- Initial SAI/DMA configuration
- Debug console via ST-LINK VCP

## Troubleshooting

### No Audio Output

1. Check PCM5102A SCK is grounded
2. Verify ESP32 is playing audio (check with scope)
3. Check 74HCT04 power (5V) and signal routing
4. Verify BCLK/LRCK go to both STM32 and DAC

### Distorted Audio

1. Check I2S format matches (Philips, 16-bit)
2. Verify volume/trim not too high with effects enabled
3. Check for ground loops (use common ground)

### UART Commands Not Working

1. Check interrupt priorities (UART must be highest)
2. Verify ESP32 TX→STM32 RX wiring
3. Check baud rate matches (115200)
4. Look at debug console for received commands

### Volume Drops When Enabling Effects

Fixed in v0.3.0. If still occurring, ensure you're using the fixed headroom approach (always -9dB).

## License

MIT License - See LICENSE file.

## Author

Robin Kluit

## References

- [Audio EQ Cookbook](https://www.w3.org/2011/audio/audio-eq-cookbook.html) - Robert Bristow-Johnson's biquad filter formulas
- [STM32H753 Reference Manual (RM0433)](https://www.st.com/resource/en/reference_manual/rm0433)
- [STM32H7 SAI Application Note (AN5543)](https://www.st.com/resource/en/application_note/an5543)
