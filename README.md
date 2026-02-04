# 42dB DSP Engine

I2S audio DSP processor with BLE control, running on STM32H753 Nucleo board.

## Overview

Real-time audio processing engine:
- **I2S Input** - Receive audio from ADC/codec
- **DSP Processing** - Apply audio effects and filters
- **I2S Output** - Send processed audio to DAC/codec
- **BLE Control** - ESP32 provides wireless parameter control via GATT

## Development Roadmap

- [x] UART communication with ESP32 (BLE control interface)
- [ ] I2S input configuration
- [ ] I2S output (echo/passthrough)
- [ ] Basic DSP processing
- [ ] BLE GATT parameter control
- [ ] Advanced DSP algorithms

## Hardware

- **STM32 Nucleo-H753ZI** - DSP processing
- **ESP32** - BLE GATT interface
- **Audio Codec** - I2S ADC/DAC (TBD)

## Pin Configuration

| Function | STM32 Pin | Connector |
|----------|-----------|-----------|
| ESP32 TX | PD5       | CN9 pin 6 |
| ESP32 RX | PD6       | CN9 pin 4 |
| Debug TX | PD8       | ST-LINK VCP |
| Debug RX | PD9       | ST-LINK VCP |
| I2S      | TBD       | TBD |

## Building

1. Open project in STM32CubeIDE
2. Build: Project > Build All
3. Flash: Run > Debug

## Current Status

ESP32 UART communication working. Received BLE control data displayed as hex dump for debugging.

```
*** 42dB DSP Engine ***
[OK] HAL initialized
[OK] Clock configured
[OK] GPIO initialized
[OK] USART3 (debug console) initialized
[OK] USART2 (ESP32 BLE) initialized

=====================================
42dB DSP Engine v0.1
=====================================
ESP32 BLE: PD5(TX), PD6(RX) @ 115200
Debug: PD8(TX), PD9(RX) @ 115200
I2S: Not configured
=====================================
```

## License

MIT License
