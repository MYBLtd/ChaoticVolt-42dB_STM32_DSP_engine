# ESP32 Bluetooth Speaker with DSP and BLE Control

A Bluetooth audio receiver with real-time DSP processing and BLE GATT control interface for ESP32.

**Version:** 2.3.0 | **Date:** 2026-01-28

## Features

- **Bluetooth A2DP Sink** - Receive high-quality audio from phones, tablets, and computers
- **Real-time DSP Processing** - EQ presets, loudness enhancement, bass boost, normalizer, and output limiting
- **BLE GATT Control** - Change DSP settings wirelessly using any generic BLE app or the 42 Decibels iOS app
- **Bass Boost** - Dedicated +8dB low-shelf boost at 100Hz for small speakers
- **Mute & DSP Bypass** - Instant mute and bypass mode for debugging (bypass keeps safety processing)
- **Audio Duck** - Panic button for instant volume reduction
- **Normalizer (DRC)** - Dynamic range compression for late-night listening
- **Volume Control** - Device-side volume trim with preset-based caps
- **OTA Firmware Updates** - Over-the-air updates via BLE provisioning + WiFi download
- **GalacticStatus** - Extended status reporting with periodic notifications (2x/sec)
- **Persistent Settings** - Settings survive power cycles via NVS flash storage
- **iOS Compatible** - Secure Simple Pairing (SSP) for seamless iOS pairing

## Hardware Requirements

- ESP32-WROOM-32 or compatible module
- I2S DAC (e.g., MAX98357A, PCM5102)
- Speaker/amplifier

### Default I2S Pin Configuration

| Signal | GPIO |
|--------|------|
| BCK (Bit Clock) | GPIO26 |
| WS (Word Select/LRCK) | GPIO25 |
| DATA (Serial Data) | GPIO22 |

## Software Requirements

- ESP-IDF v6.0-beta1 or later
- Python 3.8+

## Building

```bash
# Set up ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Configure target
idf.py set-target esp32

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

### Creating a Merged Binary

For flashing without ESP-IDF tools:

```bash
idf.py merge-bin
# Creates build/merged-binary.bin
# Flash at offset 0x0
```

## Usage

### Bluetooth Audio (A2DP)

1. Power on the ESP32
2. Search for "42 Decibels" in your device's Bluetooth settings
3. Pair and connect
4. Play audio - it will stream to the speaker

### DSP Control (BLE GATT)

Use any BLE app (nRF Connect, LightBlue) to control DSP settings:

1. Scan for BLE devices
2. Connect to "42 Decibels"
3. Find the DSP Control Service (UUID: `00000001-1234-5678-9ABC-DEF012345678`)
4. Write commands to the Control characteristic

## BLE Protocol

### Service UUID

```
DSP Control Service: 00000001-1234-5678-9ABC-DEF012345678
```

### Characteristics

| Characteristic | UUID | Properties | Size |
|----------------|------|------------|------|
| Control Write | `00000002-1234-5678-9ABC-DEF012345678` | Write, Write Without Response | 2 bytes |
| Status Notify | `00000003-1234-5678-9ABC-DEF012345678` | Read, Notify | 4 bytes |
| GalacticStatus | `00000004-1234-5678-9ABC-DEF012345678` | Read, Notify | 7 bytes |
| OTA Credentials | `00000005-1234-5678-9ABC-DEF012345678` | Write | 98 bytes |
| OTA URL | `00000006-1234-5678-9ABC-DEF012345678` | Write | 258 bytes |
| OTA Control | `00000007-1234-5678-9ABC-DEF012345678` | Write | 2 bytes |
| OTA Status | `00000008-1234-5678-9ABC-DEF012345678` | Read, Notify | 8 bytes |

### Command Format

Commands are 2 bytes: `[CMD] [VALUE]`

| CMD | Name | Value | Description |
|-----|------|-------|-------------|
| `0x01` | SET_PRESET | `0x00` | OFFICE - Mild EQ for background listening |
| `0x01` | SET_PRESET | `0x01` | FULL - Rich bass and treble enhancement |
| `0x01` | SET_PRESET | `0x02` | NIGHT - Balanced for low volume listening |
| `0x01` | SET_PRESET | `0x03` | SPEECH - Voice clarity for podcasts |
| `0x02` | SET_LOUDNESS | `0x00`/`0x01` | Loudness OFF/ON |
| `0x03` | GET_STATUS | `0x00` | Request status notification |
| `0x04` | SET_MUTE | `0x00`/`0x01` | Unmute/Mute audio |
| `0x05` | SET_AUDIO_DUCK | `0x00`/`0x01` | Audio Duck OFF/ON (panic volume) |
| `0x06` | SET_NORMALIZER | `0x00`/`0x01` | Normalizer (DRC) OFF/ON |
| `0x07` | SET_VOLUME | `0x00-0x64` | Volume trim (0=mute, 100=full) |
| `0x08` | SET_BYPASS | `0x00`/`0x01` | DSP Bypass OFF/ON (skip EQ, keep safety) |
| `0x09` | SET_BASS_BOOST | `0x00`/`0x01` | Bass Boost OFF/ON (+8dB @ 100Hz) |

### Status Notification Format (4 bytes)

Format: `[VERSION] [PRESET] [LOUDNESS] [FLAGS]`

| Byte | Description |
|------|-------------|
| VERSION | Protocol version (0x01) |
| PRESET | Current preset (0-3) |
| LOUDNESS | Loudness state (0/1) |
| FLAGS | Status bitfield (see below) |

**FLAGS bitfield:**
- Bit 0: Limiter active (always 1)
- Bit 1: Clipping detected
- Bit 2: Thermal warning
- Bit 3: Muted
- Bit 4: Audio Duck active
- Bit 5: Normalizer active
- Bit 6: DSP Bypass active

### GalacticStatus Format (7 bytes)

Extended status with periodic notifications (every 500ms when subscribed).

Format: `[VER] [PRESET] [FLAGS] [ENERGY] [VOLUME] [BATTERY] [LAST_CONTACT]`

| Byte | Field | Description |
|------|-------|-------------|
| 0 | VER | Protocol version (always 0x42) |
| 1 | PRESET | Current preset (0-3) |
| 2 | FLAGS | Shield status bitfield |
| 3 | ENERGY | Reserved (0-100) |
| 4 | VOLUME | Effective volume level (0-100) |
| 5 | BATTERY | Reserved (0-100) |
| 6 | LAST_CONTACT | Seconds since last BLE communication (0-255) |

**Shield Status FLAGS (byte 2):**
- Bit 0 (0x01): Muted
- Bit 1 (0x02): Audio Duck active
- Bit 2 (0x04): Loudness enabled
- Bit 3 (0x08): Normalizer enabled
- Bit 4 (0x10): DSP Bypass active
- Bit 5 (0x20): Bass Boost enabled

### Example Commands (Hex)

```
0100  - Set preset to OFFICE
0101  - Set preset to FULL
0102  - Set preset to NIGHT
0103  - Set preset to SPEECH
0200  - Disable loudness
0201  - Enable loudness
0300  - Request status
0400  - Unmute
0401  - Mute
0500  - Audio Duck OFF (normal volume)
0501  - Audio Duck ON (panic mode, -12dB)
0600  - Normalizer OFF
0601  - Normalizer ON
0764  - Set volume to 100%
073C  - Set volume to 60%
0700  - Set volume to 0% (mute)
0800  - DSP Bypass OFF (full DSP processing)
0801  - DSP Bypass ON (skip EQ, keep safety)
0900  - Bass Boost OFF
0901  - Bass Boost ON (+8dB @ 100Hz)
```

### OTA Commands (Write to OTA Control characteristic)

| CMD | Name | Description |
|-----|------|-------------|
| `0x10` | START | Start OTA download process |
| `0x11` | CANCEL | Cancel active OTA |
| `0x12` | REBOOT | Reboot to new firmware |
| `0x13` | GET_VERSION | Get current firmware version |
| `0x14` | ROLLBACK | Rollback to previous firmware |
| `0x15` | VALIDATE | Mark new firmware as valid |

### OTA Status Format (8 bytes)

Format: `[STATE] [ERROR] [PROGRESS] [DL_KB_L] [DL_KB_H] [TOTAL_KB_L] [TOTAL_KB_H] [RSSI]`

| Byte | Field | Description |
|------|-------|-------------|
| 0 | STATE | OTA state (0x00=Idle, 0x05=Downloading, 0x07=Success, 0xFF=Error) |
| 1 | ERROR | Error code (0=none, see Protocol.md) |
| 2 | PROGRESS | Download progress 0-100% |
| 3-4 | DOWNLOADED_KB | Bytes downloaded (little-endian KB) |
| 5-6 | TOTAL_KB | Total firmware size (little-endian KB) |
| 7 | RSSI | WiFi signal strength (signed dBm)

## DSP Architecture

### Signal Chain

```
Input → Pre-gain (-3dB) → HPF (95Hz) → Preset EQ → Loudness → Bass Boost → Normalizer → Limiter → Volume → Duck → Mute → Output
        ↑
        └── Always applied (even in Bypass mode: skips HPF through Bass Boost, keeps safety stages)
```

### Components

1. **Pre-gain**: -3 dB headroom for EQ boosts (always active, even in bypass)
2. **High-pass Filter**: 2nd-order Butterworth at 95 Hz (driver protection)
3. **Preset EQ**: 4-band parametric EQ per preset
4. **Loudness Overlay**: 2-band shelf EQ (bass +12dB @ 80Hz, treble +6dB @ 12kHz)
5. **Bass Boost**: Low-shelf +8dB @ 100Hz (new in v2.3)
6. **Normalizer**: Block-based DRC (threshold -20dB, ratio 4:1, +6dB makeup)
7. **Limiter**: Peak limiter at -1 dBFS (3ms attack, 120ms release) - always active
8. **Volume Trim**: Device-side volume control (0-100, capped by preset/normalizer)
9. **Audio Duck**: Panic button volume reduction (-12dB)
10. **Mute**: Final output gate

### Bypass Mode Behavior (v2.3)

When bypass is enabled, the following stages are **skipped**:
- HPF, Preset EQ, Loudness, Bass Boost, Normalizer

The following stages **remain active** for safety:
- Pre-gain (headroom), Limiter, Volume, Audio Duck, Mute

This ensures hot-mastered tracks don't clip even when testing "raw" audio.

### Preset EQ Curves

#### OFFICE (Background/Mild)
| Band | Type | Frequency | Gain | Q/Slope |
|------|------|-----------|------|---------|
| 1 | Low-shelf | 150 Hz | +3.0 dB | S=0.7 |
| 2 | Peaking | 350 Hz | -1.5 dB | Q=1.0 |
| 3 | Peaking | 3000 Hz | +1.0 dB | Q=1.0 |
| 4 | High-shelf | 9000 Hz | +2.0 dB | S=0.7 |

#### FULL (Rich/Enhanced)
| Band | Type | Frequency | Gain | Q/Slope |
|------|------|-----------|------|---------|
| 1 | Low-shelf | 100 Hz | +9.0 dB | S=0.7 |
| 2 | Peaking | 300 Hz | -2.0 dB | Q=1.0 |
| 3 | Peaking | 3500 Hz | +3.0 dB | Q=1.2 |
| 4 | High-shelf | 10000 Hz | +5.0 dB | S=0.7 |

#### NIGHT (Low Volume)
| Band | Type | Frequency | Gain | Q/Slope |
|------|------|-----------|------|---------|
| 1 | Low-shelf | 160 Hz | +2.5 dB | S=0.8 |
| 2 | Peaking | 350 Hz | -1.0 dB | Q=1.0 |
| 3 | Peaking | 2500 Hz | +1.0 dB | Q=1.0 |
| 4 | High-shelf | 9000 Hz | +1.0 dB | S=0.7 |

#### SPEECH (Voice Clarity)
| Band | Type | Frequency | Gain | Q/Slope |
|------|------|-----------|------|---------|
| 1 | Low-shelf | 170 Hz | -2.0 dB | S=0.8 |
| 2 | Peaking | 300 Hz | -1.0 dB | Q=1.0 |
| 3 | Peaking | 3200 Hz | +3.0 dB | Q=1.0 |
| 4 | Peaking | 7500 Hz | -1.0 dB | Q=2.0 |

### Loudness Overlay (Aggressive)
| Band | Type | Frequency | Gain | Q/Slope |
|------|------|-----------|------|---------|
| L1 | Low-shelf | 80 Hz | +12.0 dB | S=0.6 |
| L2 | High-shelf | 12000 Hz | +6.0 dB | S=0.5 |

### Bass Boost (New in v2.3)
| Type | Frequency | Gain | Slope |
|------|-----------|------|-------|
| Low-shelf | 100 Hz | +8.0 dB | S=0.7 |

## Project Structure

```
├── CMakeLists.txt
├── README.md
├── Protocol.md                      # BLE Protocol Documentation
├── FSD-DSP-001_ESP32_BLE_GATT.md    # Functional Specification Document
├── partitions_ota.csv               # OTA partition table
├── sdkconfig.defaults
└── main/
    ├── CMakeLists.txt
    ├── main.c                       # Application entry point
    ├── dsp_processor.h              # DSP API
    ├── dsp_processor.c              # DSP implementation (biquads, presets, limiter)
    ├── ble_gatt_dsp.h               # BLE GATT service API
    ├── ble_gatt_dsp.c               # BLE GATT implementation
    ├── nvs_settings.h               # Persistent storage API
    ├── nvs_settings.c               # NVS implementation with debouncing
    ├── ota_manager.h                # OTA update manager API
    ├── ota_manager.c                # OTA state machine and download logic
    ├── wifi_manager.h               # WiFi connection manager API
    └── wifi_manager.c               # WiFi STA mode for OTA downloads
```

## Configuration

### sdkconfig.defaults

Key configuration options:

```
# Dual-mode Bluetooth (Classic + BLE)
CONFIG_BTDM_CTRL_MODE_BTDM=y
CONFIG_BT_CLASSIC_ENABLED=y
CONFIG_BT_A2DP_ENABLE=y
CONFIG_BT_BLE_ENABLED=y
CONFIG_BT_GATTS_ENABLE=y

# Performance optimization
CONFIG_COMPILER_OPTIMIZATION_PERF=y
```

### Customization

- **Device Name**: Change `BT_DEVICE_NAME` in `main.c`
- **I2S Pins**: Modify `I2S_BCK_PIN`, `I2S_WS_PIN`, `I2S_DATA_PIN` in `main.c`
- **EQ Curves**: Edit `preset_params` array in `dsp_processor.c`
- **Limiter Settings**: Adjust `DSP_LIMITER_*` defines in `dsp_processor.h`
- **Bass Boost Settings**: Adjust `DSP_BASS_BOOST_*` defines in `dsp_processor.h`

## Changelog

### v2.3.0 (2026-01-28)
- **New:** Bass Boost feature (+8dB @ 100Hz) - command 0x09
- **New:** DSP Bypass command (0x08) - skips EQ, keeps safety processing
- **Fixed:** Bypass mode now keeps pre-gain, limiter, volume (prevents hot tracks from clipping)
- **Fixed:** Audio Duck command handler (was logging wrong message)
- **Fixed:** Sample rate detection for multi-bit SBC bitmasks (fallback to 44.1kHz)
- **Updated:** GalacticStatus shieldStatus now includes Bypass (bit 4) and Bass Boost (bit 5)

### v2.2.0 (2026-01-26)
- DSP optimizations with block-based normalizer
- Added DSP bypass mode (debug)
- Pre-gain adjusted to -3dB

### v2.1.0 (2026-01-23)
- OTA firmware update support (BLE+WiFi hybrid)
- GalacticStatus periodic notifications

### v2.0.0 (2026-01-22)
- Volume control with preset-based caps
- Normalizer/DRC feature
- Audio Duck panic button

### v1.1.0 (2026-01-21)
- Initial release with DSP presets and loudness

## License

MIT License - See LICENSE file for details.

## Author

Robin Kluit

## Acknowledgments

- Espressif Systems for ESP-IDF
- Audio EQ Cookbook by Robert Bristow-Johnson for biquad filter formulas
