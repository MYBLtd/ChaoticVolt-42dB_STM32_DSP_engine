# 42 Decibels BLE Protocol Documentation

**Version:** 2.4.3
**Date:** 2026-02-18

## Overview

This document describes the BLE GATT protocol for controlling the ESP32 Bluetooth Speaker with DSP. The ESP32 receives BLE commands and relays them via UART to the STM32H753 DSP engine. Status responses flow in reverse: the STM32 notifies its state over UART; the ESP32 maps this to the BLE characteristics below.

---

## Service UUID

```
DSP Control Service: 00000001-1234-5678-9ABC-DEF012345678
```

---

## Control Commands (Characteristic: 0x0002)

**UUID:** `00000002-1234-5678-9ABC-DEF012345678`
**Properties:** Write, Write Without Response
**Size:** 2 bytes

All commands are sent as 2-byte packets: `[COMMAND_TYPE, VALUE]`

### Command Types

| Command | Byte 0 | Byte 1 | Description |
|---------|--------|--------|-------------|
| **Set Preset** | `0x01` | `0x00-0x03` | Change DSP preset |
| **Set Loudness** | `0x02` | `0x00-0x01` | Enable/disable loudness compensation |
| **Request Status** | `0x03` | `0x00` | Request full status update |
| **Set Mute** | `0x04` | `0x00-0x01` | Mute/unmute audio completely |
| **Set Audio Duck** | `0x05` | `0x00-0x01` | Enable/disable Audio Duck (panic volume reduction) |
| **Set Normalizer** | `0x06` | `0x00-0x01` | Enable/disable Normalizer/DRC |
| **Set Volume** | `0x07` | `0x00-0x64` | Set volume trim (0=mute, 100=full) |
| **Set Bypass** | `0x08` | `0x00-0x01` | Enable/disable DSP Bypass (skip EQ, keep safety) |
| **Set Bass Boost** | `0x09` | `0x00-0x01` | Enable/disable Bass Boost (+8dB @ 100Hz) |
| **Set Sine Test** | `0x0A` | `0x00-0x01` | Enable/disable internal 1kHz sine test tone |

### Preset Values (for Command 0x01)

| Value | Preset | Description |
|-------|--------|-------------|
| `0x00` | OFFICE | Balanced sound for office/desk use |
| `0x01` | FULL | Full-range audio with enhanced bass |
| `0x02` | NIGHT | Balanced for low volume listening (volume capped at 60%) |
| `0x03` | SPEECH | Optimized for voice content |

### Command Examples

```
// Set preset to NIGHT
0x01 0x02

// Enable loudness compensation
0x02 0x01

// Mute audio
0x04 0x01

// Unmute audio
0x04 0x00

// Enable Audio Duck (reduce volume to ~25%)
0x05 0x01

// Disable Audio Duck (restore normal volume)
0x05 0x00

// Enable Normalizer (dynamic range compression)
0x06 0x01

// Disable Normalizer
0x06 0x00

// Set volume to 100% (full)
0x07 0x64

// Set volume to 60%
0x07 0x3C

// Set volume to 0% (mute)
0x07 0x00

// Enable DSP Bypass (skip EQ, keep safety processing)
0x08 0x01

// Disable DSP Bypass (full DSP processing)
0x08 0x00

// Enable Bass Boost (+8dB @ 100Hz)
0x09 0x01

// Disable Bass Boost
0x09 0x00

// Enable Sine Test (1kHz internal tone, ignores audio input)
0x0A 0x01

// Disable Sine Test
0x0A 0x00
```

---

## Status Responses (Characteristic: 0x0003)

**UUID:** `00000003-1234-5678-9ABC-DEF012345678`
**Properties:** Read, Notify
**Size:** 4 bytes

The device sends status updates via notifications on this characteristic after each command.

### Response Format

```
[VERSION][PRESET][LOUDNESS][FLAGS]
   0        1        2        3
```

| Byte | Field    | Type     | Description              |
|------|----------|----------|--------------------------|
| 0    | VERSION  | `uint8`  | Protocol version (0x01)  |
| 1    | PRESET   | `uint8`  | Current preset (0-3)     |
| 2    | LOUDNESS | `uint8`  | Loudness state (0/1)     |
| 3    | FLAGS    | bitfield | Status flags             |

### FLAGS Bitfield (Byte 3)

> **Note (v2.4.3 fix):** FLAGS byte 3 uses its own bit layout that is **different** from the GalacticStatus Shield byte. Do not copy `s_dsp_flags` directly — remap explicitly.

| Bit | Mask | Field           | Description                    |
|-----|------|-----------------|--------------------------------|
| 0   | 0x01 | Limiter Active  | Always 1 (limiter is always on) |
| 1   | 0x02 | Clipping        | Clipping detected (optional)   |
| 2   | 0x04 | Thermal Warning | Thermal warning (optional)     |
| 3   | 0x08 | Muted           | Audio is muted                 |
| 4   | 0x10 | Audio Duck      | Volume reduced (panic mode)    |
| 5   | 0x20 | Normalizer      | DRC enabled                    |
| 6   | 0x40 | Bypass          | DSP Bypass active              |
| 7   | —    | Reserved        | Future use                     |

---

## Galactic Status (Characteristic: 0x0004)

**UUID:** `00000004-1234-5678-9ABC-DEF012345678`
**Properties:** Read, Notify
**Size:** 7 bytes

This characteristic provides a comprehensive status snapshot. It is automatically notified every 500ms when notifications are enabled.

### Packet Format

```
[VER][PRESET][FLAGS][ENERGY][VOLUME][BATTERY][LAST_CONTACT]
  0     1       2      3       4       5          6
```

### Byte Descriptions

| Byte | Field        | Type     | Description                          |
|------|--------------|----------|--------------------------------------|
| 0    | VER          | `uint8`  | Protocol version (always `0x42`)     |
| 1    | PRESET       | `uint8`  | Active preset (0-3)                  |
| 2    | FLAGS        | bitfield | Shield status flags (see below)      |
| 3    | ENERGY       | `uint8`  | Reserved (0-100)                     |
| 4    | VOLUME       | `uint8`  | Effective volume level (0-100)       |
| 5    | BATTERY      | `uint8`  | Battery percentage placeholder (0-100)|
| 6    | LAST_CONTACT | `uint8`  | Seconds since last BLE communication |

### Shield Status Bitfield (Byte 2)

> **Note (v2.4.3 fix):** Bit 4 = Bypass, Bit 5 = Bass Boost. These were swapped prior to v2.4.3.

| Bit | Mask   | Field      | Description                       |
|-----|--------|------------|-----------------------------------|
| 0   | `0x01` | Muted      | Audio is muted                    |
| 1   | `0x02` | Audio Duck | Volume reduced (panic mode)       |
| 2   | `0x04` | Loudness   | Loudness compensation enabled     |
| 3   | `0x08` | Normalizer | Dynamic range compression enabled |
| 4   | `0x10` | Bypass     | DSP Bypass active                 |
| 5   | `0x20` | Bass Boost | Bass Boost enabled                |
| 6-7 | —      | Reserved   | Future use                        |

### Last Contact Behavior

The `LAST_CONTACT` field (byte 6) shows seconds since the last BLE communication:

- **0 seconds** = Just communicated
- **1-5 seconds** = Very recent
- **30+ seconds** = Getting stale
- **255 seconds** = Maximum (>4 minutes)

Communication includes incoming commands, reads, and outgoing notifications. Since GalacticStatus is notified every 500ms, this value is typically `0x00` or `0x01` during an active session.

### Example Galactic Status

```
42 01 26 64 50 64 00
```

Interpretation:
- Protocol Version: `0x42` ✓
- Preset: `0x01` (FULL)
- Shield Status: `0x26` = bits 1, 2, 5 set
  - Muted: NO
  - Audio Duck: YES (bit 1)
  - Loudness: YES (bit 2)
  - Normalizer: NO
  - Bypass: NO
  - Bass Boost: YES (bit 5)
- Energy Core: 100%
- Volume: 50%
- Battery: 100%
- Last Contact: 0 seconds ago

---

## Implementation Details

### DSP Bypass (v2.3+)

When DSP Bypass is enabled (`0x08 0x01`), on the STM32 DSP engine:

**Skipped stages:**
- Preset EQ
- Loudness overlay
- Bass Boost
- Normalizer/DRC

**Kept stages (for safety):**
- Volume trim (with -9dB fixed headroom)
- Duck
- Mute
- Limiter (hard clip)

On the ESP32, Bypass also skips the High-pass filter (95Hz), Preset EQ (4 bands), Loudness overlay, Bass Boost, and Normalizer. Pre-gain, Limiter, Volume, Duck, and Mute remain active.

This ensures hot-mastered tracks don't clip the DAC/amplifier even when testing "raw" audio.

### Bass Boost (v2.3+)

When Bass Boost is enabled (`0x09 0x01`):
1. Apply low-shelf filter: +8dB @ 100Hz (S=0.7)
2. Use smooth gain ramp (no clicks — filter state reset on disable)
3. Set bit 5 (0x20) in Shield Status byte
4. Setting is persisted to NVS on ESP32

### Sine Test (v2.4.1+)

When Sine Test is enabled (`0x0A 0x01`):
1. STM32 generates a 1kHz sine tone internally at -12dB
2. Audio input from ESP32 I2S is ignored
3. Entire DSP chain (including Bypass) is skipped
4. Useful for verifying PCM5102A DAC and amplifier independently of Bluetooth

Sine Test is a diagnostic feature — it is not persisted to NVS and resets on power cycle.

### Audio Duck

When Audio Duck is enabled (`0x05 0x01`):
1. Apply -12 dB gain reduction (~25% volume)
2. Use smooth gain transition (no clicks)
3. Set bit 1 in Shield Status byte
4. Update and notify GalacticStatus

### Normalizer/DRC

**STM32 DSP Engine parameters:**
- Threshold: -12 dB
- Ratio: 2:1 (gentle, transparent)
- Attack: ~10 ms
- Release: ~80 ms
- Makeup gain: +3 dB

**ESP32 DSP Engine parameters:**
- Threshold: -20 dB
- Ratio: 4:1
- Attack: 7 ms
- Release: 150 ms
- Makeup gain: +6 dB

### Mute

When Mute is enabled (`0x04 0x01`):
1. Apply smooth gain fade to 0 (no click)
2. Set bit 0 in Shield Status byte
3. Update and notify GalacticStatus

### Volume Control

When Volume is set (`0x07 0xNN`):
1. Accept value 0–100 (0x00–0x64)
2. Apply volume cap based on preset:
   - OFFICE / FULL / SPEECH: cap = 100
   - NIGHT: cap = 60
3. Apply smooth gain transition (no clicks)
4. Update GalacticStatus byte 4 with effective volume
5. Volume to dB mapping (perceptual):
   - 100 → 0 dB
   - 80 → -6 dB
   - 60 → -12 dB
   - 40 → -20 dB
   - 20 → -35 dB
   - 0 → -60 dB (near-mute)

A fixed -9dB headroom is always applied on the STM32 engine so toggling effects never causes a perceived volume change.

---

## Characteristic Summary

| UUID   | Name           | Properties                    | Purpose                          |
|--------|----------------|-------------------------------|----------------------------------|
| 0x0002 | Control Write  | Write, Write Without Response | Send DSP commands to device      |
| 0x0003 | Status Notify  | Read, Notify                  | Receive simple status updates    |
| 0x0004 | Galactic Status| Read, Notify                  | Receive comprehensive status     |
| 0x0005 | OTA Credentials| Write                         | WiFi SSID + password for OTA     |
| 0x0006 | OTA URL        | Write                         | Firmware download URL            |
| 0x0007 | OTA Control    | Write                         | OTA commands (start/cancel/etc)  |
| 0x0008 | OTA Status     | Read, Notify                  | OTA progress notifications       |

---

## OTA Firmware Update

The ESP32 supports over-the-air firmware updates using a hybrid BLE + WiFi approach:
1. WiFi credentials and firmware URL are provisioned via BLE
2. Device connects to WiFi and downloads firmware
3. Progress is reported via OTA Status notifications
4. Device reboots to new firmware on command

---

## OTA Credentials (Characteristic: 0x0005)

**UUID:** `00000005-1234-5678-9ABC-DEF012345678`
**Properties:** Write
**Size:** Up to 98 bytes

### Format

```
[SSID (max 32 bytes)] [0x00 separator] [PASSWORD (max 64 bytes)]
```

Write WiFi credentials as: `SSID\0PASSWORD` (null-separated).

---

## OTA URL (Characteristic: 0x0006)

**UUID:** `00000006-1234-5678-9ABC-DEF012345678`
**Properties:** Write
**Size:** Up to 258 bytes

Write the firmware URL as a null-terminated string.

---

## OTA Control (Characteristic: 0x0007)

**UUID:** `00000007-1234-5678-9ABC-DEF012345678`
**Properties:** Write
**Size:** 2 bytes — `[CMD] [PARAM]`

### OTA Commands

| CMD | Name | Param | Description |
|-----|------|-------|-------------|
| `0x10` | START | `0x00` | Start OTA download process |
| `0x11` | CANCEL | `0x00` | Cancel active OTA operation |
| `0x12` | REBOOT | `0x00` | Reboot to apply new firmware |
| `0x13` | GET_VERSION | `0x00` | Request firmware version |
| `0x14` | ROLLBACK | `0x00` | Rollback to previous firmware |
| `0x15` | VALIDATE | `0x00` | Mark new firmware as valid |

---

## OTA Status (Characteristic: 0x0008)

**UUID:** `00000008-1234-5678-9ABC-DEF012345678`
**Properties:** Read, Notify
**Size:** 8 bytes

```
[STATE][ERROR][PROGRESS][DL_KB_L][DL_KB_H][TOTAL_KB_L][TOTAL_KB_H][RSSI]
   0      1       2         3        4         5           6        7
```

### OTA States

| Value | State | Description |
|-------|-------|-------------|
| `0x00` | IDLE | Ready for OTA |
| `0x01` | CREDS_RECEIVED | WiFi credentials received |
| `0x02` | URL_RECEIVED | Firmware URL received |
| `0x03` | WIFI_CONNECTING | Connecting to WiFi |
| `0x04` | WIFI_CONNECTED | WiFi connected, ready to download |
| `0x05` | DOWNLOADING | Firmware download in progress |
| `0x06` | VERIFYING | Verifying firmware integrity |
| `0x07` | SUCCESS | OTA complete, ready for reboot |
| `0x08` | PENDING_VERIFY | New firmware booted, awaiting validation |
| `0xFF` | ERROR | Error occurred (see ERROR byte) |

### OTA Error Codes

| Value | Error | Description |
|-------|-------|-------------|
| `0x00` | NONE | No error |
| `0x01` | WIFI_CONNECT | WiFi connection failed |
| `0x02` | HTTP_CONNECT | HTTP connection failed |
| `0x03` | HTTP_RESPONSE | HTTP 4xx/5xx error |
| `0x04` | DOWNLOAD | Download failed (network error) |
| `0x05` | VERIFY | Firmware verification failed |
| `0x06` | WRITE | Flash write failed |
| `0x07` | NO_CREDS | No WiFi credentials provided |
| `0x08` | NO_URL | No firmware URL provided |
| `0x09` | INVALID_IMAGE | Invalid firmware image |
| `0x0A` | CANCELLED | OTA cancelled by user |
| `0x0B` | ROLLBACK_FAILED | Rollback operation failed |

---

## Quick Reference

### DSP Commands (Write to 0x0002)

```
0100  - Set preset to OFFICE
0101  - Set preset to FULL
0102  - Set preset to NIGHT (volume capped at 60%)
0103  - Set preset to SPEECH
0200  - Disable loudness
0201  - Enable loudness (+6dB @ 150Hz)
0300  - Request status
0400  - Unmute
0401  - Mute
0500  - Disable Audio Duck
0501  - Enable Audio Duck (-12dB panic)
0600  - Disable Normalizer
0601  - Enable Normalizer
0764  - Set volume to 100%
073C  - Set volume to 60%
0700  - Set volume to 0%
0800  - Disable DSP Bypass (full DSP)
0801  - Enable DSP Bypass (skip EQ, keep safety)
0900  - Disable Bass Boost
0901  - Enable Bass Boost (+8dB @ 100Hz)
0A00  - Disable Sine Test
0A01  - Enable Sine Test (1kHz internal tone)
```

### OTA Commands (Write to 0x0007)

```
1000  - Start OTA download
1100  - Cancel OTA
1200  - Reboot to new firmware
1300  - Get firmware version
1400  - Rollback to previous firmware
1500  - Validate new firmware
```

---

## Testing Checklist

### DSP Features
- [x] Mute silences audio completely
- [x] Mute uses smooth fade (no clicks)
- [x] Audio Duck reduces volume to ~25%
- [x] Audio Duck uses smooth transition
- [x] Normalizer applies compression
- [x] Volume trim 0–100 adjusts audio level
- [x] Volume uses smooth transition (no clicks)
- [x] NIGHT preset caps volume at 60%
- [x] DSP Bypass skips EQ but keeps safety stages
- [x] Bass Boost adds +8dB @ 100Hz
- [x] Bass Boost uses smooth crossfade
- [x] Sine Test generates 1kHz tone, ignores I2S input

### GalacticStatus (v2.4.3+)
- [x] Bit 0 (0x01) reflects Mute state
- [x] Bit 1 (0x02) reflects Audio Duck state
- [x] Bit 2 (0x04) reflects Loudness state
- [x] Bit 3 (0x08) reflects Normalizer state
- [x] Bit 4 (0x10) reflects **Bypass** state (corrected in v2.4.3)
- [x] Bit 5 (0x20) reflects **Bass Boost** state (corrected in v2.4.3)
- [x] Byte 4 reflects effective volume
- [x] Last Contact resets on notifications
- [x] Last Contact shows 0–1 during active connection

### Status Notify FLAGS (Byte 3, v2.4.3+)
- [x] Bit 0 (0x01) = Limiter always 1
- [x] Bit 3 (0x08) = Mute state
- [x] Bit 4 (0x10) = Audio Duck state
- [x] Bit 5 (0x20) = Normalizer state
- [x] Bit 6 (0x40) = Bypass state
- [x] FLAGS correctly remapped (not a raw copy of internal flags)

### OTA Features
- [x] WiFi credentials can be written to 0x0005
- [x] Firmware URL can be written to 0x0006
- [x] OTA Start command initiates WiFi connection
- [x] OTA Status notifications show progress
- [x] OTA can be cancelled mid-download
- [x] Reboot command applies new firmware
- [x] Rollback command restores previous firmware
- [x] Validate command prevents auto-rollback

---

## Changelog

### v2.4.3 (2026-02-18)
- **Fixed:** Bypass/Bass Boost bit assignments in GalacticStatus Shield byte were swapped. Bit 4 (0x10) is now correctly Bypass; bit 5 (0x20) is Bass Boost.
- **Fixed:** Status Notify FLAGS byte (0x0003[3]) was a raw copy of internal `s_dsp_flags`. Now correctly remapped: bit 0 = limiter, bit 3 = mute, bit 4 = duck, bit 5 = normalizer, bit 6 = bypass.

### v2.4.2 (2026-02-16)
- **Fixed:** Transient DSP state (duck, mute) now correctly reflected in BLE status notifications after commands.

### v2.4.1 (2026-02-14)
- **New:** Sine Test mode (command 0x0A) — 1kHz internal tone for DAC/amp verification
- **Fixed:** Audio path ring buffer with 50ms pre-buffering to prevent DMA underrun at startup
- **New:** UART echo of BLE commands to STM32 DSP engine (GPIO4/GPIO5 @ 115200)

### v2.4.0 (2026-02-10)
- **New:** V4 architecture — dual Classic BT A2DP + BLE GATT in same firmware
- **New:** BT RSSI monitoring and SBC quality logging
- **New:** UART echo for external STM32 DSP engine

### v2.3.0 (2026-01-28)
- **New:** Bass Boost feature (+8dB @ 100Hz) — command 0x09
- **New:** DSP Bypass command (0x08) — skips EQ, keeps safety processing
- **Fixed:** Bypass mode keeps pre-gain, limiter, volume (prevents hot tracks from clipping)
- **Updated:** GalacticStatus shieldStatus now includes Bypass (bit 4) and Bass Boost (bit 5)

---

*Last Updated: 2026-02-18 (v2.4.3)*
