# 42 Decibels BLE Protocol Documentation

**Version:** 2.3.0
**Date:** 2026-01-28

## Overview

This document describes the BLE GATT protocol for controlling the ESP32 Bluetooth Speaker with DSP.

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

### Preset Values (for Command 0x01)

| Value | Preset | Description |
|-------|--------|-------------|
| `0x00` | OFFICE | Balanced sound for office/desk use |
| `0x01` | FULL | Full-range audio with enhanced bass |
| `0x02` | NIGHT | Balanced for low volume listening |
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

| Bit | Mask | Field           | Description                    |
|-----|------|-----------------|--------------------------------|
| 0   | 0x01 | Limiter Active  | Always 1 (limiter is on)       |
| 1   | 0x02 | Clipping        | Clipping detected (optional)   |
| 2   | 0x04 | Thermal Warning | Thermal warning (optional)     |
| 3   | 0x08 | Muted           | Audio is muted                 |
| 4   | 0x10 | Audio Duck      | Volume reduced (panic mode)    |
| 5   | 0x20 | Normalizer      | DRC enabled                    |
| 6   | 0x40 | Bypass          | DSP Bypass active              |
| 7   | -    | Reserved        | Future use                     |

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

| Bit | Mask   | Field      | Description                       |
|-----|--------|------------|-----------------------------------|
| 0   | `0x01` | Muted      | Audio is muted                    |
| 1   | `0x02` | Audio Duck | Volume reduced (panic mode)       |
| 2   | `0x04` | Loudness   | Loudness compensation enabled     |
| 3   | `0x08` | Normalizer | Dynamic range compression enabled |
| 4   | `0x10` | Bypass     | DSP Bypass active (v2.3+)         |
| 5   | `0x20` | Bass Boost | Bass Boost enabled (v2.3+)        |
| 6-7 | -      | Reserved   | Future use                        |

### Last Contact Behavior

The `LAST_CONTACT` field (byte 6) shows seconds since the last BLE communication:

- **0 seconds** = Just communicated
- **1-5 seconds** = Very recent
- **30+ seconds** = Getting stale
- **255 seconds** = Maximum (>4 minutes)

Communication includes:
- Incoming commands (writes)
- Incoming reads
- Outgoing notifications (status updates)

Since GalacticStatus is notified every 500ms, this value should typically be `0x00` or `0x01`.

### Example Galactic Status

```
42 01 26 64 50 64 00
```

Interpretation:
- Protocol Version: `0x42` ✓
- Preset: `0x01` (FULL)
- Shield Status: `0x26` (bits 1, 2, and 5 set)
  - Muted: NO
  - Audio Duck: YES (volume reduced)
  - Loudness: YES
  - Normalizer: NO
  - Bypass: NO
  - Bass Boost: YES
- Energy Core: 100%
- Volume: 50%
- Battery: 100%
- Last Contact: 0 seconds ago

---

## Implementation Details

### DSP Bypass (v2.3+)

When DSP Bypass is enabled (`0x08 0x01`):

**Skipped stages:**
- High-pass filter (95Hz)
- Preset EQ (4 bands)
- Loudness overlay
- Bass Boost
- Normalizer/DRC

**Kept stages (for safety):**
- Pre-gain (-3dB headroom)
- Limiter (-1dBFS peak protection)
- Volume trim
- Audio Duck
- Mute

This ensures hot-mastered tracks don't clip the DAC/amplifier even when testing "raw" audio.

### Bass Boost (v2.3+)

When Bass Boost is enabled (`0x09 0x01`):
1. Apply low-shelf filter: +8dB @ 100Hz (S=0.7)
2. Use smooth gain crossfade (no clicks)
3. Set bit 5 in Shield Status byte
4. Setting is persisted to NVS

Bass Boost is ideal for small speakers that lack natural bass response.

### Audio Duck (FR-21)

When Audio Duck is enabled (`0x05 0x01`):
1. Apply -12 dB gain reduction (~25% volume)
2. Use smooth gain transition (no clicks)
3. Set bit 1 in Shield Status byte
4. Update and notify GalacticStatus

When Audio Duck is disabled (`0x05 0x00`):
1. Restore normal gain (0 dB)
2. Use smooth gain transition
3. Clear bit 1 in Shield Status byte
4. Update and notify GalacticStatus

### Normalizer/DRC (FR-22)

When Normalizer is enabled (`0x06 0x01`):
1. Enable dynamic range compression in DSP chain
2. Settings:
   - Threshold: -20 dB
   - Ratio: 4:1
   - Attack: 7 ms
   - Release: 150 ms
   - Makeup gain: +6 dB
3. Set bit 3 in Shield Status byte
4. Update and notify GalacticStatus

### Mute (FR-21)

When Mute is enabled (`0x04 0x01`):
1. Apply smooth gain fade to 0
2. Set bit 0 in Shield Status byte
3. Update and notify GalacticStatus

### Volume Control (FR-24)

When Volume is set (`0x07 0xNN`):
1. Accept value 0-100 (0x00-0x64)
2. Apply volume cap based on preset:
   - OFFICE/FULL/SPEECH: cap = 100
   - NIGHT: cap = 60
3. If Normalizer is enabled, reduce cap by ~20% for headroom
4. Apply smooth gain transition (no clicks)
5. Update GalacticStatus byte 4 with effective volume
6. Volume to dB mapping:
   - 100 → 0 dB
   - 80 → -6 dB
   - 60 → -12 dB
   - 40 → -20 dB
   - 20 → -35 dB
   - 0 → mute

---

## Characteristic Summary

| UUID   | Name           | Properties                  | Purpose                          |
|--------|----------------|-----------------------------|---------------------------------|
| 0x0002 | Control Write  | Write, Write Without Response | Send commands to device        |
| 0x0003 | Status Notify  | Read, Notify                | Receive simple status updates   |
| 0x0004 | Galactic Status| Read, Notify                | Receive comprehensive status    |
| 0x0005 | OTA Credentials| Write                       | WiFi SSID + password for OTA    |
| 0x0006 | OTA URL        | Write                       | Firmware download URL           |
| 0x0007 | OTA Control    | Write                       | OTA commands (start/cancel/etc) |
| 0x0008 | OTA Status     | Read, Notify                | OTA progress notifications      |

---

## OTA Firmware Update (FR-25)

The device supports over-the-air firmware updates using a hybrid BLE + WiFi approach:
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

### Example

```
// Connect to "MyWiFi" with password "secret123"
4D 79 57 69 46 69 00 73 65 63 72 65 74 31 32 33
```

---

## OTA URL (Characteristic: 0x0006)

**UUID:** `00000006-1234-5678-9ABC-DEF012345678`
**Properties:** Write
**Size:** Up to 258 bytes

### Format

```
[LENGTH_L] [LENGTH_H] [URL bytes...]
```

Or simply write the URL as a null-terminated string.

### Example

```
// Set firmware URL
https://example.com/firmware/v2.3.bin
```

---

## OTA Control (Characteristic: 0x0007)

**UUID:** `00000007-1234-5678-9ABC-DEF012345678`
**Properties:** Write
**Size:** 2 bytes

### Command Format

```
[CMD] [PARAM]
```

### OTA Commands

| CMD | Name | Param | Description |
|-----|------|-------|-------------|
| `0x10` | START | `0x00` | Start OTA download process |
| `0x11` | CANCEL | `0x00` | Cancel active OTA operation |
| `0x12` | REBOOT | `0x00` | Reboot to apply new firmware |
| `0x13` | GET_VERSION | `0x00` | Request firmware version |
| `0x14` | ROLLBACK | `0x00` | Rollback to previous firmware |
| `0x15` | VALIDATE | `0x00` | Mark new firmware as valid |

### Command Examples

```
// Start OTA download (after credentials and URL are set)
0x10 0x00

// Cancel OTA in progress
0x11 0x00

// Reboot to new firmware (after successful download)
0x12 0x00

// Rollback to previous firmware
0x14 0x00

// Validate new firmware (prevents auto-rollback)
0x15 0x00
```

---

## OTA Status (Characteristic: 0x0008)

**UUID:** `00000008-1234-5678-9ABC-DEF012345678`
**Properties:** Read, Notify
**Size:** 8 bytes

Status notifications are sent automatically during OTA operations.

### Packet Format

```
[STATE][ERROR][PROGRESS][DL_KB_L][DL_KB_H][TOTAL_KB_L][TOTAL_KB_H][RSSI]
   0      1       2         3        4         5           6        7
```

### Byte Descriptions

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | STATE | `uint8` | Current OTA state |
| 1 | ERROR | `uint8` | Error code (0 = no error) |
| 2 | PROGRESS | `uint8` | Download progress 0-100% |
| 3-4 | DOWNLOADED_KB | `uint16` | Downloaded size in KB (little-endian) |
| 5-6 | TOTAL_KB | `uint16` | Total firmware size in KB (little-endian) |
| 7 | RSSI | `int8` | WiFi signal strength in dBm |

### OTA States

| Value | State | Description |
|-------|-------|-------------|
| `0x00` | IDLE | Ready for OTA, no operation in progress |
| `0x01` | CREDS_RECEIVED | WiFi credentials received |
| `0x02` | URL_RECEIVED | Firmware URL received |
| `0x03` | WIFI_CONNECTING | Connecting to WiFi network |
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
| `0x02` | HTTP_CONNECT | HTTP connection to server failed |
| `0x03` | HTTP_RESPONSE | HTTP error response (4xx/5xx) |
| `0x04` | DOWNLOAD | Download failed (network error) |
| `0x05` | VERIFY | Firmware verification failed |
| `0x06` | WRITE | Flash write failed |
| `0x07` | NO_CREDS | No WiFi credentials provided |
| `0x08` | NO_URL | No firmware URL provided |
| `0x09` | INVALID_IMAGE | Invalid firmware image |
| `0x0A` | CANCELLED | OTA cancelled by user |
| `0x0B` | ROLLBACK_FAILED | Rollback operation failed |

### Example OTA Status

```
05 00 45 C8 00 20 03 D2
```

Interpretation:
- State: `0x05` (DOWNLOADING)
- Error: `0x00` (no error)
- Progress: `0x45` = 69%
- Downloaded: `0x00C8` = 200 KB
- Total: `0x0320` = 800 KB
- RSSI: `0xD2` = -46 dBm (good signal)

---

## OTA Workflow

### Typical OTA Update Sequence

1. **Provision Credentials**: Write WiFi SSID/password to 0x0005
2. **Set URL**: Write firmware URL to 0x0006
3. **Start OTA**: Write `0x10 0x00` to 0x0007
4. **Monitor Progress**: Subscribe to 0x0008 notifications
5. **Wait for Success**: STATE becomes `0x07`
6. **Reboot**: Write `0x12 0x00` to 0x0007
7. **Validate** (after reboot): Write `0x15 0x00` to 0x0007

### Firmware Rollback

If the new firmware fails to validate within a certain period, or on user request:
1. Write `0x14 0x00` to OTA Control (0x0007)
2. Device reboots to previous firmware

---

## Quick Reference

### DSP Commands (Write to 0x0002)

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
0500  - Disable Audio Duck
0501  - Enable Audio Duck (-12dB panic)
0600  - Disable Normalizer
0601  - Enable Normalizer
0764  - Set volume to 100%
073C  - Set volume to 60%
0700  - Set volume to 0% (mute)
0800  - Disable DSP Bypass
0801  - Enable DSP Bypass (skip EQ, keep safety)
0900  - Disable Bass Boost
0901  - Enable Bass Boost (+8dB @ 100Hz)
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
- [x] Volume trim 0-100 adjusts audio level
- [x] Volume uses smooth transition (no clicks)
- [x] NIGHT preset caps volume at 60%
- [x] Normalizer reduces volume cap for headroom
- [x] DSP Bypass skips EQ but keeps safety stages
- [x] Bass Boost adds +8dB @ 100Hz
- [x] Bass Boost uses smooth crossfade

### GalacticStatus
- [x] Bit 0 reflects Mute state
- [x] Bit 1 reflects Audio Duck state
- [x] Bit 2 reflects Loudness state
- [x] Bit 3 reflects Normalizer state
- [x] Bit 4 reflects Bypass state
- [x] Bit 5 reflects Bass Boost state
- [x] Byte 4 reflects effective volume
- [x] Last Contact resets on notifications
- [x] Last Contact shows 0-1 during active connection

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

*Last Updated: 2026-01-28 (v2.3.0)*
