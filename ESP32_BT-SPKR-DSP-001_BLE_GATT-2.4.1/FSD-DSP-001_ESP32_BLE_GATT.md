# Functional Specification Document

## ESP32 Bluetooth Speaker - DSP Presets + Loudness via BLE GATT

## Version
**Document ID:** FSD-DSP-001
**Status:** Release (v2.1)
**Date:** 2026-01-23
**GIT Origin:** https://github.com/MYBLtd/ESP32_BT-SPKR-DSP-001_BLE_GATT.git

## Author

Robin Kluit

## Revision History

| Version | Date       | Description                                      |
|---------|------------|--------------------------------------------------|
| v1.0    | 2026-01-21 | Initial release with presets and loudness        |
| v1.1    | 2026-01-22 | Added Mute, Audio Duck, Normalizer; fixed lastContact |
| v1.2    | 2026-01-22 | Added Volume Control (device trim)               |
| v2.0    | 2026-01-23 | Document review and alignment with firmware 2.0  |
| v2.1    | 2026-01-23 | Added OTA firmware update support (BLE+WiFi)     |

## 1. Purpose and Scope

This document specifies the functional requirements, behaviour, interfaces and constraints of a Bluetooth audio receiver processing an audio stream and driving a loudspeaker.

The FSD is intended to:

- Define **what the system shall do**, not how it is implemented internally
- Serve as a reference for development, testing and future extensions
- Be understandable by engineers, educators and technically inclined users

## 2. System Overview

### 2.1 System Description
The system offers a headless user interface (no buttons/GUI on the speaker) that allows users to use their phone to:

- Select presets (e.g., Office, Full, Night, Speech)
- Toggle loudness (on/off)
- Mute/unmute audio
- Enable Audio Duck (panic button for quick volume reduction)
- Enable Normalizer (dynamic range compression)

Audio is streamed via Bluetooth Classic A2DP. Control is via BLE GATT. A dedicated companion app is not required: control is possible with generic BLE apps (nRF Connect/LightBlue) or the 42 Decibels iOS app.

### 2.2 Scope (v2.1)

**In scope:**
- DSP preset engine: fixed EQ-curves per preset
- Loudness overlay (on/off)
- Mute function with smooth fade
- Audio Duck (panic button) with ~25% volume reduction
- Normalizer (dynamic range compression)
- Volume Control (device trim) with preset-based caps
- Protection chain: high-pass + limiter
- BLE GATT control plane (write + status notify)
- Persistent storage of settings
- GalacticStatus extended status reporting
- OTA firmware updates via BLE provisioning + WiFi download

**Out of scope**
- Room calibration / microphone-measuring
- Wi-Fi SoftAP UI for general configuration

### 2.3 Design-principles

A2DP audio in -> decode -> DSP (EQ/presets/loudness/normalizer/limiter) -> I2S out -> amplifier/driver.

BLE Controls send discrete settings to DSP without interrupting audio.
Robust: no audio stuttering due to control traffic.

Simple UX: buttons/presets and toggles, no 'pro audio' terms needed.

Driver protection first: bass boost should never cause driver/amp clipping or over-excitation.

### 2.4 Assumptions and parameters

Since the exact driver/enclosure may still be unknown, this document uses practical defaults that can be easily tuned later.

Assumptions (default):
- Driver: small full range (40-70 mm)
- Enclosure: suboptimal/small volume -> limited real low end
- Sample rate: 44.1 kHz or 48 kHz (DSP must support both)
- Mono or stereo: DSP must support stereo

## 3 Functional Requirements

### 3.1 Bluetooth functionality

**FR-1**
The system shall publish its system name on Bluetooth to allow devices to connect to it.

**FR-2**
The system understands iOS Bluetooth security and is able to let an iOS device connect to it.

**FR-3**
The system receives an audio stream from the connected device.

**FR-4**
The system processes the received audio stream and outputs it using I2S.

**FR-5**
The system supports both 44.1 kHz and 48 kHz sample rates.

**FR-6**
The system uses a robust mechanism to recover itself from crashes or other malfunctioning.

### 3.2 DSP functionality

**FR-7**
Global DSP headroom (requirement):
Pre-gain: -6 dB (default) to cleanly accommodate EQ boosts + loudness + limiter.

**FR-8**
The system implements 4 presets:
- OFFICE (office/background)
- FULL (full/rich)
- NIGHT (low volume / evening)
- SPEECH (podcast/speech)

**FR-9**
User can toggle loudness: with a toggle, make low-volume playback fuller and clearer.
- Loudness is on/off
- Loudness overlay is mild and designed for low volume
- Loudness overlay must not make limiter behavior extremely "pumpy"

**FR-10** Bass/Treble discrete steps (optional v1.2)
- Bass boost level: 0..3
- Treble level: 0..2

**FR-11**
Safety: the speaker must not distort or "bottom out" due to overly aggressive bass.

**FR-12** Persistent storage
- Last selected preset and loudness state are stored in NVS/flash.
- Write frequency is limited: only save after 1-2 s without changes or on a commit event.

**FR-13** Live parameter updates
- Parameter updates must not cause audible clicks (20-50 ms ramp on gains/filters).
- DSP remains real-time; control tasks must not block audio tasks.

**FR-14**
Latency: control updates feel < 150 ms (best effort).

**FR-15**
Stability: BLE connect/disconnect must not interrupt A2DP audio.

**FR-16**
CPU budget: DSP chain within real-time budget on ESP32 (no large FFTs in v1).

**FR-17**
Memory: no dynamic allocations in the audio callback.

**FR-18**
GalacticStatus characteristic:

- Characteristic: GalacticStatus
- UUID: `00000004-1234-5678-9ABC-DEF012345678`
- Properties: READ, NOTIFY
- Length: 7 bytes

| Byte | Field                   | Value   | Description                              |
|------|-------------------------|---------|------------------------------------------|
| 0    | protocolVersion         | 0x42    | Protocol version (always 0x42)           |
| 1    | currentQuantumFlavor    | 0-3     | Current preset ID                        |
| 2    | shieldStatus            | bitfield| Status flags (see Section 10.5)          |
| 3    | energyCoreLevel         | 0-100   | Reserved (placeholder)                   |
| 4    | distortionFieldStrength | 0-100   | Volume level (placeholder)               |
| 5    | energyCore              | 0-100   | Battery level (placeholder)              |
| 6    | lastContact             | 0-255   | Seconds since last BLE communication     |

**FR-19**
Last contact tracking:

- The system tracks the timestamp of the last BLE communication.
- Communication includes: incoming reads, incoming writes, and outgoing notifications.
- The `lastContact` field in GalacticStatus reports seconds since last communication.
- Value is clamped to 255 (max uint8_t) for interactions older than 255 seconds.
- When notifications are sent every 500ms, lastContact should always be 0-1.

**FR-20**
Periodic status notifications:

- GalacticStatus notifications are pushed automatically 2 times per second (every 500ms).
- Notifications are sent only when a client is connected and has enabled notifications via CCCD.
- Timer is started on BLE connect and stopped on disconnect.

**FR-21**
Mute function:

- Command 0x04 mutes/unmutes audio output completely.
- Mute is applied with smooth gain fade (no clicks).
- Mute state is reported in shieldStatus bit 0.

**FR-22**
Audio Duck (Panic Button):

- Command 0x05 enables/disables Audio Duck mode.
- Reduces volume to approximately 25% (-12 dB) when activated.
- Applied with smooth gain transition.
- Audio Duck state is reported in shieldStatus bit 1.
- Quick toggle on/off for "panic" situations.

**FR-23**
Normalizer (Dynamic Range Compression):

- Command 0x06 enables/disables the normalizer.
- Applies dynamic range compression:
  - Threshold: -20 dB
  - Ratio: 4:1
  - Attack: 7 ms
  - Release: 150 ms
  - Makeup gain: +6 dB
- Makes quiet sounds louder and loud sounds quieter.
- Perfect for late-night viewing or noisy environments.
- Normalizer state is reported in shieldStatus bit 3.

**FR-24**
Volume Control (Device Trim):

- Command 0x07 sets the volume trim level (0-100).
- Volume is applied with smooth gain transition (no clicks).
- Actual volume may be capped based on preset (NIGHT: max 60%).
- When Normalizer is enabled, volume cap is further reduced for headroom.
- Effective volume is reported in GalacticStatus byte 4.
- Volume mapping: 100=0dB, 80=-6dB, 60=-12dB, 40=-20dB, 20=-35dB, 0=mute.
- Details for implementation are in chapter 10 of this document.

**FR-25**
OTA Firmware Updates (Hybrid BLE+WiFi):

- WiFi credentials are provisioned via BLE characteristic 0x0005.
- Firmware URL is provisioned via BLE characteristic 0x0006.
- OTA control commands are sent via BLE characteristic 0x0007.
- OTA progress is reported via BLE characteristic 0x0008 (8-byte status).
- OTA process:
  1. App writes WiFi SSID/password to OTA Credentials characteristic
  2. App writes firmware URL to OTA URL characteristic
  3. App sends START command (0x10) to OTA Control characteristic
  4. Device connects to WiFi, downloads firmware, reports progress
  5. On success, app sends REBOOT command (0x12)
  6. After reboot, app sends VALIDATE command (0x15) to prevent rollback
- Rollback support: device can revert to previous firmware if validation fails.
- WiFi is only active during OTA; disconnects after update completes.
- Details for implementation are in chapter 15 of this document.


## 4 DSP Architecture

### 4.1 Signal chain (v2.0)

```
Input -> Pre-gain (-6dB) -> HPF -> Preset EQ -> Loudness EQ -> Normalizer -> Limiter -> Volume Trim -> Audio Duck -> Mute -> Output
```

Components:
1. **Pre-gain**: -6 dB headroom
2. **High-pass filter (HPF)**: Protection against subsonic content
3. **Preset EQ**: Fixed filters per preset (4 bands)
4. **Loudness overlay EQ**: Optional bass/treble boost (2 bands)
5. **Normalizer**: Optional dynamic range compression
6. **Limiter**: Output protection against clipping
7. **Volume Trim**: Device-side volume control (0-100, capped by preset)
8. **Audio Duck**: Optional volume reduction (-12 dB)
9. **Mute**: Final output gate

### 4.2 Filter type

Recommended default (implementation freedom allowed): IIR biquad filters:
- High-pass (2nd order)
- Low-shelf / High-shelf
- Peaking (bell) EQ

### 4.3 Limiter behavior (minimum)

Recommended starting values (tunable):
- Threshold: -1.0 dBFS
- Attack: 2-5 ms
- Release: 80-200 ms
- Soft-knee: mild (or equivalent)

### 4.4 Normalizer behavior

Dynamic range compressor settings:
- Threshold: -20 dBFS
- Ratio: 4:1
- Attack: 7 ms
- Release: 150 ms
- Makeup gain: +6 dB

## 8. Preset Definition

### 8.1 Global defaults (always active)

| Component | Parameter | Value               |
|-----------|-----------|---------------------|
| Pre-gain  | Gain      | -6 dB               |
| HPF       | Type      | 2nd-order high-pass |
| HPF       | Fc        | 95 Hz (default)     |
| Limiter   | Threshold | -1.0 dBFS           |
| Limiter   | Attack    | 3 ms                |
| Limiter   | Release   | 120 ms              |

### 8.2 Preset: OFFICE (office/background)

| #   | Type       |      Fc |    Gain | Q / Slope |
|-----|------------|--------:|--------:|-----------|
| 1   | Low-shelf  |  160 Hz | +1.5 dB | S=0.7     |
| 2   | Peaking    |  320 Hz | -1.0 dB | Q=1.0     |
| 3   | Peaking    | 2800 Hz | -1.5 dB | Q=1.0     |
| 4   | High-shelf | 9000 Hz | +0.5 dB | S=0.7     |

### 8.3 Preset: FULL (full/rich)

| #   | Type       |      Fc |    Gain | Q / Slope |
|-----|------------|--------:|--------:|-----------|
| 1   | Low-shelf  |  140 Hz | +4.0 dB | S=0.8     |
| 2   | Peaking    |  420 Hz | -1.5 dB | Q=1.0     |
| 3   | Peaking    | 3200 Hz | +0.7 dB | Q=1.0     |
| 4   | High-shelf | 9500 Hz | +1.5 dB | S=0.7     |

### 8.4 Preset: NIGHT (low volume / evening)

| #   | Type       |      Fc |    Gain | Q / Slope |
|-----|------------|--------:|--------:|-----------|
| 1   | Low-shelf  |  160 Hz | +2.5 dB | S=0.8     |
| 2   | Peaking    |  350 Hz | -1.0 dB | Q=1.0     |
| 3   | Peaking    | 2500 Hz | +1.0 dB | Q=1.0     |
| 4   | High-shelf | 9000 Hz | +1.0 dB | S=0.7     |

### 8.5 Preset: SPEECH (podcast/speech)

| #   | Type      |      Fc |    Gain | Q / Slope |
|-----|-----------|--------:|--------:|-----------|
| 1   | Low-shelf |  170 Hz | -2.0 dB | S=0.8     |
| 2   | Peaking   |  300 Hz | -1.0 dB | Q=1.0     |
| 3   | Peaking   | 3200 Hz | +3.0 dB | Q=1.0     |
| 4   | Peaking   | 7500 Hz | -1.0 dB | Q=2.0     |

## 9. Loudness Overlay (v1)

### 9.1 Definition (on/off)

When Loudness is ON, an extra EQ overlay is applied on top of the preset.

| #   | Type       |      Fc |    Gain | Q / Slope |
|-----|------------|--------:|--------:|-----------|
| L1  | Low-shelf  |  140 Hz | +2.5 dB | S=0.8     |
| L2  | High-shelf | 8500 Hz | +1.0 dB | S=0.7     |

### 9.2 Constraints

- Loudness must not make "FULL + Loudness" extremely boomy; overlay is intentionally mild.
- Implementation must apply parameter smoothing when toggling.


## 10 Volume Control

### 10.0 Semantics
To distinquish between the iOS volume setting and the BT speaker actual Volume we call the BT speaker actual volume setting 'Device Trim' and the iOS volume setting 'Phone Volume' .

### 10.1
Functional description:
• User-facing volume = iPhone volume buttons / Control Center (always, even when locked).
• Device-side gain = automatic and invisible (for caps, headroom, duck, protection).

So the user experiences: “volume always works normally,” while we keep the speaker sounding safe and beautiful underwater.

What does the app do with “volume”?

We don't necessarily need to have a slider in the app. Better:
• In Detailed Status, we only show:
• System Volume: 38%
•    Profile Cap: Night max 60% (only if relevant)
•    If applicable a message, for instance “Safe headroom active”

### 10.2
Control logic:
• User controls volume with iPhone buttons
• ESP provides VOLUME trim for night cap/headroom/protection
• App can display this (and optionally adjust it)

### 10.3
control Flow:
1. App writes 0x07 xx (Write Without Response is fine for volume)
2. ESP applies smooth gain
3. Within max 500 ms, the app sees the new volume in GalacticStatus byte 4

### 10.4
The ESP handles Night / Normalizer as follows:
Profile automatically determines the cap (mostly “set & forget”)
• OFFICE/FULL/SPEECH: cap = 100
• NIGHT: cap = e.g. 60 (or whatever you like)
• Normalizer ON: ESP reserves headroom by temporarily lowering the cap (e.g. −6 dB equivalent) or via limiter.

Then the user never has to adjust the volume in the app.

### 10.5
Because “0–100” often feels strange to people, we put the mapping to dB in the protocol doc (only as an implementation guideline), e.g.:
•    100 → 0 dB
•    80 → −6 dB
•    60 → −12 dB
•    40 → −20 dB
•    20 → −35 dB
•    0 → mute-ish

### 10.6
command type:
	•	Set Volume Trim: 0x07
	•	Byte 1: 0x00–0x64 (= 0–100)

Example:
	•	0x07 0x64 → trim = 100%
	•	0x07 0x3C → trim = 60%


## 11. BLE GATT Interface

### 11.1 Goal

A generic BLE app must be able to write presets/loudness and read status.

### 11.2 Services & Characteristics

**Service UUID:** `00000001-1234-5678-9ABC-DEF012345678`

The service UUID is included in BLE advertising packets for service discovery.

**DSP Control Characteristics:**

| Characteristic   | UUID                                   | Properties               | Size    |
|------------------|----------------------------------------|--------------------------|---------|
| CONTROL_WRITE    | `00000002-1234-5678-9ABC-DEF012345678` | Write, Write No Response | 2 bytes |
| STATUS_NOTIFY    | `00000003-1234-5678-9ABC-DEF012345678` | Read, Notify             | 4 bytes |
| GALACTIC_STATUS  | `00000004-1234-5678-9ABC-DEF012345678` | Read, Notify             | 7 bytes |

**OTA Characteristics:**

| Characteristic   | UUID                                   | Properties  | Size      |
|------------------|----------------------------------------|-------------|-----------|
| OTA_CREDENTIALS  | `00000005-1234-5678-9ABC-DEF012345678` | Write       | 98 bytes  |
| OTA_URL          | `00000006-1234-5678-9ABC-DEF012345678` | Write       | 258 bytes |
| OTA_CONTROL      | `00000007-1234-5678-9ABC-DEF012345678` | Write       | 2 bytes   |
| OTA_STATUS       | `00000008-1234-5678-9ABC-DEF012345678` | Read, Notify| 8 bytes   |

### 11.3 Control Protocol (2-byte commands)

Format: **[CMD (1 byte)] [VAL (1 byte)]**

| CMD  | Name           | VAL    | Meaning                              |
|-----:|----------------|-------:|--------------------------------------|
| 0x01 | SET_PRESET     | 0..3   | 0=OFFICE, 1=FULL, 2=NIGHT, 3=SPEECH  |
| 0x02 | SET_LOUDNESS   |  0/1   | 0=OFF, 1=ON                          |
| 0x03 | GET_STATUS     |    0   | Triggers immediate notify            |
| 0x04 | SET_MUTE       |  0/1   | 0=Unmute, 1=Mute                     |
| 0x05 | SET_AUDIO_DUCK |  0/1   | 0=OFF, 1=ON (reduces volume to ~25%) |
| 0x06 | SET_NORMALIZER |  0/1   | 0=OFF, 1=ON (enables DRC)            |
| 0x07 | SET_VOLUME     | 0..100 | Volume trim (0=mute, 100=full)       |

Behavior:

- Each valid write results in a DSP update (with smoothing).
- STATUS_NOTIFY sends (notify) the current state after each change.
- GalacticStatus is updated and notified after each command.

### 11.4 Status Payload (4 bytes)

Format: **[VER][PRESET][LOUDNESS][FLAGS]**

| Byte | Field    | Value   | Description           |
|------|----------|---------|----------------------|
| 0    | VER      | 0x01    | Protocol version     |
| 1    | PRESET   | 0-3     | Current preset ID    |
| 2    | LOUDNESS | 0/1     | Loudness off/on      |
| 3    | FLAGS    | bitfield| Status flags         |

FLAGS bitfield:
- bit0: limiter active (always 1)
- bit1: clipping detected (optional)
- bit2: thermal warning (optional)
- bit3: muted
- bit4: audio duck active
- bit5: normalizer active
- bit6-7: reserved (0)

### 11.5 GalacticStatus Payload (7 bytes)

Format: **[VER][PRESET][FLAGS][ENERGY][VOLUME][BATTERY][LAST_CONTACT]**

| Byte | Field        | Value   | Description                           |
|------|--------------|---------|---------------------------------------|
| 0    | VER          | 0x42    | Protocol version (always 0x42)        |
| 1    | PRESET       | 0-3     | Current preset (QuantumFlavor)        |
| 2    | FLAGS        | bitfield| Shield status flags                   |
| 3    | ENERGY       | 0-100   | Energy core level (reserved)          |
| 4    | VOLUME       | 0-100   | Effective volume level (FR-24)        |
| 5    | BATTERY      | 0-100   | Energy core/battery (reserved)        |
| 6    | LAST_CONTACT | 0-255   | Seconds since last BLE communication  |

**Shield Status FLAGS (byte 2) bitfield:**

| Bit | Mask | Field      | Description                         |
|-----|------|------------|-------------------------------------|
| 0   | 0x01 | Muted      | Audio is muted                      |
| 1   | 0x02 | Audio Duck | Volume reduced (panic mode)         |
| 2   | 0x04 | Loudness   | Loudness compensation enabled       |
| 3   | 0x08 | Normalizer | Dynamic range compression enabled   |
| 4-7 | -    | Reserved   | Future use                          |

This characteristic is automatically notified every 500ms (FR-20) when notifications are enabled.

### 11.6 BLE Advertising Configuration

| Parameter        | Value           | Description                        |
|------------------|-----------------|------------------------------------|
| Device Name      | "42 Decibels"   | Advertised BLE device name         |
| Adv Interval Min | 20ms (0x20)     | Minimum advertising interval       |
| Adv Interval Max | 40ms (0x40)     | Maximum advertising interval       |
| Appearance       | 0x0841          | BLE Speaker appearance             |
| Service UUID     | Included        | Service UUID in advertising packet |

The advertising packet contains the service UUID for filtering/discovery. The scan response contains the device name and TX power.

### 11.7 Connection Parameters

On connection, the following parameters are requested to optimize latency (FR-14):

| Parameter    | Value   | Description                       |
|--------------|---------|-----------------------------------|
| Min Interval | 20ms    | Minimum connection interval       |
| Max Interval | 40ms    | Maximum connection interval       |
| Latency      | 0       | Slave latency (no skipped events) |
| Timeout      | 4000ms  | Supervision timeout               |

## 12. State Machine and Error Handling

### 12.1 BLE connect/disconnect

- On BLE connect:
  - Connection parameters are updated for low latency (FR-14)
  - Last contact timestamp is initialized
  - GalacticStatus notification timer is started (FR-20)
  - Status values are updated and ready for read
- On BLE disconnect:
  - Audio continues playing; settings remain active (FR-15)
  - Notifications are disabled
  - GalacticStatus notification timer is stopped
  - Advertising is automatically restarted
- On reboot: settings are loaded from NVS; status notify on first connect (read/notify).

### 12.2 Unknown command / value

- Unknown CMD: ignore; send status back unchanged.
- VAL out of range: v1 recommendation: ignore and notify status.

## 13. Persistency

### 13.1 Fields to store

- preset_id (uint8)
- loudness (uint8)
- (optional) bass_level, treble_level
- config_version (uint8)

### 13.2 Write policy

- Do not flash on every BLE write.
- Debounce: write after **1-2 seconds** without new changes.

## 14. Performance and Real-time Requirements

### 14.1 Audio task priority

- Audio pipeline tasks have highest priority.
- BLE handling must never run in the audio callback.

### 14.2 Parameter smoothing

- On preset change: crossfade/parameter ramp (**20-50 ms**) to prevent clicks.
- Filter coefficient updates: switch in one go with output ramp, or interpolate per block.
- Mute, Audio Duck, and Normalizer use smooth gain transitions.

## 15. OTA Firmware Updates

### 15.1 Architecture

The OTA system uses a hybrid BLE + WiFi approach:
- **BLE**: Used for provisioning (credentials, URL) and control commands
- **WiFi**: Used only for firmware download (temporary connection)

This approach allows OTA updates without requiring a permanent WiFi configuration UI.

### 15.2 OTA Characteristics

#### OTA Credentials (0x0005)
- **Format**: `SSID\0PASSWORD` (null-separated)
- **Max Size**: 98 bytes (32 SSID + 1 separator + 64 password + 1 padding)
- Credentials are stored in memory only (not persisted)

#### OTA URL (0x0006)
- **Format**: URL string (null-terminated or length-prefixed)
- **Max Size**: 258 bytes (2 length + 256 URL)
- Supports HTTP and HTTPS URLs

#### OTA Control (0x0007)
- **Format**: `[CMD][PARAM]` (2 bytes)
- Commands:
  - `0x10`: START - Begin OTA process
  - `0x11`: CANCEL - Abort OTA operation
  - `0x12`: REBOOT - Reboot to new firmware
  - `0x13`: GET_VERSION - Query firmware version
  - `0x14`: ROLLBACK - Revert to previous firmware
  - `0x15`: VALIDATE - Mark firmware as valid

#### OTA Status (0x0008)
- **Format**: 8-byte status packet (see Protocol.md)
- Notifications sent automatically during OTA operations
- Contains: state, error code, progress %, downloaded/total KB, WiFi RSSI

### 15.3 OTA State Machine

```
IDLE ──(credentials)──> CREDS_RECEIVED
  │                            │
  │                    (url)───┘
  │                            │
  │                            v
  │                     URL_RECEIVED
  │                            │
  │                    (start)─┘
  │                            │
  │                            v
  │                  WIFI_CONNECTING
  │                            │
  │                    (connected)
  │                            │
  │                            v
  │                   WIFI_CONNECTED
  │                            │
  │                    (download)
  │                            │
  │                            v
  │                    DOWNLOADING ──(progress)──> DOWNLOADING
  │                            │
  │                    (complete)
  │                            │
  │                            v
  │                     VERIFYING
  │                            │
  │                    (verified)
  │                            │
  │                            v
  │                      SUCCESS
  │                            │
  │                    (reboot)─┘
  │                            │
  │                            v
  │                   [Device Reboots]
  │                            │
  │                            v
  │                  PENDING_VERIFY
  │                            │
  │                   (validate)
  │                            │
  │                            v
  └─────────────────────────> IDLE

Any State ──(error)──> ERROR
Any State ──(cancel)──> IDLE
```

### 15.4 Error Handling

| Error Code | Description | Recovery |
|------------|-------------|----------|
| 0x01 | WiFi connection failed | Check credentials, retry |
| 0x02 | HTTP connection failed | Check URL, network |
| 0x03 | HTTP error response | Check server/URL |
| 0x04 | Download failed | Check network, retry |
| 0x05 | Verification failed | Re-download |
| 0x06 | Flash write failed | Critical - check storage |
| 0x07 | No credentials | Write credentials first |
| 0x08 | No URL | Write URL first |
| 0x09 | Invalid firmware | Check firmware file |
| 0x0A | Cancelled | User cancelled |
| 0x0B | Rollback failed | Critical error |

### 15.5 Partition Table

OTA requires a dual-partition scheme (`partitions_ota.csv`):

| Name | Type | SubType | Offset | Size |
|------|------|---------|--------|------|
| nvs | data | nvs | 0x9000 | 0x6000 |
| phy_init | data | phy | 0xf000 | 0x1000 |
| factory | app | factory | 0x10000 | 1M |
| ota_0 | app | ota_0 | 0x110000 | 1M |
| ota_1 | app | ota_1 | 0x210000 | 1M |
| ota_data | data | ota | 0x310000 | 0x2000 |

### 15.6 Rollback Protection

- After booting new firmware, the device enters `PENDING_VERIFY` state
- App must send VALIDATE command (0x15) to mark firmware as valid
- If not validated within timeout, device may auto-rollback on next boot
- Manual rollback available via ROLLBACK command (0x14)

### 15.7 WiFi Behavior

- WiFi is initialized only when OTA START command is received
- WiFi disconnects automatically after OTA completes (success or error)
- WiFi does not interfere with A2DP audio during normal operation
- WiFi RSSI is reported in OTA Status for signal quality monitoring
