# Functional Specification Document (FSD)

## BM83SM1-00AB + STM32H753 + ESP32 — Bluetooth Speaker (DSP + BLE GATT Control)

## Version
- **Document ID:** FSD-DSP-002_H753_ESP32_BLE_GATT_v2.md
- **Status:** Develop
- **Date:** 2026-02-03
- **Applies to:** ESP32 firmware, STM32H753 DSP firmware, iOS + watchOS apps, BM83 configuration/host control

## Author
Robin Kluit

## Revision History
| Version | Date       | Description                                                  |
| ------: | ---------- | ------------------------------------------------------------ |
|    v1.0 | 2026-01-21 | Initial BLE GATT + DSP preset definition                     |
|    v2.0 | 2026-02-03 | V2: STM32H753 DSP engine + clarified responsibilities, fixed FR numbering |
|    v2.1 | 2026-02-03 | V2: Added BM83SM1-00AB as Bluetooth Classic audio module and clarified audio/control planes |

---

## 1. Purpose and Scope

This document defines the functional behavior and interfaces for a Bluetooth speaker system consisting of:
- **BM83SM1-00AB (Microchip)**: Bluetooth Classic audio endpoint (A2DP sink, codec decode) and PCM audio output
- **STM32H753**: Real-time DSP engine (presets, loudness, normalizer, limiter, ramps)
- **ESP32**: BLE GATT control endpoint, OTA orchestration, persistency, and bridging between BLE and DSP engine
- **iOS + watchOS**: BLE GATT control and status UI

The intent is that this document is directly usable for:
1) ESP32 firmware development (BLE GATT + OTA + bridging + persistency),
2) STM32H753 DSP implementation (audio pipeline + real-time constraints),
3) iPhone/watchOS app development (GATT protocol + payload decoding),
4) BM83 integration (audio roles + host control expectations).

---

## 2. System Overview

### 2.1 High-Level Architecture (V2.1)

**Audio Plane (Bluetooth Classic → PCM → DSP → Output)**
1) Phone / tablet streams audio using Bluetooth Classic **A2DP**
2) **BM83** receives A2DP, decodes codec to PCM, outputs PCM over **I2S**
3) **STM32H753** receives PCM (I2S in), applies DSP chain, outputs processed PCM (I2S out)
4) DAC / amplifier renders audio

**Control Plane (BLE GATT → Commands → DSP)**
1) iOS/watchOS app connects to **ESP32 BLE GATT**
2) App writes 2-byte control commands `[CMD][VAL]`
3) ESP32 validates and forwards commands to **STM32H753**
4) H753 updates DSP state and returns status payloads to ESP32
5) ESP32 notifies BLE clients with fixed-size status payloads

**Key principle:** Bluetooth Classic streaming is handled by **BM83**; BLE control is handled by **ESP32**. The DSP engine is handled by **STM32H753**.

---

### 2.2 Scope

**This specification covers:**

 - Bluetooth audio input via BM83 (A2DP AAC) and output via I2S PCM
 - DSP processing on STM32H753
 - BLE-based control (ESP32 acting as a control/OTA coprocessor)
 - Status reporting via BLE GATT notifications
 - A reference DAC/AMP chain for bring-up and testing

 **Non-normative note (product deployment variants):**
 - This document is written to support multiple product deployments that share the same firmware feature set.
 - In particular, **P2b (wired stereo via a single external electronics box driving two passive speakers over speaker cables)** is considered *in scope* as a deployment variant of the same DSP/control architecture.
 - Conversely, **P3 (true wireless stereo / grouping via BM83 MSPK2 “Concert/Stereo” modes)** is treated as **out of scope** for this document and will be specified and validated separately if/when required.

### 2.3 Component Responsibilities (Implementation Contract)

#### BM83SM1-00AB Responsibilities (Bluetooth Classic Audio Endpoint)
- Operates as the Bluetooth Classic **A2DP sink** (primary use case)
- Decodes received audio (codec) to PCM
- Outputs PCM audio via **I2S** to STM32H753
- Link management (pair/connect/disconnect) as per BM83 firmware capabilities
- Optional: AVRCP sink behavior (play/pause/next/prev) if required by product UX
- BM83 is not required to expose the speaker control protocol to apps (apps use ESP32 BLE GATT)

#### STM32H753 Responsibilities (Audio + DSP Engine)
- Receives PCM from BM83 over I2S
- Applies real-time DSP chain (presets, loudness, normalizer, limiter, ramps)
- Outputs processed PCM over I2S to DAC/amplifier
- Provides status values consumed by ESP32 for BLE payloads
- Must meet deterministic real-time constraints (see §14)

#### ESP32 Responsibilities (BLE Control + OTA + Persistency)
- BLE GATT server (services/characteristics as defined in §11)
- Validates GATT writes and clamps/ignores invalid values
- Bridges commands to STM32H753 and publishes status back to BLE
- OTA firmware update orchestration (ESP32 image, plus optional H753 image where applicable)
- Stores user settings persistently (ESP32 NVS; see §13)
- Must not disrupt audio continuity (even during BLE/OTA activity)

### 2.3 Design-principles
- audio chain: BM83SM1-00AB → STM32 (DSP) → PCM5102 (I2S DAC) → TPA3116D2
- control chain: ESP32 → STM32 (DSP)
- BLE Controls send discrete settings to DSP without interrupting audio.
- Robust: no audio stuttering due to control traffic.
- Driver protection first: bass boost should never cause driver/amp clipping or over-excitation.

### 2.4 Naming and Advertising
- BLE device name: `42dB <MODEL>-<SUFFIX>` (see FR-1)
- Bluetooth Classic (BM83) “friendly name” may match or slightly shorten BLE name, but should remain recognizable and unique within a household.

---

## 3. Functional Requirements (FR)

### 3.1 Identity / Discoverability

**FR-1 — BLE Device Name**
The system shall advertise a BLE device name in the form:
`42dB <MODEL>-<SUFFIX>`  
Example: `42dB FREDD-A3C7`

**FR-2 — iOS/watchOS BLE Compatibility**
The system shall support iOS/watchOS BLE connection and control via the GATT protocol defined in §11.

---

### 3.2 Bluetooth Classic Audio (BM83)

**FR-3 — Bluetooth Classic A2DP Sink (BM83)**
The system shall receive audio streams using Bluetooth Classic A2DP on the BM83 module and produce PCM audio for downstream DSP processing.

**FR-4 — PCM Output from BM83**
BM83 shall output stereo PCM audio over I2S to the STM32H753.

**FR-5 — Audio Output**
The system shall output processed audio using I2S from STM32H753 to the DAC/amplifier path.

**FR-6 — Sample Rate Support**
The DSP engine shall support 44.1 kHz and 48 kHz sample rates (at minimum).  
(If BM83 is configured for other sample rates, firmware shall either support them or force a supported rate.)

---

### 3.3 DSP Features (User-Facing)

**FR-7 — Global DSP Headroom**
The DSP engine shall apply a global pre-gain default of **-6 dB** (configurable constant) to provide headroom.

**FR-8 — DSP Presets**
The system shall support at least four presets selectable via BLE:
- 0 = OFFICE
- 1 = FULL
- 2 = NIGHT
- 3 = SPEECH  
Preset intent is specified in §8.

**FR-9 — Loudness Overlay**
The system shall provide a loudness overlay (on/off) as defined in §9.

**FR-10 — Mute**
The system shall support mute on/off via BLE with smooth gain ramps (no clicks).

**FR-11 — Audio Duck**
The system shall support an Audio Duck mode on/off via BLE:
- When enabled, output reduces to ~25% (~-12 dB) with smooth ramps.

**FR-12 — Normalizer (DRC)**
The system shall support a Normalizer mode on/off via BLE (DRC behavior defined in §4.4).

**FR-13 — Bass / Excursion Safety**
The system shall avoid distortion and mechanical “bottoming out” due to aggressive low-frequency boost.

**FR-14 — Device Trim (Device-side Volume)**
The system shall support a “Device Trim” control (0–100) via BLE that applies device-side gain in the DSP chain.

---

### 3.4 Control Responsiveness + Stability

**FR-15 — Live Parameter Updates (No Clicks)**
Preset/loudness/mute/duck/normalizer/trim changes shall not produce audible clicks/pops. All transitions must be smoothed (recommended 20–50 ms ramps).

**FR-16 — Control Latency**
Control updates shall “feel” responsive with best-effort apply + status reflect within **<150 ms** under normal conditions.

**FR-17 — Audio Continuity**
BLE activity (connect/disconnect/control writes/notifications) and OTA activity on ESP32 shall not interrupt BM83 A2DP audio playback.

**FR-18 — DSP Real-Time Budget**
STM32H753 DSP shall run within the real-time budget with margin (no audio overruns/underruns).

---

### 3.5 Real-Time Memory / Scheduling Constraints

**FR-19 — No Dynamic Allocation in Audio Path**
No dynamic memory allocation/free operations shall occur in any real-time audio processing path (ISR/DMA callback/block process) on STM32H753, nor in any ESP32 code paths that run in an audio callback context.

**FR-20 — No Blocking in Audio Path**
Real-time audio processing on STM32H753 shall not block, wait on locks, or call APIs with unbounded execution time.

---

### 3.6 Status + Telemetry

**FR-21 — Status Payloads**
The system shall provide:
- `STATUS_NOTIFY` (4 bytes)
- `GALACTIC_STATUS` (7 bytes)  
Formats defined in §11.4 and §11.5.

**FR-22 — Last Contact**
The system shall track seconds since last BLE communication and expose it as `LAST_CONTACT` in GalacticStatus.

**FR-23 — Periodic Notify**
When CCCD notifications are enabled, the system shall send GalacticStatus notifications at **2 Hz (every 500 ms)**.

---

### 3.7 OTA

**FR-24 — OTA Firmware Updates**
The system shall support OTA updates for ESP32 firmware via the BLE+WiFi hybrid approach defined in §15.
- Updating BM83 firmware is out-of-scope for baseline product requirements unless explicitly enabled as a separate maintenance procedure.

---

## 4. DSP Architecture (STM32H753)

### 4.1 Conceptual Signal Chain
1) PCM input (from BM83)
2) Global pre-gain (FR-7)
3) Preset EQ
4) Loudness overlay (optional)
5) Normalizer/DRC (optional)
6) Device Trim + Duck + Mute (gain stage with ramps)
7) Limiter (safety net)
8) PCM output

### 4.2 Filter Type
- BiQuad filters recommended for EQ stages (low shelf / peaking / high shelf)
- Coefficients may be per-sample-rate

### 4.3 Limiter (Minimum)
- Prevent clipping/distortion
- Target threshold example: -0.5 dBFS (implementation-tunable)
- Attack/release tuned to avoid pumping

### 4.4 Normalizer (DRC) Baseline (Tunable)
- Threshold: -20 dB
- Ratio: 4:1
- Attack: 7 ms
- Release: 150 ms
- Makeup gain: +6 dB (must respect headroom)

---

## 5. Persistency (ESP32 NVS)

### 5.1 Fields to Store
- `preset_id` (uint8)
- `loudness` (uint8)
- `mute` (uint8)
- `duck` (uint8)
- `normalizer` (uint8)
- `volume_trim` (uint8)
- `config_version` (uint8)

### 5.2 Write Policy
- Debounce writes (e.g., commit after 1–2 s of inactivity) to avoid flash wear.

---

## 6. Inter-Processor Interfaces

### 6.1 ESP32 ↔ STM32H753 Bridge (Control + Status)

**6.1.1 Command Forwarding**
- ESP32 shall forward each valid BLE control write to H753 as the same 2 bytes: `[CMD][VAL]`.

**6.1.2 Status Return**
- H753 shall produce the same payload formats as BLE expects:
  - StatusNotify (4 bytes)
  - GalacticStatus (7 bytes)
- ESP32 shall publish these payloads to BLE clients without changing field semantics.

**6.1.3 Timing**
- After receiving a command, H753 applies smoothing and updates status.
- ESP32 notifies clients as soon as practical after receiving updated status.

---

### 6.2 ESP32 ↔ BM83 Host Control (Optional but Recommended)

> This is optional for baseline audio playback (BM83 can be largely standalone), but recommended for a complete product UX (pairing, re-pairing, connection management, optional AVRCP).

**FR-BM83-1 — BM83 Host Interface Availability**
The design shall expose a host interface between ESP32 and BM83 (UART recommended) to allow at minimum:
- querying connection state (connected/disconnected),
- initiating pairing / clearing pairing (if required by UX),
- optional media control bridging (AVRCP key events) if physical buttons exist.

**FR-BM83-2 — Separation of Roles**
ESP32 BLE control protocol shall remain independent from BM83 audio stack behavior. Apps must not be required to talk to BM83 directly.

---

## 7. Acceptance Tests (Cross-Platform)

### 7.1 Audio Streaming (BM83)
- Start A2DP stream to BM83; verify stable PCM to H753 and audio output.
- Stress BLE connect/disconnect while streaming; verify no stutter/stop (FR-17).

### 7.2 Control Responsiveness
- Set preset; verify audible change without clicks and status updates quickly (FR-15/16).
- Toggle loudness/normalizer/duck/mute; verify smooth ramps and correct flags.

### 7.3 Periodic Notify
- Enable notifications; verify 2 Hz GalacticStatus and LAST_CONTACT behavior.

---

## 8. Presets (Intent)

### 8.1 Global Defaults
- Global pre-gain: -6 dB
- Limiter: enabled

### 8.2 OFFICE
- Clear, balanced, non-fatiguing for background listening.

### 8.3 FULL
- Rich, extended bass/treble, “fun” curve within safety constraints.

### 8.4 NIGHT
- Low-volume friendly, reduced harshness, safer bass/excursion.

### 8.5 SPEECH
- Speech intelligibility priority, reduced boominess, controlled sibilance.

> Note: exact biquad coefficients are implementation data and may live in firmware tables; control IDs and audible intent must remain stable.

---

## 9. Loudness Overlay

- On/off tonal compensation optimized for low listening levels.
- Must not destabilize limiter or create pumping.
- Must ramp smoothly when toggled.

---

## 10. Volume Model (Device Trim)

### 10.1 Definition
- Phone volume remains normal (system UI).
- “Device Trim” is device-side gain applied in DSP.

### 10.2 Mapping (Guideline)
Suggested perceptual curve (non-normative):
- 100 → 0 dB
- 80 → −6 dB
- 60 → −12 dB
- 40 → −20 dB
- 20 → −35 dB
- 0 → mute-ish

---

## 11. BLE GATT Interface (ESP32)

### 11.1 Service UUID
`00000001-1234-5678-9ABC-DEF012345678`

### 11.2 Characteristics
| Characteristic  | UUID                                   | Properties               | Size    |
| --------------- | -------------------------------------- | ------------------------ | ------- |
| CONTROL_WRITE   | `00000002-1234-5678-9ABC-DEF012345678` | Write, Write No Response | 2 bytes |
| STATUS_NOTIFY   | `00000003-1234-5678-9ABC-DEF012345678` | Read, Notify             | 4 bytes |
| GALACTIC_STATUS | `00000004-1234-5678-9ABC-DEF012345678` | Read, Notify             | 7 bytes |

(OTA characteristics unchanged; see §15.)

### 11.3 Control Commands (2 bytes)
Format: `[CMD][VAL]`

|  CMD | Name           |    VAL | Meaning                             |
| ---: | -------------- | -----: | ----------------------------------- |
| 0x01 | SET_PRESET     |   0..3 | 0=OFFICE, 1=FULL, 2=NIGHT, 3=SPEECH |
| 0x02 | SET_LOUDNESS   |    0/1 | 0=OFF, 1=ON                         |
| 0x03 | GET_STATUS     |      0 | Triggers immediate notify           |
| 0x04 | SET_MUTE       |    0/1 | 0=Unmute, 1=Mute                    |
| 0x05 | SET_AUDIO_DUCK |    0/1 | 0=OFF, 1=ON                         |
| 0x06 | SET_NORMALIZER |    0/1 | 0=OFF, 1=ON                         |
| 0x07 | SET_VOLUME     | 0..100 | Device Trim                         |

### 11.4 StatusNotify Payload (4 bytes)
`[VER][PRESET][LOUDNESS][FLAGS]`
- VER = 0x01

FLAGS bitfield (recommended):
- bit3: muted
- bit4: audio duck active
- bit5: normalizer active
(other bits reserved/optional)

### 11.5 GalacticStatus Payload (7 bytes)
`[VER][PRESET][FLAGS][ENERGY][VOLUME][BATTERY][LAST_CONTACT]`
- VER = 0x42

FLAGS (byte 2):
- bit0: muted
- bit1: audio duck active
- bit2: loudness enabled
- bit3: normalizer enabled

Periodic notify:
- every 500 ms when CCCD enabled.

---

## 12. Error Handling

### 12.1 Unknown CMD / Out-of-Range VAL
- Unknown CMD: ignore safely (no crash)
- Invalid VAL: clamp or ignore (choose one behavior and keep consistent)

### 12.2 Robustness
- Must recover from crash (watchdog + safe boot)
- Must not corrupt persistency on unexpected power loss (best effort)

---

## 13. Real-Time Requirements (STM32H753)

- No malloc/free in audio path (FR-19)
- No blocking/locks in audio path (FR-20)
- Audio processing must preempt control plane tasks

---

## 14. OTA (ESP32)

Hybrid BLE + WiFi OTA:
- OTA characteristics remain as previously defined
- ESP32 handles download + flash + reboot strategy
- H753 update mechanism (if any) is implementation-specific and must not disrupt BM83 audio unless user explicitly initiates update.

---

## 15. Out of Scope (for this baseline FSD)
- BM83 firmware update procedure (unless you choose to productize it)
- Multi-speaker / concert / stereo-link features of BM83 (unless explicitly enabled later)
- Battery measurement and “energy” telemetry (fields reserved in GalacticStatus)

---