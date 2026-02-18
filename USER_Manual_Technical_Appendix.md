# Bluetooth Audio Quality

This speaker uses a Bluetooth audio module for Bluetooth Classic (A2DP) music streaming. In simple terms: your phone sends a Bluetooth audio stream, the Bluetooth module decodes it, and the rest of the speaker takes it from there.

## Supported Bluetooth Codecs

To balance sound quality, compatibility, and stability, the BM83 supports:

- **AAC** (preferred by most iOS / iPadOS / macOS devices when available)
- **SBC** (the universal Bluetooth audio codec - always available as a fallback)

## Maximum Streaming Format (What Bluetooth Can Realistically Deliver)

Assuming ideal conditions and ideal downstream audio hardware, Bluetooth A2DP with AAC/SBC is typically:

- Stereo
- 44.1 kHz or 48 kHz
- Lossy compression (AAC or SBC)

**Important note:** Bluetooth A2DP (AAC/SBC) is designed for high-quality wireless playback, not "hi-res lossless" transport. Even with premium electronics inside the speaker, Bluetooth itself remains the limiting link.

---

## Getting the Best Sound (Quick Tips)

For the most consistent, premium listening experience:

- Keep your phone/tablet close to the speaker (1–3 m), ideally with a clear path (2.4 GHz radio is easily blocked by walls and even the human body).
- If you notice a sudden drop in clarity or stability, move closer first - it fixes most issues instantly.
- Use the highest streaming quality in your music app, or lossless sources when available.
- If quality seems "stuck" worse than normal: toggle Bluetooth off/on or re-pair the speaker.

---

## Technical Appendix (For Curious Users)

### 1) "Max Spec" vs "Real-World Best"

Codec names don't tell the whole story. In practice, two factors dominate real-world sound quality:

- **RF conditions** (Bluetooth radio link quality)
- **Double encoding** (your audio may be compressed more than once)

### 2) AAC vs SBC - What to Expect

**AAC**

- Most Apple devices select AAC when supported.
- Typically delivers a smooth, refined top end and strong perceived detail at everyday Bluetooth bitrates.
- When constrained, you may hear slightly "glassy" ambience or softened transients.

**SBC**

- Guaranteed compatibility across virtually all devices.
- Often very good in "high quality" modes, especially with a clean RF link.
- When constrained, you may hear more "watery" high frequencies (cymbals/hi-hats), grain in sibilance ("S" sounds), or a slightly flatter sense of space.

**Premium takeaway:**
With iOS/macOS, you'll usually get AAC, which tends to sound more effortless and polished. SBC remains an excellent fallback when conditions or platform choices demand it.

### 3) RF / Connection Quality - Often the #1 Factor

Bluetooth operates in the busy 2.4 GHz band (shared with Wi-Fi and many household devices). When the radio link is stressed, devices may reduce bitrate or lean on error correction - you might hear:

- Small dropouts or "blips"
- A softer, less defined high end
- Occasional instability in spatial detail

**Best practices:**

- Keep the source device closer.
- Avoid placing your phone behind your body relative to the speaker.
- If you're in a crowded RF environment (busy cafes, apartments with dense Wi-Fi), reducing distance helps more than anything else.

### 4) Double Encoding - Why Your Source Quality Still Matters

Many music services already use lossy compression. Bluetooth then re-encodes that audio to AAC/SBC for transmission:

```
compressed source → decoded by phone → re-encoded for Bluetooth → decoded by speaker
```

This can slightly reduce clarity, especially in:

- Cymbals and fine high-frequency detail
- Reverb tails and room ambience
- Dense mixes with lots of layers

**Best practices:**

- Use the highest quality streaming settings, or lossless sources where possible.
- Avoid stacking multiple "enhancements" (heavy EQ + loudness normalization + other DSP), which can amplify artifacts after Bluetooth re-encoding.

### 5) Troubleshooting Flow (Fast and Effective)

If audio quality is worse than usual:

1. Move closer (1–3 m), clear line-of-sight if possible
2. Stop and restart playback
3. Toggle Bluetooth off/on
4. Forget device + re-pair

---

# DSP Audio Processing

Beyond Bluetooth decoding, this speaker includes a real-time DSP (Digital Signal Processing) engine running on a 32-bit microcontroller. The DSP sits after the Bluetooth receiver and before the DAC/amplifier stage, shaping the sound in the digital domain with precision and repeatability.

## What the DSP Does (And Why It Matters)

The DSP is designed to make a compact speaker sound bigger, clearer, and more consistent across different rooms and listening levels - while protecting the system from harsh peaks and unwanted level jumps.

Key processing blocks include:

- **EQ & Presets** - voicing for different use cases
- **Loudness enhancement** - bass support at lower volumes
- **Bass Boost mode** - extra low-end emphasis for small speakers
- **Dynamic Range Control / Normalizer** - keeps levels consistent, reduces sudden peaks
- **Output limiting & headroom management** - prevents clipping when toggling effects or switching presets

## Presets

The DSP includes curated presets for different environments and listening habits:

- **OFFICE** - balanced for background listening
- **FULL** - wide-range, full-body sound
- **NIGHT** - volume capped for late-night use
- **SPEECH** - optimized for voice clarity (podcasts, calls)

These are intentionally tuned to be "set-and-forget": clear differences, no gimmicks.

## Safety, Headroom, and "No Surprises"

DSP can improve perceived loudness and bass - but it can also create sudden level jumps if not managed carefully. To prevent that, the engine applies fixed headroom (-9 dB) and keeps behavior predictable when enabling/disabling features.

## Instant Control Features

For real-world usability (and sanity when the room gets loud), the DSP includes:

- **Duck Mode** - an instant -12 dB reduction ("panic button")
- **Mute** - immediate silence
- **DSP Bypass** - direct passthrough (no EQ/loudness/compression shaping)

**Note:** "Bypass" is intended for reference listening and troubleshooting. In normal use, the tuned presets are where the speaker sounds its best.

## Control via App (BLE)

DSP settings can be controlled wirelessly via BLE GATT. In the reference architecture, BLE commands are received by an ESP32 and forwarded to the DSP engine over UART.

In the planned BM83-based architecture, the BM83 handles Bluetooth A2DP audio, while BLE/app control remains on a companion controller for a flexible, updatable control interface.

---

## Practical Notes (Honest Expectations)

### DSP can't "undo" Bluetooth compression

Bluetooth AAC/SBC is still a lossy link. The DSP can improve tonal balance, clarity, and perceived punch - but it cannot restore detail that isn't present in the source. (This matters most with low-bitrate streams and double-encoded sources.)

### DSP is tuned for "premium everyday listening"

The goal isn't to sound exaggerated - it's to sound effortless:

- Fuller at low volume
- Controlled at high volume
- Consistent across different content

### If you want the most "studio neutral" playback

Use DSP Bypass as a reference mode. Otherwise, select the preset that best matches your room and listening level.
