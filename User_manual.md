# Bluetooth Audio Quality (BM83)

This speaker uses a Microchip BM83 Bluetooth audio module for Bluetooth Classic (A2DP) music streaming. The BM83 decodes the incoming Bluetooth audio stream and outputs digital audio internally to the rest of the speaker.

## Supported Bluetooth Audio Codecs

- **AAC** (A2DP AAC)
- **SBC** (A2DP SBC - mandatory baseline codec for all A2DP devices)

In practice, iOS/macOS devices commonly use AAC when the receiver supports it, and fall back to SBC if needed. Android devices typically prefer SBC unless a higher-quality codec is available.

## Maximum Technical Audio Formats

Assuming everything after the BM83 is ideal, the relevant operating modes for SBC/AAC music streaming over A2DP are:

- **Sample rate:** 44.1 kHz or 48 kHz
- **Channels:** Stereo
- **Compression:** Lossy (AAC or SBC)

Internally, the BM83 can output digital audio over I2S in:

- **Bit depth:** 16-bit or 24-bit
- **Sample rates:** 8 / 16 / 44.1 / 48 / 88.2 / 96 kHz (capability depends on the use case and audio path)

**Important:** Even if a device supports higher internal sample rates, Bluetooth A2DP with AAC/SBC is not a "hi-res lossless" link. For best compatibility, expect 44.1/48 kHz lossy audio over Bluetooth.
