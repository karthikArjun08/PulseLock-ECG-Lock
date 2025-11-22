# PulseLock (ESP32 + AD8232) — Human-detection Biometric Lock (Demo)

**Short:** a lightweight PulseLock demo that unlocks when the AD8232 detects a real human ECG waveform (peaks + variance). Also supports manual `unlock` via Serial.

## Files
- `PulseLock.ino` — main Arduino sketch (ESP32). Upload to your ESP32 in Arduino IDE.
- `README.md` — this file.
- Project doc (local): `file:///mnt/data/Skip to content.pdf`

## Wiring (minimum)
- AD8232 OUTPUT -> ESP32 **GPIO34**
- AD8232 3.3V   -> ESP32 **3V3**
- AD8232 GND    -> ESP32 **GND**
- Relay IN      -> ESP32 **GPIO5**
- Relay VCC     -> **5V** (relay module)
- Relay GND     -> **GND**
- Optional LEDs:
  - Green LED -> GPIO2 (through 220Ω resistor to GND)
  - Red LED -> GPIO4 (through 220Ω resistor to GND)
- Solenoid lock: 12V supply -> Relay COM, Relay NO -> Solenoid +, Solenoid - -> 12V -

## How to use
1. Open Arduino IDE, select your ESP32 board and COM port.
2. Paste `PulseLock.ino` into a sketch and upload.
3. Open **Serial Monitor** at **115200** baud.
4. Attach electrodes and ensure stable ECG waveform in Serial Plotter (optional).
5. For demo:
   - Automatic: the device will unlock when it detects consistent ECG activity.
   - Manual: type `unlock` in Serial Monitor to force the relay.

## Commands (via Serial Monitor)
- `unlock` — manual unlock (for demo/testing)
- `status` — prints current detection stats (stddev, peaks)
- `tune` — prints current tuning values (for debugging)

## Tuning
If the lock triggers on noise or not on real ECG, tune constants near the top of `PulseLock.ino`:
- `MIN_STDDEV` (increase to avoid noise / decrease to be more sensitive)
- `MIN_PEAKS_WINDOW` (increase to require more beats)
- `PEAK_DELTA` (amplitude threshold for identifying peaks)

Use `status` to observe `stddev` and peak count and pick thresholds that work for typical users.

## Notes & Safety
- Do NOT connect electrodes to mains or any AC source.
- Use 3.3V for AD8232.
- Solenoid/lock must be powered by separate 12V supply and fused correctly.
- This demo uses a simple detector — not intended as a secure production biometric system.

## Local project document
Your uploaded project PDF is available locally at:
`file:///mnt/data/Skip to content.pdf`
(You can add it to the repo or use it as reference.)

## License
MIT
