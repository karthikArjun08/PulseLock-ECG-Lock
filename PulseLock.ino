/********************************************************************************
 * PulseLock.ino
 *
 * ECG-based human-presence lock + serial command unlock (ESP32)
 *
 * - Auto-unlocks when AD8232 shows real human ECG activity (peaks + variance).
 * - Does NOT unlock when electrodes are not attached / flat signal.
 * - Accepts manual "unlock" command via Serial Monitor for demo.
 *
 * Wiring (summary):
 *  - AD8232 OUTPUT -> GPIO34 (ADC1)
 *  - AD8232 3.3V   -> 3V3
 *  - AD8232 GND    -> GND
 *  - Relay IN      -> GPIO5
 *  - Relay VCC     -> 5V (relay module)
 *  - Relay GND     -> GND (common ground)
 *  - Green LED     -> GPIO2 (optional)
 *  - Red LED       -> GPIO4 (optional)
 *
 * Project doc (local): file:///mnt/data/Skip to content.pdf
 *
 * License: MIT
 ********************************************************************************/

#include <Arduino.h>

// ----- Pins -----
const int ECG_PIN   = 34;   // AD8232 output (ADC1 pin)
const int RELAY_PIN = 5;    // Relay control pin (active HIGH)
const int LED_GREEN = 2;    // Optional: unlock indicator
const int LED_RED   = 4;    // Optional: no-signal indicator

// ----- Sampling / window -----
const int SAMPLE_RATE = 250;                     // Hz
const int SAMPLE_PERIOD_MS = 1000 / SAMPLE_RATE; // ~4 ms
const int WINDOW_SECONDS = 4;                    // seconds of data used for detection
const int WINDOW_SAMPLES = SAMPLE_RATE * WINDOW_SECONDS;

// ----- Detection thresholds (tune on your bench) -----
float MIN_STDDEV = 30.0f;    // minimal stddev required to consider signal non-flat
int MIN_PEAKS_WINDOW = 4;    // minimal number of R-like peaks in the window
int PEAK_DELTA = 100;        // amplitude threshold (centered units) for peak detection
const int MIN_RR_MS = 250;   // minimum ms between peaks (ignore too-close events)

// ----- Unlock & persistence params -----
const unsigned long UNLOCK_MS = 3000;    // how long to hold relay ON (ms)
const unsigned long PRESENCE_HOLD_MS = 3000; // require detection for this long before unlocking

// ----- Circular buffer for samples -----
volatile int16_t samples[WINDOW_SAMPLES];
volatile int writeIndex = 0;
volatile bool bufferFilled = false;

unsigned long lastSampleMillis = 0;
unsigned long detectStartMs = 0;

// ----- Utility: pin setup -----
void setupPins() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  delay(200);

  setupPins();

  analogReadResolution(12);                 // ADC 0..4095
  analogSetPinAttenuation(ECG_PIN, ADC_11db);

  Serial.println("PulseLock - Human Detection + Serial Unlock");
  Serial.println("Type 'unlock' to force unlock or 'status' to get detection details");
  Serial.println("Project doc (local): file:///mnt/data/Skip to content.pdf");

  // Warm the buffer a bit with initial readings so recent data exists
  for (int i = 0; i < WINDOW_SAMPLES / 4; ++i) {
    int raw = analogRead(ECG_PIN);
    samples[writeIndex++] = (int16_t)(raw - 2048);
    if (writeIndex >= WINDOW_SAMPLES) { writeIndex = 0; bufferFilled = true; }
    delay(2);
  }
  lastSampleMillis = millis();
}

// ----- Circular buffer push -----
void pushSample(int16_t v) {
  samples[writeIndex++] = v;
  if (writeIndex >= WINDOW_SAMPLES) {
    writeIndex = 0;
    bufferFilled = true;
  }
}

// ----- Copy last N samples into outBuf (non-volatile) -----
void copyWindow(int16_t *outBuf, int N) {
  int idx = writeIndex;
  int start = idx - N;
  if (start < 0) start += WINDOW_SAMPLES;
  for (int i = 0; i < N; ++i) {
    int s = start + i;
    if (s >= WINDOW_SAMPLES) s -= WINDOW_SAMPLES;
    outBuf[i] = samples[s];
  }
}

// ----- Stats: mean and stddev -----
void computeStats(const int16_t *buf, int N, float &mean, float &stddev) {
  double sum = 0;
  for (int i = 0; i < N; ++i) sum += buf[i];
  mean = (float)(sum / N);
  double ss = 0;
  for (int i = 0; i < N; ++i) {
    double d = buf[i] - mean;
    ss += d * d;
  }
  stddev = (float)sqrt(ss / N);
}

// ----- Basic R-peak detector: count positive local maxima above threshold -----
int detectPeaks(const int16_t *buf, int N, float mean) {
  int peaks = 0;
  int lastPeakTms = -999999;
  for (int i = 1; i < N - 1; ++i) {
    if (buf[i] > buf[i-1] && buf[i] >= buf[i+1]) {
      int amp = abs(buf[i] - (int)mean);
      if (amp > PEAK_DELTA) {
        int t_ms = (i * 1000) / SAMPLE_RATE;
        if (lastPeakTms < 0 || (t_ms - lastPeakTms) > MIN_RR_MS) {
          peaks++;
          lastPeakTms = t_ms;
        }
      }
    }
  }
  return peaks;
}

// ----- Decide if last window indicates human ECG presence -----
bool isHumanInWindow(const int16_t *buf, int N) {
  float mean = 0, sd = 0;
  computeStats(buf, N, mean, sd);
  if (sd < MIN_STDDEV) return false;               // too flat -> likely not attached
  int peaks = detectPeaks(buf, N, mean);
  if (peaks < MIN_PEAKS_WINDOW) return false;      // not enough beats
  return true;
}

// ----- Unlock action (relay + LED) -----
void doUnlock() {
  Serial.println("UNLOCK: activating relay");
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(RELAY_PIN, HIGH);
  delay(UNLOCK_MS);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_GREEN, LOW);
  Serial.println("UNLOCK: done");
}

// ----- Diagnostic status print -----
void printStatus() {
  static int16_t tmp[WINDOW_SAMPLES];
  copyWindow(tmp, WINDOW_SAMPLES);
  float mean, sd;
  computeStats(tmp, WINDOW_SAMPLES, mean, sd);
  int peaks = detectPeaks(tmp, WINDOW_SAMPLES, mean);
  Serial.printf("Status -> stddev=%.1f, peaks=%d (window %ds)\n", sd, peaks, WINDOW_SECONDS);
}

// ----- Main loop -----
void loop() {
  unsigned long now = millis();

  // sampling at SAMPLE_RATE
  if ((long)(now - lastSampleMillis) >= SAMPLE_PERIOD_MS) {
    lastSampleMillis += SAMPLE_PERIOD_MS;
    int raw = analogRead(ECG_PIN);               // 0..4095
    int16_t centered = (int16_t)(raw - 2048);   // center
    pushSample(centered);
  }

  // Serial commands: manual override + status
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.equalsIgnoreCase("unlock")) {
      Serial.println("Manual command: unlock");
      doUnlock();
    } else if (s.equalsIgnoreCase("status")) {
      printStatus();
    } else if (s.equalsIgnoreCase("tune")) {
      // optional: print current thresholds for quick adjustment
      Serial.printf("TUNING -> MIN_STDDEV=%.1f MIN_PEAKS_WINDOW=%d PEAK_DELTA=%d\n",
                    MIN_STDDEV, MIN_PEAKS_WINDOW, PEAK_DELTA);
    } else {
      Serial.println("Unknown command. Use 'unlock' or 'status' (or 'tune').");
    }
  }

  // automatic detection on the latest window
  if (bufferFilled) {
    static int16_t tmp[WINDOW_SAMPLES];
    copyWindow(tmp, WINDOW_SAMPLES);
    bool human = isHumanInWindow(tmp, WINDOW_SAMPLES);

    if (human) {
      if (detectStartMs == 0) detectStartMs = millis();
      if (millis() - detectStartMs >= PRESENCE_HOLD_MS) {
        // detected as human continuously for hold period => unlock
        doUnlock();
        detectStartMs = 0;
        // avoid immediate re-unlock; small cooldown
        delay(1000);
      }
      digitalWrite(LED_RED, LOW);
    } else {
      detectStartMs = 0;
      digitalWrite(LED_RED, HIGH); // indicate no human ECG
    }
  }

  // be cooperative with OS
  yield();
}
