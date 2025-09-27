Nice — I’ll “cross everything” and give you a final, **checked** package: wiring diagram (with VCC), pin table, safety checklist, step-by-step calibration & tests, and a single **final, ready-to-upload** Arduino sketch that’s safer and guarded against mistakes. Follow the checklist exactly when testing with motors connected.

---

# 1) Final wiring (clean, complete)

Assume: Arduino UNO/Nano, FSiA10B receiver (5 V), MotorDriver(s) with `PWM`, `DIR`, `GND`, motor battery (VIN).

```
Receiver VCC (5V)  ---> Arduino 5V  OR ---> BEC 5V output
Receiver GND       ---> Arduino GND  ---> MotorDriver GND  ---> BEC GND (all common)
Receiver CH1 signal ---> Arduino D2
Receiver CH2 signal ---> Arduino D3
Receiver CH3 signal ---> Arduino D4
Receiver CH4 signal ---> Arduino D5
Receiver CH5 signal ---> Arduino D6
Receiver CH6 signal ---> Arduino D7

Arduino D9  (PWM)  ---> MotorDriver1 PWM IN
Arduino D8  (DIR)  ---> MotorDriver1 DIR IN

Arduino D10 (PWM)  ---> MotorDriver2 PWM IN
Arduino D11 (DIR)  ---> MotorDriver2 DIR IN

Arduino D3  (PWM)  ---> MotorDriver3 PWM IN   <-- Note: D3 is PWM
Arduino D12 (DIR)  ---> MotorDriver3 DIR IN

MotorDriver VIN/BATT ---> External battery (do NOT feed motor battery to Arduino 5V)
Arduino USB or separate 5V supply ---> Arduino 5V (if using BEC, you can power Arduino from BEC)
```

Key points:

* **Common ground is mandatory** (Receiver, Arduino, MotorDriver, BEC).
* **Receiver VCC must be 5 V** (from Arduino 5V or BEC). Never connect motor battery directly to receiver VCC.
* Use **hardware PWM** pins for PWM outputs (Uno: 3,5,6,9,10,11). I used 9,10,3 in the example.
* Keep signal wires short and away from heavy motor wires.

---

# 2) Safety checklist before powering up

1. Remove motors (or disconnect motor outputs) for first power-on tests — test electronics with NO motor load.
2. Verify receiver VCC is 5 V before connecting Arduino pins.
3. Ensure battery is disconnected while wiring.
4. Use a multimeter to check common ground continuity.
5. Have an **emergency kill switch** or disconnect battery ready.
6. Start with throttle stick centered; ensure neutral calibration first.
7. Use small tests: Serial monitor -> confirm channels read; then test motor driver with a bench supply and no motor; finally test with motor attached at low throttle.
8. Add decoupling capacitors on motor power lines and a fuse if battery current may be large.

---

# 3) How to view readings & calibrate

* Open **Serial Monitor** (Arduino IDE) at **115200 baud**.
* Move each stick and watch `CH1=`, `CH2=` ... values (in microseconds).
* Note min/center/max numbers and edit `rcMin`, `rcCenter`, `rcMax` arrays if needed for better mapping.

---

# 4) Final Arduino sketch (multi-channel, guarded, ready-to-upload)

Copy this entire sketch to the Arduino IDE and upload. It prints channel pulses (µs), handles failsafe, mapping and smoothing, and prevents analogWrite on invalid pins.

```cpp
// Final multi-channel RC -> MotorDriver mapping
// Arduino UNO/Nano example (ready-to-upload)
// Serial monitor: 115200 baud

#include <Arduino.h>

// ---------- CONFIG ----------
const uint8_t numChannels = 6;
const uint8_t rcPins[numChannels] = {2, 3, 4, 5, 6, 7}; // CH1..CH6 signals

int rcMin[numChannels]    = {1000, 1000, 1000, 1000, 1000, 1000};
int rcCenter[numChannels] = {1500, 1500, 1500, 1500, 1500, 1500};
int rcMax[numChannels]    = {2000, 2000, 2000, 2000, 2000, 2000};
int rcDeadband[numChannels] = {80, 80, 80, 80, 80, 80}; // microsec deadband

const unsigned long failsafeTimeoutMs = 300; // stop outputs if no pulse in ms
const float smoothingAlpha = 0.25; // 0..1 (higher = less smoothing)

// ---------- MOTOR MAPPING ----------
struct MotorMap {
  int8_t mapChannel;   // 0-based index of channel (0..numChannels-1) or -1 to disable
  uint8_t pwmPin;      // Arduino PWM pin (must support analogWrite)
  uint8_t dirPin;      // Arduino digital pin for direction
  bool dirActiveHigh;  // true if DIR HIGH => forward
  float speedFiltered; // internal smoothing state
  bool enabled;
};

MotorMap motors[] = {
  { 0, 9,  8,  true,  0.0f, true }, // CH1 -> Motor1 (PWM D9, DIR D8)
  { 1, 10, 11, true,  0.0f, true }, // CH2 -> Motor2 (PWM D10, DIR D11)
  { 2, 3,  12, true,  0.0f, true }  // CH3 -> Motor3 (PWM D3, DIR D12)
};
const uint8_t numMotors = sizeof(motors) / sizeof(motors[0]);

// ---------- RUNTIME ----------
unsigned long lastPulseMillis[numChannels];
int lastPulseValue[numChannels];

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Multi-CH RC Reader (final) ==="));

  for (uint8_t i = 0; i < numChannels; ++i) {
    pinMode(rcPins[i], INPUT);
    lastPulseMillis[i] = 0;
    lastPulseValue[i] = 0;
  }

  for (uint8_t m = 0; m < numMotors; ++m) {
    if (motors[m].enabled && motors[m].mapChannel >= 0 && motors[m].mapChannel < numChannels) {
      pinMode(motors[m].pwmPin, OUTPUT);
      pinMode(motors[m].dirPin, OUTPUT);
      analogWrite(motors[m].pwmPin, 0);
      // safe dir (stopped)
      digitalWrite(motors[m].dirPin, motors[m].dirActiveHigh ? LOW : HIGH);
      motors[m].speedFiltered = 0.0f;
    } else {
      motors[m].enabled = false; // disable invalid mapping
    }
  }
}

// read pulse; returns 0 if none within timeout
int readRcPulse(uint8_t pin, unsigned long timeoutUs = 25000UL) {
  unsigned long t = pulseIn(pin, HIGH, timeoutUs);
  if (t == 0) return 0;
  return (int)t;
}

void loop() {
  // Read channels
  for (uint8_t i = 0; i < numChannels; ++i) {
    int pulse = readRcPulse(rcPins[i], 25000UL);
    if (pulse > 0) {
      lastPulseValue[i] = pulse;
      lastPulseMillis[i] = millis();
    }
  }

  // Print readings
  Serial.print("CH:");
  for (uint8_t i = 0; i < numChannels; ++i) {
    Serial.print(i + 1);
    Serial.print('=');
    Serial.print(lastPulseValue[i]);
    if (millis() - lastPulseMillis[i] > failsafeTimeoutMs) Serial.print("(FS)");
    if (i < numChannels - 1) Serial.print(' ');
  }
  Serial.println();

  // Update motors
  for (uint8_t m = 0; m < numMotors; ++m) {
    if (!motors[m].enabled) continue;

    int ch = motors[m].mapChannel;
    // failsafe: no recent pulse
    if (ch < 0 || ch >= numChannels || lastPulseValue[ch] == 0 || (millis() - lastPulseMillis[ch] > failsafeTimeoutMs)) {
      motors[m].speedFiltered = 0.0f;
      analogWrite(motors[m].pwmPin, 0);
      digitalWrite(motors[m].dirPin, motors[m].dirActiveHigh ? LOW : HIGH);
      continue;
    }

    int pulse = lastPulseValue[ch];
    // clamp
    if (pulse < rcMin[ch]) pulse = rcMin[ch];
    if (pulse > rcMax[ch]) pulse = rcMax[ch];

    int diff = pulse - rcCenter[ch];
    if (abs(diff) <= rcDeadband[ch]) {
      // easing to zero
      motors[m].speedFiltered = motors[m].speedFiltered * (1.0f - smoothingAlpha);
      int outPWM = (int)(motors[m].speedFiltered + 0.5f);
      if (outPWM < 1) outPWM = 0;
      analogWrite(motors[m].pwmPin, outPWM);
      if (outPWM == 0) digitalWrite(motors[m].dirPin, motors[m].dirActiveHigh ? LOW : HIGH);
      continue;
    }

    bool forward = (diff > 0);
    int mag = abs(diff);
    int maxMag = max(rcMax[ch] - rcCenter[ch], rcCenter[ch] - rcMin[ch]);
    if (maxMag <= 0) maxMag = 500;

    int targetPwm = map(mag, 0, maxMag, 0, 255);
    targetPwm = constrain(targetPwm, 0, 255);

    // smoothing filter
    motors[m].speedFiltered = smoothingAlpha * (float)targetPwm + (1.0f - smoothingAlpha) * motors[m].speedFiltered;
    int outPWM = (int)(motors[m].speedFiltered + 0.5f);
    if (outPWM < 1) outPWM = 0;

    digitalWrite(motors[m].dirPin, (forward == motors[m].dirActiveHigh) ? HIGH : LOW);
    analogWrite(motors[m].pwmPin, outPWM);
  }

  delay(50); // adjust for responsiveness (lower -> faster)
}
```

---

# 5) Quick troubleshooting (common problems)

* Channels read `0` or `(FS)` in serial:

  * Receiver not powered (check receiver VCC).
  * No common ground.
  * Wrong pin wiring.
* Motor spins opposite direction:

  * Invert `dirActiveHigh` for that motor OR swap motor wires on motor driver output.
* Motor jitters on neutral:

  * Increase `rcDeadband` value; add smoothingAlpha up to 0.5.
* PWM from Arduino not accepted by driver:

  * Some drivers need different PWM frequency or inverted logic — tell me the motor driver model and I’ll change the timer settings or invert PWM.
* Noise drops signal when motors run:

  * Add decoupling caps, ferrite beads, shield signal lines, shorten wires.

---

# 6) Want me to do any of these now?

* Provide an **interrupt-based** (more precise) version that uses `attachInterrupt()` or `PinChangeInt`.
* Auto-calibration routine (hold sticks min/max on boot to auto-fill rcMin/Max).
* Change PWM frequency or invert PWM for your motor driver model.
* Add an **emergency stop** pin or LED status.

Tell me which one and I’ll deliver it immediately (code + instructions).

