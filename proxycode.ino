// Proxy Arduino Code: Generates PWM signals to simulate FS-iA10B receiver output

#include <Arduino.h>

// Define RC channels and signal pins (simulating FS-iA10B receiver outputs)
const int rcPins[4] = {2, 3, 4, 5};  // Output pins for RC channels (PWM signals)

// Define the PWM ranges for simulation (1000-2000 µs range)
int rcMin[4] = {988, 985, 982, 990};     // Minimum value for each channel
int rcCenter[4] = {1500, 1500, 1500, 1500}; // Default center value (no stick movement)
int rcMax[4] = {2012, 2015, 2018, 2010};  // Maximum value for each channel

// Time intervals to update PWM values
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 20; // Update every 20 ms (~50 Hz)

void setup() {
  Serial.begin(115200);
  
  // Set RC pins as outputs
  for (int i = 0; i < 4; i++) {
    pinMode(rcPins[i], OUTPUT);
  }

  Serial.println("Proxy Arduino for FS-iA10B initialized.");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Only update PWM signals at regular intervals
  if (currentMillis - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentMillis;
    
    // Simulate PWM signals for each channel
    for (int i = 0; i < 4; i++) {
      int pwmValue = map(analogRead(A0 + i), 0, 1023, rcMin[i], rcMax[i]); // Map input to PWM range
      analogWrite(rcPins[i], pwmValue / 4);  // Use PWM on the defined pins (value should be scaled to 0-255 range)
    }
  }

  delay(1);  // Small delay to avoid blocking
}
