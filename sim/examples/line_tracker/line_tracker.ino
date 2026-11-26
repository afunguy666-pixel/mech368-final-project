// Standalone navigation example for the simulator. firmware/robot runs the
// same direct-wired tracker with the BLE watchdog and latched remote E-stop.
// Four line sensors on A0..A3, obstacle LM393s on D12/D11, DRV8833 nSLEEP on D8.
// Swipe UP to calibrate and run, DOWN to stop.
#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <DirectLineTracker.h>

bool standalonePermission() { return true; }
mech368::DirectLineTracker tracker(standalonePermission);
void leftEncoderIsr() { tracker.leftEdge(); }
void rightEncoderIsr() { tracker.rightEdge(); }

void setup() {
  tracker.begin(leftEncoderIsr, rightEncoderIsr);
  Serial.println("line tracker ready: keep still, then swipe UP to calibrate and start, DOWN to stop");
}

void loop() {
  const int gesture = tracker.readGesture();
  if (gesture == GESTURE_UP) tracker.start();
  if (gesture == GESTURE_DOWN) {
    tracker.stop();
    Serial.println("gesture DOWN: stopping");
  }
  tracker.update();
}
