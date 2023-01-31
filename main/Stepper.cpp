#include "Stepper.h"
#include "Arduino.h"

void Stepper::init() {
  pinMode(dir, OUTPUT);
  pinMode(step, OUTPUT);
  digitalWrite(step, LOW);
  digitalWrite(dir, LOW);
  // Default freq of 500Hz
  ledcSetup(ledChannel, 500, 10);
  ledcAttachPin(step, ledChannel);
  // Write 0 to stop any output
  ledcWrite(ledChannel, 0);
}

void Stepper::write (int amount) {
  digitalWrite(dir, reversed ? amount < 0 : amount > 0);
  if (amount == 0) {
    ledcWrite(ledChannel, 0);
    return;
  }
  amount = abs(amount);
  // Change freq and set duty cycle to 1.
  int requestedFreq = (int)(((float) amount / 255.0f) * maxStepRate);
  int ra = ledcChangeFrequency(ledChannel, requestedFreq, 10);
  ledcWrite(ledChannel, 2000);
  // Serial.printf("lc: %d, rf: %d, ret1: %d\n", ledChannel, requestedFreq, ra);
}
