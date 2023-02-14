#include "Stepper.h"
#include "Arduino.h"

#define LEDC_RESOLUTION_BITS 14
#define DIR_OCTAVE_OFFSET 1

void Stepper::init() {
  pinMode(dir, OUTPUT);
  pinMode(step, OUTPUT);
  digitalWrite(step, LOW);
  digitalWrite(dir, LOW);
  // Setup ledc channels - step is attached immediately, but
  // we leave dir unattached since it's only for music
  ledcSetup(dirChannel, 0, LEDC_RESOLUTION_BITS);
  ledcSetup(stepChannel, 500, LEDC_RESOLUTION_BITS);
  ledcAttachPin(step, stepChannel);
  // Write 0 to stop any output
  ledcWrite(stepChannel, 0);
}

void Stepper::write (int amount) {
  digitalWrite(dir, reversed ? amount < 0 : amount > 0);
  if (amount == 0) {
    ledcWrite(stepChannel, 0);
    return;
  }
  amount = abs(amount);
  // Change freq and set duty cycle to 1.
  int requestedFreq = (int)(((float) amount / 255.0f) * maxStepRate);
  int ra = ledcChangeFrequency(stepChannel, requestedFreq, 10);
  ledcWrite(stepChannel, 2000);
  // Serial.printf("lc: %d, rf: %d, ret1: %d\n", stepChannel, requestedFreq, ra);
}


void Stepper::playNote(uint8_t note, uint8_t octave) {
  // Switch dir from manual digitalWrite control to ledc
  ledcAttachPin(dir, dirChannel);
  // Step at the desired frequency
  ledcWriteNote(stepChannel, (note_t) note, octave);
  // Change directions a little bit less frequently, to make sure
  // we don't actually move very far
  ledcWriteNote(dirChannel, (note_t) note, octave - DIR_OCTAVE_OFFSET);
}

void Stepper::stopNote() {
  write(0);
  // Switch dir() back to manual control
  ledcDetachPin(dir);
  pinMode(dir, OUTPUT);
}
