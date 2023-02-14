#ifndef STEPPER_H
#define STEPPER_H
#include <stdint.h>

// hack: global counter to allocate new ledc channels to steppers
static int channelIdx = 0;

/**
 * Stepper motor abstraction.  Input range is from
 * -255 to 255; maxStepRate is in Hz.
 * 
 * NOTE: This class consumes ~~one~~ TWO ledc channels per instance!  If you want to use ledc
 * channels for other purposes, this class needs to be rewritten.
 */
class Stepper {
  private:
    const int step, dir, stepChannel, maxStepRate, dirChannel;
    const bool reversed;
  public:
    Stepper(int step, int dir, int maxStepRate, bool reversed) :
      step(step), dir(dir), stepChannel(channelIdx++), maxStepRate(maxStepRate), reversed(reversed), dirChannel(channelIdx++) {}

    void init();
    void write (int amount);
    /**
     * Given a note and octave, play on the stepper motor
     * until stopNote() is called.
     */
    void playNote(uint8_t note, uint8_t octave);
    void stopNote();
};
#endif
