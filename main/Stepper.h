#ifndef STEPPER_H
#define STEPPER_H

// hack: global counter to allocate new ledc channels to steppers
static int channelIdx = 0;

/**
 * Stepper motor abstraction.  Input range is from
 * -255 to 255; maxStepRate is in Hz.
 * 
 * NOTE: This class consumes one ledc channel per instance!  If you want to use ledc
 * channels for other purposes, this class needs to be rewritten.
 */
class Stepper {
  private:
    const int step, dir, ledChannel, maxStepRate;
    const bool reversed;
  public:
    Stepper(int step, int dir, int maxStepRate, bool reversed) :
      step(step), dir(dir), ledChannel(channelIdx++), maxStepRate(maxStepRate), reversed(reversed) {}

    void init();
    void write (int amount);
};
#endif
