#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * Simple PID controller.
 */
class PIDController {
  private:
//    float p, i, d;
    float err_acc = 0;
    float err_prev = 0;

  public:
    float err_mul = 0.999;
    float target;
    float p, i, d;
    PIDController(float p, float i, float d, float target):
      p(p), i(i), d(d), target(target) {}
    
    void setTarget(float t);
    float calcPid(float currentPos, float dt);
};

#endif
