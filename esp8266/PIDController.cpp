#include "PIDController.h"

void PIDController::setTarget(float t) {
  target = t;
}
    
float PIDController::calcPid(float currentPos, float dt) {
  float err = currentPos - target;
  err_acc = (err_acc + dt * err) * err_mul;
  float dErr = (err - err_prev) / dt;
  float adjust = p * err + i * err_acc + d * dErr;
  err_prev = err;
  return adjust;
}
