class PIDController {
  private:
    float p, i, d;
    float err_acc = 0;
    float target, err_prev = 0;
  public:
    PIDController(float p, float i, float d, float target):
      p(p), i(i), d(d), target(target) {
    }
    
    void setTarget(float t) {
      target = t;
    }
    
    float calcPid(float currentPos, float dt) {
      float err = currentPos - target;
      err_acc = (err_acc + dt * err) * 0.9;
      float dErr = (err - err_prev) / dt;
      float adjust = p * err + err_acc + dErr;
      err_prev = err;
      return adjust;
    }
};

void setup() {
  
}

void loop() {
  
}
