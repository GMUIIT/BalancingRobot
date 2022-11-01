 #include <ESP32Servo.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"


#define LED_PIN 13

#define PIN_IN3  32
#define PIN_IN4  33
#define PIN_IN1  27 // ESP32 pin GIOP27 connected to the IN1 pin L298N
#define PIN_IN2  26 // ESP32 pin GIOP26 connected to the IN2 pin L298N
#define PIN_ENA  14 // ESP32 pin GIOP14 connected to the EN1 pin L298N
#define PIN_ENB  12
#define PIN_TRIMPOT 25
#define PIN_DTRIM 35
#define PIN_MPUINT 18

class PIDController {
  private:
//    float p, i, d;
    float err_acc = 0;
    float target, err_prev = 0;
  public:
    float p, i, d;
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
      float adjust = p * err + i * err_acc + d * dErr;
      err_prev = err;
      return adjust;
    }
};


class TwoWayMotor {
  private:
    int dirPinA, dirPinB, enPin;
  public:
    TwoWayMotor(int dirPinA, int dirPinB, int enPin):
      dirPinA(dirPinA), dirPinB(dirPinB), enPin(enPin) {
      }

      void init() {
        pinMode(dirPinA, OUTPUT);
        pinMode(dirPinB, OUTPUT);
        pinMode(enPin, OUTPUT);
        digitalWrite(dirPinA, LOW);
        digitalWrite(dirPinB, LOW);
        digitalWrite(enPin, LOW);
      }

      void write(int amount){
        if (amount < -255 || amount > 255) {
          // Serial.println("OUT OF RANGE URBAD");
          return;
        }
        if (amount < 0) {
          // Serial.println("Below zero");
          digitalWrite(dirPinA, HIGH);
          digitalWrite(dirPinB, LOW);
          amount *= -1;
        } else if (amount > 0) {
          // Serial.println("Banove zero");
          digitalWrite(dirPinA, LOW);
          digitalWrite(dirPinB, HIGH);
        } else {
          // Serial.println("ZEORKEORKOESR");
          digitalWrite(dirPinA, LOW);
          digitalWrite(dirPinB, LOW);
        }
        // Serial.print("Marco is: "); // Serial.println(amount);
        analogWrite(enPin, amount);
      }
};

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

TwoWayMotor left(PIN_IN1, PIN_IN2, PIN_ENA);
TwoWayMotor right(PIN_IN4, PIN_IN3, PIN_ENB);
// Roughly 1/64 conversion factor
//                       p,     i, d,  target
PIDController controller(0.012, 0, 0, 0);

MPU6050 mpu;

void setup() {
  Wire.begin();
  Serial.begin(38400);
  pinMode(PIN_TRIMPOT, INPUT);
  pinMode(PIN_DTRIM, INPUT);
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  mpu.dmpInitialize();
  // Help what are these numbers and why are they magic
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  //  attachInterrupt(digitalPinToInterrupt(PIN_MPUINT), dmpDataReady, RISING);
  //  mpuIntStatus = mpu.getIntStatus();
  
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  left.init();
  right.init();
    
//  ax_target = mpu.getAccelerationX();
}

int lastMillis = 0;
void loop() {
    // read raw accel/gyro measurements from device
//  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Turn motors proportional to x accel
//    int err = map(ax, -16386, 16385, -255, 255);
  float requested = controller.calcPid(ax, (millis() - lastMillis) / 1000.0f);
  delay(2);
  int amt = requested;// constrain(requested, -255, 255);
  float n = analogRead(PIN_TRIMPOT);
  float d = analogRead(PIN_DTRIM);

  controller.p = mapf(n, 0.0, 4095.0, 0, 0.04);
  controller.d = mapf(d, 0.0, 4095.0, 0, 0.40);
  
//  Serial.printf("n: %f, d: %f\n", n, d);

//  Serial.print("Err: "); Serial.print(requested); Serial.print(", p: "); Serial.print(controller.p); Serial.print(", d: "); Serial.println(controller.d);
  Serial.printf("E: %f\n", requested);
  
  left.write(amt);
  right.write(amt);
}
