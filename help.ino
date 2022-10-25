#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t ax_target;

#define LED_PIN 13
bool blinkState = false;

#define PIN_IN3  32
#define PIN_IN4  33
#define PIN_IN1  27 // ESP32 pin GIOP27 connected to the IN1 pin L298N
#define PIN_IN2  26 // ESP32 pin GIOP26 connected to the IN2 pin L298N
#define PIN_ENA  14 // ESP32 pin GIOP14 connected to the EN1 pin L298N
#define PIN_ENB  12

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

TwoWayMotor left(PIN_IN1, PIN_IN2, PIN_ENA);
TwoWayMotor right(PIN_IN4, PIN_IN3, PIN_ENB);
// Roughly 1/64 conversion factor
//                       p,     i, d,  target
PIDController controller(0.020, 0, 0, 0);
//PIDController controller(0.010, 0, -0.00015, 0);

void setup() {
  Wire.begin();
   Serial.begin(38400);
  
  // initialize device
  // Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // verify connection
  // Serial.println("Testing device connections...");
  // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  left.init();
  right.init();
    
  ax_target = accelgyro.getAccelerationX();
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Turn motors proportional to x accel
//    int err = map(ax, -16386, 16385, -255, 255);
    delay(2);
    float requested = controller.calcPid(ax, 0.002);
    int amt = requested;// constrain(requested, -255, 255);
    
     Serial.print("Err: "); Serial.println(requested);
    left.write(amt);
    right.write(amt);
}
