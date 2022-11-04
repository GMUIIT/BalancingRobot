 #include <ESP32Servo.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

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


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

TwoWayMotor left(PIN_IN1, PIN_IN2, PIN_ENA);
TwoWayMotor right(PIN_IN4, PIN_IN3, PIN_ENB);
// Roughly 1/64 conversion factor
//                       p,     i, d,  target
PIDController controller(0.012, 0, 0, 0);

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


void mpuInit() {
// initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(PIN_MPUINT, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//  // wait for ready
//  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  while (Serial.available() && Serial.read()); // empty buffer
//  while (!Serial.available());                 // wait for data
//  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(PIN_MPUINT));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(PIN_MPUINT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(38400);
  pinMode(PIN_TRIMPOT, INPUT);
  pinMode(PIN_DTRIM, INPUT);
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  mpuInit();
  
  // configure Arduino LED pin for output
  // pinMode(LED_PIN, OUTPUT);
  left.init();
  right.init();
}

int lastMillis = 0;
void loop() {
  // read raw accel/gyro measurements from device
  //  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if (!dmpReady) return;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[2] * 180 / M_PI);
  
  // Turn motors proportional to x accel
  float requested = controller.calcPid(ypr[1], (millis() - lastMillis) / 1000.0f);
  delay(2);
  int amt = requested;// constrain(requested, -255, 255);
  float n = analogRead(PIN_TRIMPOT);
  float d = analogRead(PIN_DTRIM);

  controller.p = mapf(n, 0.0, 4095.0, 0, 3000);
  controller.d = mapf(d, 0.0, 4095.0, 0, 2000);
  
//  Serial.printf("n: %f, d: %f\n", n, d);

//  Serial.print("Err: "); Serial.print(requested); Serial.print(", p: "); Serial.print(controller.p); Serial.print(", d: "); Serial.println(controller.d);
  Serial.printf("E: %f\n", requested);
  
  left.write(amt);
  right.write(amt);
}
