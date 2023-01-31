#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "BLEUart.h"
#include "Stepper.h"
#include "PIDController.h"

#define PIN_DIRA  32
#define PIN_STEPA  33
#define PIN_DIRB  26
#define PIN_STEPB  27
#define PIN_MPUINT 18


//////////// Prototypes
void mpuInit();
void dmpDataReady();
void handleIncoming(std::string &command);


/////////////// Globals
//                       p,     i, d,  target
int lastMillis = 0;
BLEUart ble("C4 Cat", handleIncoming);
PIDController controller(32000, 50, 0, 0);
Stepper left(PIN_STEPA, PIN_DIRA, 200, false);
Stepper right(PIN_STEPB, PIN_DIRB, 200, true);
int leftOffset = 0, rightOffset = 0;
// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
volatile bool mpuInterrupt = false; // interrruuuuuuuupt
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

void dmpDataReady() {
  mpuInterrupt = true;
}

void handleIncoming(std::string &command) {
  // First character is always the command name.
  // Min length is 1 (S)
  if (command.length() < 1) {
    return;
  }
  char cmd = command.at(0);
  std::string rest = command.substr(1, command.length());
  switch (cmd) {
    case 'P':
      controller.p = atof(rest.c_str());
      break;
    case 'I':
      controller.i = atof(rest.c_str());
      break;
    case 'D':
      controller.d = atof(rest.c_str());
      break;
    case 'R':
      ble.write("Resetting MPU...");
      mpuInit();
      break;
    case 'M':
      ble.write("Changing err_mul");
      controller.err_mul = atof(rest.c_str());
      break;
    case 'Y': // LEFT wheel
      leftOffset = atoi(rest.c_str());
      ble.write("Left wheel offset...");
      break;
    case 'A': // RIGHT wheel
      rightOffset = atoi(rest.c_str());
      ble.write("Right wheel offset...");
      break;
    case '?':
      ble.write("----- COMMANDS ------");
      ble.write("'P<float>' - set PID's P value");
      ble.write("'I<float>' - set PID's I value");
      ble.write("'D<float>' - set PID's D value");
      ble.write("'R' - reset MPU (broken)");
      ble.write("'M' - set err_mul (???) (might be how fast the integral decays)");
      ble.write("'Y<int>' - set left wheel offset (causing rotation)");
      ble.write("'M<int>' - set right wheel offset (causing rotation)");
      ble.write("\n");
      break;
    default:
      ble.write("Unknown command " + command);
      break;
  }
  ble.write("P: " + std::to_string(controller.p) + ", I: " + std::to_string(controller.i) + ", D: " + std::to_string(controller.d));
  
}

void mpuInit() {
// initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(PIN_MPUINT, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

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
  Serial.begin(115200);
  
  // initialize devices
  Serial.println("Initializing I2C devices...");
  mpuInit();
  Serial.println("Initializing BLE...");
  ble.init();
  left.init();
  right.init();
  Serial.println("Setup done!");
}

void loop() {
  int tStart = micros();
  // Requires DMP to be ready.
  if (!dmpReady) return;
  // Grab the current DMP packet
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Serial.println("I am a terrible micronctroller and I can't find my packets");
    return;
  }
  int t = micros() - tStart;
  if (t > 30000) {
    Serial.printf("T: %d\n", t);
  }
  // Tick the BLE subsystem, processing any device connections/disconnections that need to happen.
  //bleTick();

  // Extract the pitch angle from the DMP packet
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  // Feed pitch angle to the PID controller000
  float requested = controller.calcPid(ypr[1], (millis() - lastMillis) / 1000.0f);
  delay(5);

  // Forward PID controller's request to the motors.
  left.write(requested + leftOffset);
  right.write(requested + rightOffset);
}
