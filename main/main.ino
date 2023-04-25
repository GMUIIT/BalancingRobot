#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "BLEUart.h"
#include "Stepper.h"
#include "PIDController.h"

#define PIN_DIRA   32
#define PIN_STEPA  33
#define PIN_DIRB   26
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
PIDController anglePid(50000, 1000000, 0, 0.27);
PIDController velocityPid(1, 0, 0, 0);
Stepper left(PIN_STEPA, PIN_DIRA, 200, false);
Stepper right(PIN_STEPB, PIN_DIRB, 200, true);
// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
volatile bool mpuInterrupt = false; // interrruuuuuuuupt
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

void dmpDataReady() {
  mpuInterrupt = true;
}

// Fun: play a tone like the ESC startup noise
void playBootTone() {
  int baseOctave = 9;
  left.playNote(NOTE_G, baseOctave);
  right.playNote(NOTE_G, baseOctave);
  delay(200);
  left.playNote(NOTE_B, baseOctave);
  right.playNote(NOTE_B, baseOctave);
  delay(200);
  left.playNote(NOTE_D, baseOctave + 1);
  right.playNote(NOTE_D, baseOctave + 1);
  delay(200);
  left.stopNote();
  right.stopNote();
  delay(750);
  left.playNote(NOTE_G, baseOctave);
  right.playNote(NOTE_G, baseOctave);
  delay(750);
  left.playNote(NOTE_D, baseOctave + 1);
  right.playNote(NOTE_D, baseOctave + 1);
  delay(750);
  left.stopNote();
  right.stopNote();
  delay(500);
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
    case '?':
      ble.println("There's a few PID controllers here - A(acceleration, or angle), V(velocity)");
      ble.println("----- PID COMMANDS ------");
      ble.println("PID commands are of the format '<C><command...>' where <C> is the single-letter PID identifier, and <command> is one of the following commands with parameters.'");
      ble.println("'P<float>' - set PID's P value");
      ble.println("'I<float>' - set PID's I value");
      ble.println("'D<float>' - set PID's D value");
      ble.println("'M' - set err_mul (???) (might be how fast the integral decays)");
      ble.println("'S<float>' - change velocity PID setpoint");
      ble.println("\n");
      break;
    default:
      char actualCmd = rest.at(0);
      PIDController *pid;
      rest = rest.substr(1, rest.length());
      switch(cmd) {
        case 'A':
          pid = &anglePid;
          break;
        case 'V':
          pid = &velocityPid;
          break;
        default:
          ble.println("Unknown PID controller; use '?' for help");
          return;
      }
      switch (actualCmd) {
        case 'P':
          pid->p = atof(rest.c_str());
          break;
        case 'I':
          pid->i = atof(rest.c_str());
          break;
        case 'D':
          pid->d = atof(rest.c_str());
          break;
        case 'M':
          ble.println("Changing err_mul");
          pid->err_mul = atof(rest.c_str());
          break;
        case 'S': // Setpoint
          pid->target = atof(rest.c_str());
          ble.println("New setpoint: " + std::to_string(pid->target));
          break;
        default:
          ble.println("Unknown command " + actualCmd);
          break;
      }
  }
  ble.println("P: " + std::to_string(anglePid.p) + ", I: " + std::to_string(anglePid.i) + ", D: " + std::to_string(anglePid.d));
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
  int devStatus = mpu.dmpInitialize();

  // CALIBRATION:
  // TODO: figure out the proper offsets for our particular MPU6050.  We can
  // determine the offsets using these methods (CalibrateAccel and CalibrateGyro),
  // then read them back using PrintActiveOffsets, then plug them into the below
  // setXOffset() calls.  Until we're able to calibrate, though, we just leave the
  // offsets at the factory default.
  // mpu.CalibrateAccel(6);
  // mpu.CalibrateGyro(6);
  // supply your own gyro offsets here, scaled for min sensitivity
//  mpu.setXGyroOffset(51);
//  mpu.setYGyroOffset(8);
//  mpu.setZGyroOffset(21);
//  mpu.setXAccelOffset(1150);
//  mpu.setYAccelOffset(-50);
//  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
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
  Wire.setTimeout(1000);
  Serial.begin(115200);
  
  // initialize devices
  Serial.println("MPU Init...");
  mpuInit();
  Serial.println("Initializing BLE...");
  ble.init();
  Serial.println("Stepper init...");
  left.init();
  right.init();
  playBootTone();
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
  VectorInt16 accel, linAccel, worldAccel;
  float ypr[3];
  float dt = (millis() - lastMillis) / 1000.0f;
  lastMillis = millis();
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&accel, fifoBuffer);
  mpu.dmpGetLinearAccel(&linAccel, &accel, &gravity);
  mpu.dmpGetLinearAccelInWorld(&worldAccel, &linAccel, &q);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Feed X velocity to PID controller to get requested acceleration
  float angleSetpoint = velocityPid.calcPid(worldAccel.x / 4096.0f, dt);
  // Feed pitch angle to the PID controller to get motor speed
  float motorSpeed = anglePid.calcPid(angleSetpoint, dt);
  delay(5);

  // Forward PID controller's request to the motors.
  left.write(motorSpeed);
  right.write(motorSpeed);
}
