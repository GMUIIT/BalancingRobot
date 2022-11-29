#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#define PIN_DIRA  32
#define PIN_STEPA  33
#define PIN_DIRB  26
#define PIN_STEPB  27
#define PIN_MPUINT 18
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define DEVICE_NAME "C4 Cat"

class PIDController {
  private:
//    float p, i, d;
    float err_acc = 0;
    float target, err_prev = 0;

  public:
      float err_mul = 0.999;
    float p, i, d;
    PIDController(float p, float i, float d, float target):
      p(p), i(i), d(d), target(target) {
    }
    
    void setTarget(float t) {
      target = t;
    }
    
    float calcPid(float currentPos, float dt) {
      float err = currentPos - target;
      err_acc = (err_acc + dt * err) * err_mul;
      float dErr = (err - err_prev) / dt;
      float adjust = p * err + i * err_acc + d * dErr;
      err_prev = err;
      return adjust;
    }
};


//class TwoWayMotor {
//  private:
//    int dirPinA, dirPinB, enPin;
//  public:
//    TwoWayMotor(int dirPinA, int dirPinB, int enPin):
//      dirPinA(dirPinA), dirPinB(dirPinB), enPin(enPin) {
//      }
//
//      void init() {
//        pinMode(dirPinA, OUTPUT);
//        pinMode(dirPinB, OUTPUT);
//        pinMode(enPin, OUTPUT);
//        digitalWrite(dirPinA, LOW);
//        digitalWrite(dirPinB, LOW);
//        digitalWrite(enPin, LOW);
//      }
//
//      void write(int amount){
//        if (amount < -255 || amount > 255) {
//          // Serial.println("OUT OF RANGE URBAD");
//          return;
//        }
//        if (amount < 0) {
//          // Serial.println("Below zero");
//          digitalWrite(dirPinA, HIGH);
//          digitalWrite(dirPinB, LOW);
//          amount *= -1;
//        } else if (amount > 0) {
//          // Serial.println("Banove zero");
//          digitalWrite(dirPinA, LOW);
//          digitalWrite(dirPinB, HIGH);
//        } else {
//          // Serial.println("ZEORKEORKOESR");
//          digitalWrite(dirPinA, LOW);
//          digitalWrite(dirPinB, LOW);
//        }
//        // Serial.print("Marco is: "); // Serial.println(amount);
//        analogWrite(enPin, amount);
//      }
//};

// don't make too many of these
int channelIdx = 0;
/**
 * Stepper motor abstraction.  Input range is from
 * -255 to 255; maxStepRate is in Hz.
 */
class Stepper {
  private:
    const int step, dir, ledChannel, maxStepRate;
  public:
    Stepper(int step, int dir, int maxStepRate) :
      step(step), dir(dir), ledChannel(channelIdx++), maxStepRate(maxStepRate) {
    }

    void init() {
      pinMode(dir, OUTPUT);
      pinMode(step, OUTPUT);
      digitalWrite(step, LOW);
      digitalWrite(dir, LOW);
      // Default freq of 500Hz
      ledcSetup(ledChannel, 500, 10);
      ledcAttachPin(step, ledChannel);
      // Write 0 to stop any output
      ledcWrite(ledChannel, 0);
    }

    void write (int amount) {
      digitalWrite(dir, amount < 0);
      if (amount == 0) {
        ledcWrite(ledChannel, 0);
        return;
      }
      amount = abs(amount);
      // Change freq and set duty cycle to 1.
      int requestedFreq = (int)(((float) amount / 255.0f) * maxStepRate);
      int ra = ledcChangeFrequency(ledChannel, requestedFreq, 10);
      ledcWrite(ledChannel, 2000);
      Serial.printf("lc: %d, rf: %d, ret1: %d\n", ledChannel, requestedFreq, ra);
    }
};

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//TwoWayMotor left(PIN_IN1, PIN_IN2, PIN_ENA);
//TwoWayMotor right(PIN_IN4, PIN_IN3, PIN_ENB);
Stepper left(PIN_STEPA, PIN_DIRA, 200);
Stepper right(PIN_STEPB, PIN_DIRB, 200);

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


// BLE globals
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

/**
 * Write the given string to the virtual serial port.
 */
void writeString(std::string str) {
    pTxCharacteristic->setValue((uint8_t*)str.c_str(), str.length());
    pTxCharacteristic->notify();
    delay(10); // bluetooth stack will go into congestion, if too many packets are sent
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
      writeString("Resetting MPU...");
      mpuInit();
      break;
    case 'M':
      writeString("Changing err_mul");
      controller.err_mul = atof(rest.c_str());
      break;
    default:
      writeString("Unknown command " + command);
      break;
  }
  writeString("P: " + std::to_string(controller.p) + ", I: " + std::to_string(controller.i) + ", D: " + std::to_string(controller.d));
  
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
        handleIncoming(rxValue);
      }
    }
};

void bleInit() {
  //////// BLE init
  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void bleTick() {
      // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
//        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
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
  bleInit();
  left.init();
  right.init();
}

int lastMillis = 0;
void loop() {
  int tStart = micros();
  // Requires DMP to be ready.
  if (!dmpReady) return;
  // Grab the current DMP packet
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
  // Tick the BLE subsystem, processing any device connections/disconnections that need to happen.
  //bleTick();

  // Extract the pitch angle from the DMP packet
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  // Feed pitch angle to the PID controller
  float requested = controller.calcPid(ypr[1], (millis() - lastMillis) / 1000.0f);
  delay(5);

  // Forward PID controller's request to the motors.
  left.write(requested);
  right.write(requested);
  int t = micros() - tStart;
  if (t > 10000) {
    Serial.printf("E: %f, T: %d\n", requested, t);
  }
  }
}
