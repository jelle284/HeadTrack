#include "I2Cdev.h"
#include "MPU9250_9Axis_MotionApps41.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// ====================================ooo0000ooo================================
// ==============================================================================
unsigned long startTime = 0, loopTime;

bool b_Debug = false;

//                                    DATA PACKET
// ==============================================================================
struct DataPacket_t {
  float quat[4], acc[3], gyro[3];
  bool TriggerBtn;
} DataPacket;

char incomingByte;
//                                       MPU
// ==============================================================================
MPU9250 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

const float ACC_SCALING = 0.00119750977;  // 2 g full scale output
const float GYRO_SCALING = 0.00013316211; // 250 deg/s full scale output
//const float GYRO_SCALING = 0.00106529695; // 2000 deg/s full scale output
//const float GYRO_SCALING = 0.0171110711; // 250 deg/s full scale output, 8 bit signed int

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

Quaternion q;
VectorInt16 Acc;
int16_t gyro[3];

//                                    LED
// =================================================================================
#define RED_PIN 9
#define GREEN_PIN 10
#define BLUE_PIN 11

void setColor(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

//                                  SETUP
// =================================================================================
void setup()
{
  // Start MPU
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    while (1) {}
  }

  // Turn off LED and open serial
  setColor(0, 0, 0);
  Serial.begin(115200);

}

//                                     LOOP
// =================================================================================
void loop()
{
  // Timing
  startTime = millis();

  // MPU
  // ----------------------------------------------
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    // waiting for mpu data to arrive

    // Serial handling
    if (Serial.available() > 0) {
      incomingByte = Serial.read();
    }

    switch (incomingByte) {

      default:
        break;

      // send full sample [q, a, w]
      case 's':
        DataPacket.quat[0] = q.w;
        DataPacket.quat[1] = q.x;
        DataPacket.quat[2] = q.y;
        DataPacket.quat[3] = q.z;

        DataPacket.acc[0] = ACC_SCALING * Acc.x;
        DataPacket.acc[1] = ACC_SCALING * Acc.y;
        DataPacket.acc[2] = ACC_SCALING * Acc.z;

        DataPacket.gyro[0] = GYRO_SCALING * gyro[0];
        DataPacket.gyro[1] = GYRO_SCALING * gyro[1];
        DataPacket.gyro[2] = GYRO_SCALING * gyro[2];

        DataPacket.TriggerBtn = false;

        Serial.write((char*)(&DataPacket), sizeof(DataPacket));
        break;

      // LED color
      case  'n':
        setColor(0, 0, 0);
        break;
      case  'R':
        setColor(255, 0, 0);
        break;
      case 'G':
        setColor(0, 255, 0);
        break;
      case  'B':
        setColor(0, 0, 255);
        break;
    }
    incomingByte = 0;
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Read quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Read acceleration
    mpu.dmpGetAccel(&Acc, fifoBuffer);

    // read gyro
    mpu.dmpGetGyro(gyro, fifoBuffer);
  }

  // loop end
  // ----------------------------------------------
  loopTime = millis() - startTime;
  if (b_Debug) {
    Serial.print("loop time: ");
    Serial.print(loopTime);
    Serial.println(" ms.");
  }
}
