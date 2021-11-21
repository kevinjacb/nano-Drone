#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13
bool blinkState = false;
bool dmpReady = false;
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];
int motorPins[4] = {3, 5, 6, 9};
int motorSpeeds[4] = {0, 0, 0, 0};
int testPower = 25;
int changeLimit = 50;
int maxLimit = 75;
float euler[3];
float ypr[3];

void setup() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);


  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 4; i++)
    pinMode(motorPins[i], OUTPUT);
}

void loop() {
  if(Serial.available()>0){
    int data = Serial.parseInt();
    changeMotorSpeeds(data);
    setMotorSpeed();
  }
  if (!dmpReady)
    return;
  getAngles();
  if(abs(ypr[1])>2){
    if(ypr[1]<0){
      motorSpeeds[0] -= changeLimit;                        //need to check if the motorSpeeds are at the max or min, need to set threshold and
      motorSpeeds[3] += changeLimit;                        //change values accordingly. Motors should never completely turn off at any stage.
    }
    else{
      motorSpeeds[0] += changeLimit;
      motorSpeeds[3] -= changeLimit;
    }
  }
  
  delay(2);
}

void changeMotorSpeeds(int mSpeed){
  for (int i = 0; i< 4; i++)
    motorSpeeds[i] = mSpeed;
}
void setMotorSpeed(){
    for (int i = 0; i < 4; i++)
      analogWrite(motorPins[i], motorSpeeds[i]);
}

void getAngles() {
  mpu.resetFIFO();
  fifoCount = mpu.getFIFOCount();

  while (fifoCount < packetSize)fifoCount = mpu.getFIFOCount();

  mpu.getFIFOBytes(fifoBuffer, packetSize);

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180 / M_PI);
}
