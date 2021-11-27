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
int motorPins[4] = {5, 9, 3, 6};
int motorSpeeds[4] = {0, 0, 0, 0};
int testPower = 25;
int changeLimit = 50;
int maxLimit = 75;
float euler[3];
float ypr[3];
float ypr_deg[3];


void changeMotorSpeeds(int mSpeed);
void setMotorSpeed(); 
void getAngles();

void setup() {
  for (int i = 0; i < 4; i++)
    pinMode(motorPins[i], OUTPUT);
    changeMotorSpeeds(0);
    setMotorSpeed();

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(1000000);
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
  
}

void loop() {
  if (!dmpReady)
    return;
  if (Serial.available() > 0) {
    int data = Serial.parseInt();
    if (data > 0) {
      

      changeMotorSpeeds(data); 
      
      setMotorSpeed();
    }
  }
  getAngles();
  balanceCopter();
  delay(2);
//  if (abs(ypr_deg[1]) > 2) {
//    int currSpeeds[2] = {motorSpeeds[0], motorSpeeds[2]};
//    if (ypr_deg[1] < -2) {
//      //change values accordingly. Motors should never completely turn off at any stage.
//      while (ypr_deg[1] < -2) {
//        getAngles();
//        int change1 = map(ypr_deg[1], 0, -60, 0,100),change2;
//        change2 = change1;
//        if(currSpeeds[0] < 30 || currSpeeds[1] < 30)
//          continue;
//        if(currSpeeds[0] == 255){
//          change2 = change1 *2;
//          change1 = 0;
//        }
//        if(currSpeeds[0] + change1 >= 255){
//          int adjust = currSpeeds[0] + change1 - 255;
//          change1 -= adjust;
//          change2 += adjust;
//        }
//        if(ypr_deg[1] > -10){
//          change1 -= 20;
//          change2 -= 30;
//        }
//        motorSpeeds[0] =  currSpeeds[0] + change1;
//        motorSpeeds[2] = currSpeeds[1] - change2;
//        Serial.print("motorSpeed 1 : ");
//        Serial.println(motorSpeeds[0]);
//        Serial.print("motorSpeed 2 : ");
//        Serial.println(motorSpeeds[2]);
//        setMotorSpeed();
//      }
//              
//    }
//    else {
//      while (ypr_deg[1] > 2) {
//        getAngles();
//        int change1 = map(ypr_deg[1], 0, 90, 0, 100),change2;
//        change2 = change1;
//        if(currSpeeds[0] < 30 || currSpeeds[1] < 30)
//          continue;
//        if(currSpeeds[1] == 255){
//          change1= change1 *2;
//          change2 = 0;
//        }
//        if(currSpeeds[1] + change2 >= 255){
//          int adjust = currSpeeds[1] + change2 - 255;
//          change1 += adjust;
//          change2 -= adjust;
//        }        
//        motorSpeeds[0] = currSpeeds[0] - change1;
//        motorSpeeds[2] = currSpeeds[1] + change2;
//        Serial.print("motorSpeed 1 : ");
//        Serial.println(motorSpeeds[0]);
//        Serial.print("motorSpeed 2 : ");
//        Serial.println(motorSpeeds[2]);
//        setMotorSpeed();
//      }
//    }
//    
//      motorSpeeds[0] = currSpeeds[0];
//      motorSpeeds[2] = currSpeeds[1];
//      setMotorSpeed();
//    delay(2);
//  }
}


void changeMotorSpeeds(int mSpeed) {
  for (int i = 0; i < 4; i+=2)
    motorSpeeds[i] = mSpeed;
}
void setMotorSpeed() {
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
  Serial.print(ypr_deg[0] = ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr_deg[1] = ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr_deg[2] = ypr[2] * 180 / M_PI);
  Serial.println(motorSpeeds[0]);
}

void balanceCopter(){

  getAngles();
  int pitch = ypr_deg[1];
  int roll = ypr_deg[2];
  int originalSpeeds[2] = {motorSpeeds[0], motorSpeeds[2]};
  int change1,change2, prevAngle;
  bool flag = false;
  if(abs(pitch) > 2){
    while(pitch > 2){
      getAngles();
      pitch = ypr_deg[1];
      if(originalSpeeds[0] < 20 || originalSpeeds[1] < 20)
        return;
      change1 = abs(pitch);
      if(prevAngle <= pitch)
        change1+= 20;
      else if(prevAngle > pitch){
        change1 -= 10;
      }
      change2 = change1;
      if(originalSpeeds[1] == 255){
        change2 = 0;
        change1 = change1*2;
      }
      if((originalSpeeds[1]+change2) > 255){
        int adjust = originalSpeeds[1]+change2 -255;
        change2 -= adjust;
        change1 += adjust; 
      }
      motorSpeeds[0] = originalSpeeds[0]- change1;
      motorSpeeds[2] = originalSpeeds[1]+ change2;
      setMotorSpeed();
              Serial.print("motorSpeed 1 : ");
        Serial.println(motorSpeeds[0]);
        Serial.print("motorSpeed 2 : ");
        Serial.println(motorSpeeds[2]);
      prevAngle = pitch;
      delay(2);
    }
    while(pitch < -2){
      getAngles();
      pitch = ypr_deg[1];
      if(originalSpeeds[0] < 20 || originalSpeeds[1] < 20)
        continue;
      change1 = abs(pitch);
      if(prevAngle >= pitch)
        change1 += 20;
      else if(prevAngle < pitch){
        change1 -= 10;
      }
      change2 = change1;
      if(originalSpeeds[1] == 255){
        change2 = 0;
        change1 = change1*2;
      }
      if((originalSpeeds[1]+change2) > 255){
        int adjust = originalSpeeds[1]+change2 -255;
        change2 -= adjust;
        change1 += adjust; 
      }
      motorSpeeds[0] = originalSpeeds[0]+ change1;
      motorSpeeds[2] = originalSpeeds[1]- change2;
              Serial.print("motorSpeed 1 : ");
        Serial.println(motorSpeeds[0]);
        Serial.print("motorSpeed 2 : ");
        Serial.println(motorSpeeds[2]);
      prevAngle = pitch;
      delay(2);
    }
    motorSpeeds[0] = originalSpeeds[0];
    motorSpeeds[2] = originalSpeeds[1];
    setMotorSpeed();
  }

}
