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
float ypr[3];
float pitch, roll;
int motorPins[4] = {5,9,3,6};
int motorSpeeds[4] = {0,0,0,0}, throttle;
float pidP_pitch = 0,pidI_pitch = 0, pidD_pitch = 0;
float pidP_roll = 0, pidI_roll = 0, pidD_roll = 0;
double kp = 1,
       ki = 0.005,
       kd = 0.2;
float prevPitchError = 0, pitchError = 0,rollError = 0, prevRollError = 0;
double currTime = 0, prevTime = 0;

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
  Serial.begin(2000000);
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
  prevTime = currTime;
  currTime = millis();
  float elapsedTime = (currTime - prevTime)/1000;
  if (!dmpReady){
   Serial.println("oh hooo");
    return;
  }
  if (Serial.available() > 0) {
    int data = Serial.parseInt();
    if (data > 0) {
      

      changeMotorSpeeds(data); 
      
      setMotorSpeed();
    }
  }
  getAngles();
  pitchError = pitch;
  rollError = roll;
  pidP_pitch = kp * pitchError;
  pidP_roll = kp * rollError;
  if(-3 < pitchError && pitchError < 3){
//    Serial.println("tf");
    pidI_pitch = pidI_pitch + (ki*pitchError);
  }
  if(-3 < rollError && rollError < 3){
//    Serial.println("tf");
    pidI_roll = pidI_roll + (ki*rollError);
  }
  pidD_pitch = kd*((pitchError - prevPitchError)/elapsedTime);
  pidD_roll = kd*((rollError - prevRollError)/elapsedTime);
  int PID_pitch = pidP_pitch + pidI_pitch + pidD_pitch;
  int PID_roll = pidP_roll + pidI_roll + pidD_roll;
  if(PID_pitch < -255)
    PID_pitch = -255;
  else if(PID_pitch > 255)
    PID_pitch = 255;
  if(PID_roll < -255)
    PID_roll = -255;
  else if(PID_roll > 255)
    PID_roll = 255;
  motorSpeeds[0] = throttle - PID_pitch;
  motorSpeeds[2] = throttle + PID_pitch;
  motorSpeeds[1] = throttle + PID_roll;
  motorSpeeds[3] = throttle - PID_roll;
  if( motorSpeeds[0] > 255)
    motorSpeeds[0] = 255;
  else if(motorSpeeds[0] < 0)
    motorSpeeds[0] = 0;
  if(motorSpeeds[2] > 255)
    motorSpeeds[2] = 255;
  else if(motorSpeeds[2] < 0)
    motorSpeeds[2] = 0;
  if( motorSpeeds[1] > 255)
    motorSpeeds[1] = 255;
  else if(motorSpeeds[1] < 0)
    motorSpeeds[1] = 0;
  if(motorSpeeds[3] > 255)
    motorSpeeds[3] = 255;
  else if(motorSpeeds[3] < 0)
    motorSpeeds[3] = 0;
  if(throttle > 50)
  setMotorSpeed();
//  Serial.print(" PID_pitch : ");
//  Serial.println(pidD_pitch);
  Serial.print(" Motor 1 : ");
  Serial.println(motorSpeeds[0]);
  Serial.print(" Motor 2 : ");
  Serial.println(motorSpeeds[2]);
  prevPitchError = pitchError;
  prevRollError = rollError;
}

void changeMotorSpeeds(int mSpeed) {
  for (int i = 0; i < 4; i++)
    motorSpeeds[i] = mSpeed;
  throttle = mSpeed;
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
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(pitch = ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(roll = ypr[2] * 180 / M_PI);
  Serial.println(motorSpeeds[0]);
}
