#include <Wire.h>
#define MPU_ADDR 0x68

int16_t rAccX, rAccY, rAccZ, rGyroX, rGyroY, rGyroZ,rTemp;
long long raxOffset, rayOffset, razOffset, rgxOffset, rgyOffset, rgzOffset;
double accX, accY, accZ, gyroX, gyroY, gyroZ,temp;
double gyroAX=0, gyroAY=0, gyroAZ=0, accAX=0, accAY=0, accAZ=0;
float pitchAngle = 0, rollAngle = 0,pitchOut=0, rollOut=0;
void mpuInit();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Initializing mpu...");
  mpuInit();
  Serial.println("Press 1 to calibrate mpu.");
  while(!Serial.available());
  int opt = Serial.parseInt();
  if(opt == 1)
    calibrateMPU();
}


long dlay = millis();
void loop() {
  long prevMillis = millis();
  calcAngles();
  if(millis() - dlay > 300){
    Serial.printf("Pitch : %f, Roll : %f\n",pitchOut,rollOut);
    Serial.printf("Gyro => X: %f, Y: %f, Z: %f\n",rollAngle,pitchAngle,gyroAZ);
    dlay = millis();
  }

  //remaining prog
  while((millis() - prevMillis) < 10);
}


void calibrateMPU(){
  Serial.println("Calibrating mpu, do not move.");
  int samples = 1000;
  for(int i = 0; i < samples; i++){
    getRawData();
    rgxOffset += rGyroX;
    rgyOffset += rGyroY;
    rgzOffset += rGyroZ;
  }
  rgxOffset /= samples;
  rgyOffset /= samples;
  rgzOffset /= samples;
}
void getRawData(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);
  rAccX = Wire.read() << 8 | Wire.read();
  rAccY = Wire.read() << 8 | Wire.read();
  rAccZ = Wire.read() << 8 | Wire.read();
  rTemp = Wire.read() << 8 | Wire.read();
  rGyroX = Wire.read() << 8 | Wire.read();
  rGyroY = Wire.read() << 8 | Wire.read();
  rGyroZ = Wire.read() << 8 | Wire.read();
}
bool setInitial = true;
void calcAngles(){
  getRawData();
  accX = rAccX/16384.0;
  accY = rAccY/16384.0;
  accZ = rAccZ/16384.0;
  temp = rTemp/340 + 36.53;

  float acc = sqrt(accX*accX + accY*accY + accZ*accZ);
  accAY = (asin(accX/acc)*180)/PI;
  accAX = (asin(accY/acc)*180)/PI;
  accAZ = (asin(accZ/acc)*180)/PI;
  
  rollAngle += (rGyroX-rgxOffset)*0.000076;
  pitchAngle += (rGyroY-rgyOffset)*0.000076;
  gyroAZ = (rGyroZ-rgzOffset)*0.000076;

  pitchAngle += rollAngle * sin(gyroAZ * (PI/180));               //If the IMU has yawed transfer the roll angle to the pitch angel
  rollAngle -= pitchAngle * sin(gyroAZ * (PI/180));
  
  if(setInitial){
    pitchAngle = accAY;
    rollAngle = accAX;
    setInitial = false;
  }
  else{
    pitchAngle = pitchAngle*0.996 + accAY*0.004;
    rollAngle = rollAngle*0.996 + accAX*0.004;
  }
  //complementary filter
  pitchOut = pitchOut*0.9 + pitchAngle*0.1;
  rollOut = rollOut*0.9 + rollAngle*0.1; 

}

void mpuInit(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x0);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x5);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x0);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x0);
  Wire.endTransmission();
}
