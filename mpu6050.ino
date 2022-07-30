#include <Wire.h>
#include <ESP8266WiFi.h>
#define MPU_ADDR 0x68

int16_t rAccX, rAccY, rAccZ, rGyroX, rGyroY, rGyroZ,rTemp;
long long raxOffset, rayOffset, razOffset, rgxOffset, rgyOffset, rgzOffset;
double accX, accY, accZ, gyroX, gyroY, gyroZ,temp;
double gyroAX=0, gyroAY=0, gyroAZ=0, accAX=0, accAY=0, accAZ=0;
float pitchAngle = 0, rollAngle = 0,pitchOut=0, rollOut=0,prevPitch = 0, prevRoll = 0;
int pidPPitch = 0, pidIPitch = 0, pidDPitch = 0, pidPRoll = 0, pidIRoll = 0, pidDRoll = 0;
int pidPitch, pidRoll,throttle=0;
void mpuInit();

int motorPins[4] = {14,12,13,15};
int motorSpeeds[4] = {0,0,0,0};

double kp = 0.45, //0.45
       ki =0.02, //0.05
       kd =0.15; //0.16

WiFiClient client;
WiFiServer server(80);

char* ssid = "GERONIMO",
*psswd = "drs12345";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  WiFi.begin(ssid,psswd);
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  pinMode(2,OUTPUT);
  server.begin();
  Serial.println("Initializing mpu...");
  mpuInit();
  Serial.println("Press 1 to calibrate mpu.");
  digitalWrite(2,HIGH);
  delay(3000);
  calibrateMPU();
  digitalWrite(2,LOW);
  
  for(int i = 0; i < 4; i++){
    pinMode(motorPins[i],OUTPUT);
    analogWrite(motorPins[i],0);
  }
}


long dlay = millis();
void loop() {
  if(!client)
    client = server.available();
  long prevMillis = millis();
  calcAngles();
//  if(millis() - dlay > 100){
//    Serial.printf("Pitch : %f, Roll : %f\n",pitchOut,rollOut);
//    Serial.printf("Gyro => X: %f, Y: %f, Z: %f\n",rollAngle,pitchAngle,gyroAZ);
//    dlay = millis();
//  }
  if(client && client.available() > 0){
    String request = client.readStringUntil('\r');
    float val = (request.substring(1,request.length())).toFloat();
    switch(request[0]){
      case 'A':
        throttle = (int)val;
        break;
      case 'P':
        kp = val;
        break;
      case 'I':
        ki = val;
        break;
      case 'D':
        kd = val;
        break;
//      Serial.println(throttle);
      //update throttle
    }
  }
  pidCal();
  while((millis() - prevMillis) < 10);
}

void pidCal(){
  pidPPitch = kp*pitchOut;
  if(abs(pitchOut) < 10)
    pidIPitch += ki*pitchOut;
  pidDPitch = kd*(pitchOut-prevPitch)*0.01; // 0.01 = time elapsed(100Hz)
  pidPitch = pidPPitch + pidDPitch + pidIPitch;
  pidPRoll = kp*rollOut;
  if(abs(rollOut) < 10)
    pidIRoll += ki*rollOut;
  pidDRoll = kd*(rollOut - prevRoll)*0.01;
  pidRoll = pidPRoll + pidDRoll + pidIRoll;
  if(pidPitch < -150)
    pidPitch = -150;
  else if(pidPitch > 150)
    pidPitch = 150;
  if(pidRoll < -150)
    pidRoll = -150;
  else if(pidRoll > 150)
    pidRoll = 150;
  setMotorSpeeds(pidPitch,pidRoll);
   
}

void setMotorSpeeds(int pidP,int pidR){
  int out = 0;
  pidP = -pidP;
  if(!throttle)
    pidP = pidR = 0;
  analogWrite(motorPins[0],((out = throttle + pidP - pidR) > 255)?255:((out < 0)?0:out));
  analogWrite(motorPins[1],((out = throttle + pidP + pidR) > 255)?255:((out < 0)?0:out));
  analogWrite(motorPins[2],((out = throttle - pidP - pidR) > 255)?255:((out < 0)?0:out));
  analogWrite(motorPins[3],((out = throttle - pidP + pidR) > 255)?255:((out < 0)?0:out));
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
    prevPitch = accAY;
    rollAngle = accAX;
    prevRoll = accAX;
    setInitial = false;
  }
  else{
    pitchAngle = pitchAngle*0.98 + accAY*0.02;
    rollAngle = rollAngle*0.98 - accAX*0.02;
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
