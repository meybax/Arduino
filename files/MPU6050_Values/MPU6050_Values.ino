#include <Servo.h>

// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>

int gySensitivity = 0;
int GyX = 0;
int GyY = 0;
int GyZ = 0;
int prev_time_elpsd = 0;
int angleX = 0;
int angleY = 0;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t rawAcX,rawAcY,rawAcZ,rawTmp,rawGyX,rawGyY,rawGyZ, zeroAcX, zeroAcY, zeroAcZ, zeroGyX, zeroGyY, zeroGyZ, AcX, AcY, AcZ, Tmp;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,12,true);  // request a total of 14 registers
  zeroAcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  zeroAcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L
  zeroAcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L))
  zeroGyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  zeroGyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  zeroGyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  rawAcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  rawAcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  rawAcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  rawTmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  rawGyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  rawGyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  rawGyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  int accXVal = rawAcX - zeroAcX;
  int accYVal = rawAcY - zeroAcY;
  int accZVal = rawAcZ - zeroAcZ;
  int AcX = RAD_TO_DEG * (atan2(accXVal, accZVal) + PI);
  int AcY = RAD_TO_DEG * (atan2(accYVal, accZVal) + PI);
  
  int gyroXRate = (rawGyX - zeroGyX) / (gySensitivity / 3.3 * 1023);
  int gyroYRate = (rawGyY - zeroGyY) / (gySensitivity / 3.3 * 1023);
  int dtime = millis() - prev_time_elpsd;
  GyX += gyroXRate * dtime / 1000;
  GyY += gyroYRate * dtime / 1000;

  angleX = 0.98 * (angleX + GyX * dtime) + 0.02 * AcX;
  angleY = 0.98 * (angleY + GyY * dtime) + 0.02 * AcY;
  
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.print(GyZ);
  Serial.print(" | angleX = "); Serial.print(angleX);
  Serial.print(" | angley = "); Serial.println(angleY);
  delay(333);
}
