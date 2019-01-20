#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Servo.h>

Servo servoX; // Servo B
Servo servoY; // Servo A

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

int angleX;
int angleY;

float i = 0;
float alt = 0;

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = 1023;
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();

  servoX.attach(4);
  servoY.attach(7);

  servoX.write(90);
  servoY.write(90);
}

void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Roll: "));
    angleX = orientation.roll;
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    angleY = orientation.pitch;
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
  }
  
  /* Calculate the altitude using the barometric pressure sensor */
//  bmp.getEvent(&bmp_event);
//  if (bmp_event.pressure)
//  {
//    /* Get ambient temperature in C */
//    float temperature;
//    bmp.getTemperature(&temperature);
//    /* Convert atmospheric pressure, SLP and temp to altitude    */
//    Serial.print(F("Alt: "));
//    float raw_alt = bmp.pressureToAltitude(seaLevelPressure,
//                                        bmp_event.pressure,
//                                        temperature) * 3.28084;
//    if(millis() < 500) {
//        i = raw_alt;
//    }
//
//    alt = raw_alt - i;
//    Serial.print(alt);
//    
//    Serial.print(F(" feet; "));
//  }
//
//  if(alt >= 10 and millis() > 1000) {
//      servoX.write(0);
//      servoY.write(0);
//      while(1);
//  }

  if(angleX + 90 <= 180 and angleX + 90 >= 0) {
      servoX.write(angleX + 90);
  }
  if(angleY + 90 <= 180 and angleY + 90 >= 0) {
      servoY.write(angleY + 90);
  }
  
  Serial.print(millis());
  Serial.println(F(""));
}
