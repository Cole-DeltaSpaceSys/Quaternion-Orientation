#include <Wire.h>
#include "BMI088.h"
#include "Orientation.h"

Orientation ori;

// Accelerometer Register
Bmi088Accel accel(Wire, 0x18);

// Gyroscope Register
Bmi088Gyro gyro(Wire, 0x68);

uint64_t currentTime = 0;
uint64_t previousTime = 0;
double dt;

struct YPR
{
  double X, Y, Z;
};
YPR ypr;

void setup(void)
{
  Serial.begin(15200);

  bool status;
  status = accel.begin();
  status = accel.setOdr(Bmi088Accel::ODR_400HZ_BW_40HZ);
  status = accel.setRange(Bmi088Accel::RANGE_12G);
  if (status < 0)
  {
    Serial.println("Accel Initialization Error");
    while (1)
    {
    }
  }
  // Checking to see if the Teensy can communicate with the BMI088 gyroscopes
  status = gyro.begin();
  status = gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ);
  status = gyro.setRange(Bmi088Gyro::RANGE_500DPS);
  if (status < 0)
  {
    Serial.println("Gyro Initialization Error");
    while (1)
    {
    }
  }
}

void loop(void)
{
  currentTime = micros();
  dt = ((double)(currentTime - previousTime) / 1000000.);
  previousTime = currentTime;

  gyro.readSensor();

  ypr.X = (gyro.getGyroX_rads()); 
  ypr.Y = (gyro.getGyroY_rads()); 
  ypr.Z = (gyro.getGyroZ_rads()); 

  ori.update(ypr.Z, ypr.Y, ypr.X, dt);
  ori.toEuler();

  Serial.print(ori.toDegrees(ori.yaw));
  Serial.print(ori.toDegrees(ori.pitch));
  Serial.println(ori.toDegrees(ori.roll));
}
