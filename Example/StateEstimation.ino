#include "Orientation.h"
#include <Wire.h>

Orientation ori;

double dt, currentTime, previousTime;

struct gyroCalibration {
  double biasX, biasY, biasZ;
};
gyroCalibration gyroCal;

void setup() {
  Serial.begin(115200);
  //initialization();
  gyroBiasCompute();
}

void loop() {
  currentTime = micros();
  dt = ((currentTime - previousTime) / 1000000.);
  previousTime = currentTime;
    
  gyro.readSensor();
  ori.Update(/* GyroX */ - gyroCal.biasX, /* GyroY */ - gyroCal.biasY, /* GyroZ */ - gyroCal.biasZ, dt);
  ori.toEuler();
}

void gyroBiasCompute () {
  // Read initial data from gyroscopes
  gyro.readSensor();
    
  double gyroXsum = 0;
  double gyroYsum = 0;
  double gyroZsum = 0;
  
  // # of iterations - Increase for more prescise results
  const int calCount = 1600;

  for(int i = 0; i < calCount; i++)  { 
    while(!gyro.getDrdyStatus()) {}
        
    gyro.readSensor();  
    gyroXsum += /* GyroX */;
    gyroYsum += /* GyroY */;
    gyroZsum += /* GyroZ */;
  }
  gyroCal.biasX = gyroXsum / calCount;
  gyroCal.biasY = gyroYsum / calCount;
  gyroCal.biasZ = gyroZsum / calCount;
}
