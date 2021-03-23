#include <Wire.h>
#include <math.h>
#include "BMI088.h"

Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

struct Bias {
  float gyroX, gyroY, gyroZ;
};
Bias bias;

const double gravitationalConstant = 9.80665;

struct localOri {
  // Orientation Axis'
  double Qw[4];

  // Local Angular Velocity(Radians)
  double GyroRawX, GyroRawY, GyroRawZ;

  double velX, velY, velZ;

  double posX, posY, posZ;
};
localOri local;

struct globalOri {
  // Global Orientation in radians
  double AxRAD, AyRAD, AzRAD;

  // Main Integration
  double Quat_ori[4] = {1, 0, 0, 0};

  //Euler Integration
  double Quat_dot[4];

  // Global Orientation in degrees
  double Ax, Ay, Az;

  double Ax2, Ay2, Az2;
};
globalOri global;

struct oriVector {
  double oriquat[4];
};
oriVector vector;

struct gyroCalibration {
  double Ax, Ay, Az;
};
gyroCalibration gyroCal;

struct Time {
  double dtseconds, dtmillis, currentTime, currentTime_2, previousTime;
};
Time time;

enum FlightState {
  PAD_IDLE = 0,
  COMPUTE_DATA = 1
};
FlightState flightState = PAD_IDLE;

void setup () {
  initialization();
  gyroBiasCompute();
}

void loop () {  
  time.currentTime = millis();
  time.dtseconds = ((double) (time.currentTime - time.previousTime) / 1000);
  accel.readSensor();
  gyro.readSensor();
  quaternion();
  accelerationIntegration();
  gyroOffset();
  launchDetect();
  
  logToSerial();
  time.previousTime = time.currentTime;  
}

void launchDetect () {
  if ((flightState == PAD_IDLE) && accel.getAccelX_mss() > 0) {
    flightState = COMPUTE_DATA;
  }
}

void accelerationIntegration () {
  local.velX = local.velX + (accel.getAccelX_mss() - gravitationalConstant) * (time.dtseconds);
  local.posX = local.posX + local.velX * (time.dtseconds);
}


void quaternion () {
     local.GyroRawX = gyro.getGyroX_rads() - bias.gyroX; 
    local.GyroRawY = gyro.getGyroY_rads() - bias.gyroY; 
    local.GyroRawZ = gyro.getGyroZ_rads() - bias.gyroZ; 

    // Setting the orientation axis'
    local.Qw[0] = 0; 
    local.Qw[1] = gyroCal.Ay; 
    local.Qw[2] = gyroCal.Az; 
    local.Qw[3] = gyroCal.Ax; 

    // Quaternion Integration
    global.Quat_dot[0] = (-0.5 * global.Quat_ori[1] * local.Qw[1] - 0.5 * global.Quat_ori[2] * local.Qw[2] - 0.5 * global.Quat_ori[3] * local.Qw[3]);
    global.Quat_dot[1] = (0.5 * global.Quat_ori[0] * local.Qw[1] + 0.5 * global.Quat_ori[2] * local.Qw[3] - 0.5 * global.Quat_ori[3] * local.Qw[2]);
    global.Quat_dot[2] = (0.5 * global.Quat_ori[0] * local.Qw[2] - 0.5 * global.Quat_ori[1] * local.Qw[3] + 0.5 * global.Quat_ori[3] * local.Qw[1]);
    global.Quat_dot[3] = (0.5 * global.Quat_ori[0] * local.Qw[3] + 0.5 * global.Quat_ori[1] * local.Qw[2] - 0.5 * global.Quat_ori[2] * local.Qw[1]);

    // Main Integration
    global.Quat_ori[0] = global.Quat_ori[0] + global.Quat_dot[0] * time.dtseconds;
    global.Quat_ori[1] = global.Quat_ori[1] + global.Quat_dot[1] * time.dtseconds;
    global.Quat_ori[2] = global.Quat_ori[2] + global.Quat_dot[2] * time.dtseconds;
    global.Quat_ori[3] = global.Quat_ori[3] + global.Quat_dot[3] * time.dtseconds;
  
    double quatNorm = sqrt(global.Quat_ori[0] * global.Quat_ori[0] + global.Quat_ori[1] * global.Quat_ori[1] + global.Quat_ori[2] * global.Quat_ori[2] + global.Quat_ori[3] * global.Quat_ori[3]);
  
    // Normalizing the orientation quaternion
    global.Quat_ori[0] = global.Quat_ori[0] / quatNorm;
    global.Quat_ori[1] = global.Quat_ori[1] / quatNorm;
    global.Quat_ori[2] = global.Quat_ori[2] / quatNorm;
    global.Quat_ori[3] = global.Quat_ori[3] / quatNorm;

    // Converting from quaternion to euler angles through a rotation matrix
    global.AxRAD = atan((2 * (global.Quat_ori[0] * global.Quat_ori[1] + global.Quat_ori[2] * global.Quat_ori[3])) / (1 - 2 * (sq(global.Quat_ori[1]) + sq(global.Quat_ori[2]))));
    global.AyRAD = atan(2 * (global.Quat_ori[0] * global.Quat_ori[2] - global.Quat_ori[3] * global.Quat_ori[1]));
    global.AzRAD = atan((2 * (global.Quat_ori[0] * global.Quat_ori[3] + global.Quat_ori[1] * global.Quat_ori[2])) / ( 1 - 2 * (sq(global.Quat_ori[2]) + sq(global.Quat_ori[3]))));

    global.Ax = (double) (global.AxRAD * (180 / PI));
    global.Ay = (double) ((global.AyRAD * (180 / PI)) * 2);
    global.Az = (double) global.AzRAD * (180 / PI); 
}


void initialization () {
  bool status;
    // Checking to see if the Teensy can communicate with the BMI088 accelerometer
    status = accel.begin();
    status = accel.setOdr(Bmi088Accel::ODR_100HZ_BW_10HZ);
    status = accel.setRange(Bmi088Accel::RANGE_12G);
      if (status < 0) {
        Serial.println("Accel Initialization Error");
        while (1) {}
  }
    // Checking to see if the Teensy can communicate with the BMI088 gyroscopes
    status = gyro.begin();
    status = gyro.setOdr(Bmi088Gyro::ODR_200HZ_BW_23HZ);
    status = gyro.setRange(Bmi088Gyro::RANGE_250DPS);
      if (status < 0) {
        Serial.println("Gyro Initialization Error");
        while (1) {}
  }
}


void logToSerial () {
  Serial.print(accel.getAccelX_mss());
  Serial.print("\t");
  Serial.print(local.velX);
  Serial.print("\t");
  Serial.print(local.posX);
  Serial.print("\t");
  Serial.print(global.Ax);
  Serial.print("\t");
  Serial.print(global.Ay);
  Serial.print("\t");
  Serial.print(global.Az);
  Serial.print("\n");
}

void gyroOffset () {
  if (flightState == PAD_IDLE) {
    local.GyroRawX = global.Ax2;
    local.GyroRawY = global.Ay2;
    local.GyroRawZ = global.Az2;
  }
  gyroCal.Ax = local.GyroRawX - global.Ax2;
  gyroCal.Ay = local.GyroRawY - global.Ay2;
  gyroCal.Az = local.GyroRawZ - global.Az2;
}

void gyroBiasCompute () {
  // Read initial data from gyroscopes
  gyro.readSensor();
    
  float gyroXsum = 0;
  float gyroYsum = 0;
  float gyroZsum = 0;
  
  // # of iterations - Increase for more prescise results
  const int calCount = 600;

  for(int i = 0; i < calCount; i++)  {
    while(!gyro.getDrdyStatus()) {}
        
    gyro.readSensor();  
    gyroXsum += gyro.getGyroX_rads();
    gyroYsum += gyro.getGyroY_rads();
    gyroZsum += gyro.getGyroZ_rads();
  }
  bias.gyroX = gyroXsum / calCount;
  bias.gyroY = gyroYsum / calCount;
  bias.gyroZ = gyroZsum / calCount;
}
