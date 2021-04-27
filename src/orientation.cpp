#include "Arduino.h"
#include "Orientation.h"

void Orientation::Update(double gyroRawX, double gyroRawY, double gyroRawZ, double dtseconds) { 
    localGyroRawX = gyroRawX; 
    localGyroRawY = gyroRawY; 
    localGyroRawZ = gyroRawZ; 

    // Setting the orientation axis'
    Qw[0] = 0; 
    Qw[1] = localGyroRawY; 
    Qw[2] = localGyroRawZ; 
    Qw[3] = localGyroRawX; 

    // Quaternion Integration
    Quat_dot[0] = (-0.5 * Quat_ori[1] * Qw[1] - 0.5 * Quat_ori[2] * Qw[2] - 0.5 * Quat_ori[3] * Qw[3]);
    Quat_dot[1] = (0.5 * Quat_ori[0] * Qw[1] + 0.5 * Quat_ori[2] * Qw[3] - 0.5 * Quat_ori[3] * Qw[2]);
    Quat_dot[2] = (0.5 * Quat_ori[0] * Qw[2] - 0.5 * Quat_ori[1] * Qw[3] + 0.5 * Quat_ori[3] * Qw[1]);
    Quat_dot[3] = (0.5 * Quat_ori[0] * Qw[3] + 0.5 * Quat_ori[1] * Qw[2] - 0.5 * Quat_ori[2] * Qw[1]);

    // Main Integration
    Quat_ori[0] = Quat_ori[0] + Quat_dot[0] * dtseconds;
    Quat_ori[1] = Quat_ori[1] + Quat_dot[1] * dtseconds;
    Quat_ori[2] = Quat_ori[2] + Quat_dot[2] * dtseconds;
    Quat_ori[3] = Quat_ori[3] + Quat_dot[3] * dtseconds;
  
    double quatNorm = sqrt(Quat_ori[0] * Quat_ori[0] + Quat_ori[1] * Quat_ori[1] + Quat_ori[2] * Quat_ori[2] + Quat_ori[3] * Quat_ori[3]);
  
    // Normalizing the orientation quaternion
    Quat_ori[0] = Quat_ori[0] / quatNorm;
    Quat_ori[1] = Quat_ori[1] / quatNorm;
    Quat_ori[2] = Quat_ori[2] / quatNorm;
    Quat_ori[3] = Quat_ori[3] / quatNorm;
}

void Orientation::toEuler() {
    double sinr_cosp = 2 * (Quat_ori[0] * Quat_ori[1] + Quat_ori[2] * Quat_ori[3]);
    double cosr_cosp = 1 - 2 * (Quat_ori[1] * Quat_ori[1] + Quat_ori[2] * Quat_ori[2]);
    Ax_RAD = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (Quat_ori[0] * Quat_ori[2] - Quat_ori[3] * Quat_ori[1]);
    if (abs(sinp) >= 1)
        Ay_RAD = copysign(PI / 2, sinp); 
    else
        Ay_RAD = asin(sinp);

    double siny_cosp = 2 * (Quat_ori[0] * Quat_ori[3] + Quat_ori[1] * Quat_ori[2]);
    double cosy_cosp = 1 - 2 * (Quat_ori[2] * Quat_ori[2] + Quat_ori[3] * Quat_ori[3]);
    Az_RAD = atan2(siny_cosp, cosy_cosp);

    Ax = (Ax_RAD * (180 / PI));
    Ay = ((Ay_RAD * (180 / PI)) * 1.035);
    Az = (Az_RAD * (180 / PI)); 
}

void Orientation::toRAD() {
  AxRAD = Ax * (PI/180);
  AyRAD = Ay * (PI/180);
  AzRAD = Az * (PI/180);
}
