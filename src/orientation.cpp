#include "Arduino.h"
#include "Orientation.h"

void Orientation::Update(double gyroRawX, double gyroRawY, double gyroRawZ, double dtseconds) { 
    localGyroRawX = gyroRawX; 
    localGyroRawY = gyroRawY; 
    localGyroRawZ = gyroRawZ; 

    // Setting the orientation axis'
    Qw[0] = 0; 
    Qw[1] = localGyroRawX; 
    Qw[2] = localGyroRawY; 
    Qw[3] = localGyroRawZ; 

    // Quaternion Integration
    Quat_dot[0] = (Quat_ori[0] * Qw[0]) - (Quat_ori[1] * Qw[1]) - (Quat_ori[2] * Qw[2]) - (Quat_ori[3] * Qw[3]);
    Quat_dot[1] = (Quat_ori[0] * Qw[1]) + (Quat_ori[1] * Qw[0]) + (Quat_ori[2] * Qw[3]) - (Quat_ori[3] * Qw[2]);
    Quat_dot[2] = (Quat_ori[0] * Qw[2]) - (Quat_ori[1] * Qw[3]) + (Quat_ori[2] * Qw[0]) + (Quat_ori[3] * Qw[1]);
    Quat_dot[3] = (Quat_ori[0] * Qw[3]) + (Quat_ori[1] * Qw[2]) - (Quat_ori[2] * Qw[1]) + (Quat_ori[3] * Qw[0]);
  
    double quatNorm = sqrt(sq(Quat_ori[0]) + sq(Quat_ori[1]) + sq(Quat_ori[2]) + sq(Quat_ori[3]));

    double theta = sin((quatNorm * dtseconds) / 2);
    
    // Normalizing the orientation quaternion
    Quat_ori[0] += Quat_dot[0] * ((quatNorm * dtseconds) / 2);
    Quat_ori[1] += Quat_dot[1] * theta;
    Quat_ori[2] += Quat_dot[2] * theta;
    Quat_ori[3] += Quat_dot[3] * theta;
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
    Ay = (Ay_RAD * (180 / PI)) * (1.035);
    Az = (Az_RAD * (180 / PI)); 
}

void Orientation::toQuaternion(double yaw, double pitch, double roll) {
    double cy = cos(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sy = sin(yaw * 0.5);
    double sp = sin(pitch * 0.5);
    double sr = sin(roll * 0.5);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

void Orientation::toRAD() {
    AxRAD = Ax * (PI / 180);
    AyRAD = Ay * (PI / 180);
    AzRAD = Az * (PI / 180);
}
