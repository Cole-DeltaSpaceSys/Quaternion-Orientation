#ifndef OrientationLib
#define OrientationLib

#include "Arduino.h"

class Orientation {
  public:
    void Update(double gyroRawX, double gyroRawY, double gyroRawZ, double dtseconds);
    void toEuler();
    void toQuaternion(double yaw, double pitch, double roll);
    void toRAD();

    double Ax, Ay, Az;   
    double AxRAD, AyRAD, AzRAD;
    double q[4] = {1, 0, 0, 0};

    private:
    double Qw[4] = {1, 0, 0, 0};
    double Quat_ori[4] = {1, 0, 0, 0};
    double Quat_dot[4] = {1, 0, 0, 0}; 
    double localGyroRawX, localGyroRawY, localGyroRawZ;
    double Ax_RAD, Ay_RAD, Az_RAD;
};

#endif 
