#ifndef OrientationLib
#define OrientationLib

#include "Arduino.h"

class Orientation {
  public:
    void Update(double gyroRawX, double gyroRawY, double gyroRawZ, double dtseconds);
    void toEuler();
    void toRAD();

    double Ax, Ay, Az;   
    double localGyroRawX, localGyroRawY, localGyroRawZ;
    double Qw[4];
    double Ax_RAD, Ay_RAD, Az_RAD;
    double AxRAD, AyRAD, AzRAD;
    double Quat_ori[4] = {1, 0, 0, 0};
    double Quat_dot[4];   
};

#endif 
