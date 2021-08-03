#ifndef OrientationLib
#define OrientationLib

#include "Arduino.h"
#include "Quaternion.h"
#include <math.h>

Quaternion orientation;

class Orientation
{
public:
  double yaw, pitch, roll;
  Orientation() { orientation = Quaternion(); }

  void update(double yaw, double pitch, double roll, double dt);
  void toEuler();
  float toRadians(float x);
  float toDegrees(float x);
};

#endif
