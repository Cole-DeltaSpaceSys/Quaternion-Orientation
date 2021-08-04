#include "Arduino.h"
#include "Quaternion.h"
#include <math.h>

#define minNorm 1e-12

Quaternion Quaternion::toAxis(double rawZ, double rawY, double rawX, double dt)
{
    this->rawX = rawX;
    this->rawY = rawY;
    this->rawZ = rawZ;

    // Calculating the norm
    double quatNorm = sqrt(sq(rawX) + sq(rawY) + sq(rawZ));
    quatNorm = max(abs(quatNorm), minNorm);

    double theta = quatNorm * dt;

    Quaternion q;
    q.w = cos(theta / 2);
    q.x = -(rawX / quatNorm) * sin(theta / 2);
    q.y = -(rawY / quatNorm) * sin(theta / 2);
    q.z = -(rawZ / quatNorm) * sin(theta / 2);

    return q;
}

Quaternion Quaternion::multiply()
{
    double prevW, prevX, prevY, prevZ;

    Quaternion r;
    prevW = r.w;
    prevX = r.x;
    prevY = r.y;
    prevZ = r.z;

    // Quaternion Multiplication
    r.w = (w * prevW) - (x * prevX) - (y * prevY) - (z * prevZ);
    r.x = (w * prevX) + (x * prevW) + (y * prevZ) - (z * prevY);
    r.y = (w * prevY) - (x * prevZ) + (y * prevW) + (z * prevX);
    r.z = (w * prevZ) + (x * prevY) - (y * prevX) + (z * prevW);

    return r;
}

float Quaternion::Sq(float x)
{
    float y = x * x;
    return y;
}
