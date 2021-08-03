#ifndef QuaternionLib
#define QuaternionLib

#include "Arduino.h"
#include <math.h>

class Quaternion
{
public:
    double w = 1.0f;
    double x = 0.0f;
    double y = 0.0f;
    double z = 0.0f;

    Quaternion() {}
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
    Quaternion(const Quaternion &q) : w(q.w), x(q.x), y(q.y), z(q.z) {}
    Quaternion multiply(double yaw, double pitch, double roll);
    Quaternion integrate(double dt);
    Quaternion normalize();

    Quaternion operator*(float f)
    {
        Quaternion q;
        q.w = w * f;
        q.x = x * f;
        q.y = y * f;
        q.z = z * f;
        return q;
    }

    Quaternion operator/(float f)
    {
        return Quaternion(w / f, x / f, y / f, z / f);
    }

    Quaternion operator+=(Quaternion q)
    {
        Quaternion r;
        r.w += q.w;
        return r;
    }

private:
    static float invSqrt(float x);
    static float Sq(float x);
};

#endif
