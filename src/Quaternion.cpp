#include "Arduino.h"
#include "Quaternion.h"
#include <math.h>

#define beta 0.1f

Quaternion Quaternion::multiply(double yaw, double pitch, double roll)
{
    Quaternion q;
    q.w = 0.5 * (-x * roll - 0.5 * y * pitch - 0.5 * z * yaw);
    q.x = 0.5 * (w * roll + 0.5 * y * yaw - 0.5 * z * pitch);
    q.y = 0.5 * (w * pitch - 0.5 * x * yaw + 0.5 * z * roll);
    q.z = 0.5 * (w * yaw + 0.5 * x * pitch - 0.5 * y * roll);
    return q;
}

Quaternion Quaternion::integrate(double dt)
{
    Quaternion r;
    r.w += w * dt;
    r.x += x * dt;
    r.y += y * dt;
    r.z += z * dt;

    return r;
}

Quaternion Quaternion::normalize()
{
    double norm = invSqrt(Sq(w) + Sq(x) + Sq(y) + Sq(z));
    norm = max(abs(norm), 1e-12);

    Quaternion a;
    a.w = w * norm;
    a.x = x * norm;
    a.y = y * norm;
    a.z = z * norm;
    return a;
}

float Quaternion::Sq(float x)
{
    float y = x * x;
    return y;
}

float Quaternion::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
