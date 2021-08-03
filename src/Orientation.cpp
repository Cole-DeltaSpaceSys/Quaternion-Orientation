#include "Arduino.h"
#include "Orientation.h"
#include "Quaternion.h"
#include <math.h>

Quaternion quat;
Quaternion q;

void Orientation::update(double yaw, double pitch, double roll, double dt)
{
    quat.multiply(yaw, pitch, roll);
    quat.integrate(dt);
    quat.normalize();
}

void Orientation::toEuler()
{
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    yaw = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (q.w * q.y + -q.z * q.x);
    if (abs(sinp) >= 1)
        pitch = copysign(PI / 2, sinp);
    else
        pitch = asin(sinp);

    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    roll = atan2(siny_cosp, cosy_cosp);
}

float Orientation::toRadians(float x)
{
    float y = x * (PI / 180);
    return y;
}

float Orientation::toDegrees(float x)
{
    float y = x * (180 / PI);
    return y;
}
