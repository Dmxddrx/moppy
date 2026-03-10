#include "stable.h"
#include <math.h>

static Orientation orientation;

static float gyro_scale = 131.0f;
static float acc_scale  = 16384.0f;

void STABLE_Init(void)
{
    orientation.roll = 0;
    orientation.pitch = 0;
    orientation.yaw = 0;
}

void STABLE_Update(MPU6500_RawData *imu,
                   HMC5883L_RawData *mag,
                   float dt)
{
    float ax = imu->ax / acc_scale;
    float ay = imu->ay / acc_scale;
    float az = imu->az / acc_scale;

    float gx = imu->gx / gyro_scale;
    float gy = imu->gy / gyro_scale;
    float gz = imu->gz / gyro_scale;

    /* accelerometer tilt */

    float acc_roll  = atan2f(ay, az) * 57.2958f;
    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;

    /* complementary filter */

    orientation.roll =
        0.98f*(orientation.roll + gx*dt) + 0.02f*acc_roll;

    orientation.pitch =
        0.98f*(orientation.pitch + gy*dt) + 0.02f*acc_pitch;

    /* magnetometer heading */

    float heading = atan2f(mag->my, mag->mx) * 57.2958f;

    if (heading < 0)
        heading += 360;

    orientation.yaw = heading;
}

Orientation STABLE_GetOrientation(void)
{
    return orientation;
}
