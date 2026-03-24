#ifndef STABLE_H
#define STABLE_H

#include <mpu6050.h>
#include "hmc5883l.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;

} Orientation;

void STABLE_Init(void);

void STABLE_Update(MPU6500_RawData *imu,
                   HMC5883L_RawData *mag,
                   float dt);

Orientation STABLE_GetOrientation(void);

#endif
