#ifndef STABLE_H
#define STABLE_H

#include "mpu6050.h"
#include "hmc5883l.h"

/* Holds the final, perfect 3D orientation of the robot */
typedef struct {
    float pitch; /* Nose up / down slope (Degrees) */
    float roll;  /* Leaning left / right (Degrees) */
    float yaw;   /* True North Compass Heading (Degrees) */
} Orientation;

void STABLE_Init(void);

/* Feed the raw sensor data here every 10ms */
void STABLE_Update(MPU6050_RawData* imu, HMC5883L_RawData* mag, float dt);

/* Ask for the cleaned data anywhere in your code */
Orientation STABLE_GetOrientation(void);

#endif /* STABLE_H */
