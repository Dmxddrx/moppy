#ifndef MPU6500_H
#define MPU6500_H

#include "main.h"
#include <stdint.h>

#define MPU6500_ADDR (0x68 << 1)

/* Raw sensor data structure */
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

} MPU6500_RawData;

void MPU6500_Init(I2C_HandleTypeDef *hi2c);
void MPU6500_ReadRaw(MPU6500_RawData *data);

#endif
