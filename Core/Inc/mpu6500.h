#ifndef MPU6500_H
#define MPU6500_H

#include "main.h"
#include <stdint.h>

#define MPU6500_ADDR (0x68 << 1)

typedef enum {
    MPU_OK      = 0,
    MPU_NO_I2C  = 1,   // HAL_I2C_Mem_Read failed
    MPU_WRONG_ID = 2,  // wrong WHO_AM_I value
} MPU_Status;

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

MPU_Status MPU6500_Check(I2C_HandleTypeDef *hi2c);
void MPU6500_Init(void);
void MPU6500_ReadRaw(MPU6500_RawData *data);

#endif
