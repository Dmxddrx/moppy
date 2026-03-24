#ifndef MPU6500_H
#define MPU6500_H

#include "main.h"
#include <stdint.h>

#define MPU6500_ADDR   (0x68 << 1)   /* AD0 = LOW   */

typedef enum {
    MPU_OK       = 0,
    MPU_NO_I2C   = 1,
    MPU_WRONG_ID = 2,
} MPU_Status;

/* Raw 16-bit values from sensor registers */
typedef struct {
    int16_t ax, ay, az;   /* accelerometer — bias already removed */
    int16_t gx, gy, gz;   /* gyroscope     — bias already removed */
} MPU6500_RawData;

/* Physical units
   Scale factors:  ±2 g  → 16384 LSB/g  → × (9.81/16384) = m/s²
                   ±250°/s → 131 LSB/(°/s) → × (1/131)   = °/s  */
typedef struct {
    float ax, ay, az;     /* m/s²   */
    float gx, gy, gz;     /* deg/s  */
} MPU6500_PhysData;

MPU_Status        MPU6500_Check(I2C_HandleTypeDef *hi2c);
void              MPU6500_Init(void);
void              MPU6500_CalibrateGyro(void);   /* call while stationary */
HAL_StatusTypeDef MPU6500_ReadRaw(MPU6500_RawData *data);
MPU6500_PhysData  MPU6500_GetPhysical(const MPU6500_RawData *raw);

#endif /* MPU6500_H */
