#include "mpu6500.h"

#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define CONFIG         0x1A
#define SMPLRT_DIV     0x19

static I2C_HandleTypeDef *mpu_i2c;

void MPU6500_Init(I2C_HandleTypeDef *hi2c)
{
    mpu_i2c = hi2c;
    uint8_t data;

    /* Wake up device */
    data = 0x00;
    if(HAL_I2C_Mem_Write(mpu_i2c, MPU6500_ADDR,
                         PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK)
        return;   // sensor not responding — skip rest of init

    HAL_Delay(10);

    /* Sample rate divider */
    data = 0x07;
    HAL_I2C_Mem_Write(mpu_i2c, MPU6500_ADDR, SMPLRT_DIV, 1, &data, 1, 100);

    /* DLPF config */
    data = 0x03;
    HAL_I2C_Mem_Write(mpu_i2c, MPU6500_ADDR, CONFIG, 1, &data, 1, 100);

    /* Gyro ±250 dps */
    data = 0x00;
    HAL_I2C_Mem_Write(mpu_i2c, MPU6500_ADDR, GYRO_CONFIG, 1, &data, 1, 100);

    /* Accel ±2g */
    data = 0x00;
    HAL_I2C_Mem_Write(mpu_i2c, MPU6500_ADDR, ACCEL_CONFIG, 1, &data, 1, 100);
}

void MPU6500_ReadRaw(MPU6500_RawData *data)
{
    uint8_t buf[14] = {0};

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        mpu_i2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

    if(status != HAL_OK)
    {
        /* I2C failed — sensor not connected or wrong address
           data stays zero, robot continues safely            */
        data->ax = 0; data->ay = 0; data->az = 0;
        data->gx = 0; data->gy = 0; data->gz = 0;
        return;
    }

    data->ax = (int16_t)((buf[0]  << 8) | buf[1]);
    data->ay = (int16_t)((buf[2]  << 8) | buf[3]);
    data->az = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6] buf[7] = temperature — skipped, correct */
    data->gx = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gy = (int16_t)((buf[10] << 8) | buf[11]);
    data->gz = (int16_t)((buf[12] << 8) | buf[13]);
}
