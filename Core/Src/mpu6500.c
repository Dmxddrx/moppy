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
    HAL_I2C_Mem_Write(mpu_i2c, MPU6500_ADDR, PWR_MGMT_1, 1, &data, 1, 100);

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
    uint8_t buf[14];

    HAL_I2C_Mem_Read(mpu_i2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

    data->ax = (buf[0] << 8) | buf[1];
    data->ay = (buf[2] << 8) | buf[3];
    data->az = (buf[4] << 8) | buf[5];

    data->gx = (buf[8] << 8) | buf[9];
    data->gy = (buf[10] << 8) | buf[11];
    data->gz = (buf[12] << 8) | buf[13];
}
