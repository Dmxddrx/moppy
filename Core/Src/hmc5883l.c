#include "hmc5883l.h"

#define CONFIG_A       0x00
#define CONFIG_B       0x01
#define MODE_REG       0x02
#define DATA_X_MSB     0x03

static I2C_HandleTypeDef *hmc_i2c;

void HMC5883L_Init(I2C_HandleTypeDef *hi2c)
{
    hmc_i2c = hi2c;

    uint8_t data;

    /* 8 samples avg, 15Hz */
    data = 0x70;
    HAL_I2C_Mem_Write(hmc_i2c, HMC5883L_ADDR, CONFIG_A, 1, &data, 1, 100);

    /* Gain */
    data = 0x20;
    HAL_I2C_Mem_Write(hmc_i2c, HMC5883L_ADDR, CONFIG_B, 1, &data, 1, 100);

    /* Continuous measurement mode */
    data = 0x00;
    HAL_I2C_Mem_Write(hmc_i2c, HMC5883L_ADDR, MODE_REG, 1, &data, 1, 100);
}

void HMC5883L_ReadRaw(HMC5883L_RawData *data)
{
    uint8_t buf[6];

    HAL_I2C_Mem_Read(hmc_i2c, HMC5883L_ADDR, DATA_X_MSB, 1, buf, 6, 100);

    data->mx = (buf[0] << 8) | buf[1];
    data->mz = (buf[2] << 8) | buf[3];
    data->my = (buf[4] << 8) | buf[5];
}
