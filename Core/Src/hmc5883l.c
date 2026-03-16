#include "hmc5883l.h"

#define CONFIG_A       0x00
#define CONFIG_B       0x01
#define MODE_REG       0x02
#define DATA_X_MSB     0x03

#define ID_REG_A    0x0A
#define ID_REG_B    0x0B
#define ID_REG_C    0x0C
#define ID_VAL_A    0x48   /* 'H' */
#define ID_VAL_B    0x34   /* '4' */
#define ID_VAL_C    0x33   /* '3' */

static I2C_HandleTypeDef *hmc_i2c;

/* ================================================================
   HMC5883L_Check
   Call this first — sets the I2C handle and verifies the chip.
   Reads identity registers 0x0A/0x0B/0x0C in one transaction.
   HMC5883L always returns 'H'(0x48), '4'(0x34), '3'(0x33).
   ================================================================ */
HMC_Status HMC5883L_Check(I2C_HandleTypeDef *hi2c)
{
    hmc_i2c = hi2c;

    uint8_t id[3] = {0};

    if(HAL_I2C_Mem_Read(hmc_i2c, HMC5883L_ADDR,
                        0x0A, 1, id, 3, 100) != HAL_OK)
        return HMC_NO_I2C;

    if(id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33)
        return HMC_WRONG_ID;

    return HMC_OK;
}

void HMC5883L_Init(void)
{
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
    uint8_t buf[6] = {0};

    if(HAL_I2C_Mem_Read(hmc_i2c, HMC5883L_ADDR,
                        DATA_X_MSB, 1, buf, 6, 100) != HAL_OK)
    {
        data->mx = 0; data->my = 0; data->mz = 0;
        return;
    }

    /* HMC register order is X, Z, Y — not X, Y, Z */
    data->mx = (int16_t)((buf[0] << 8) | buf[1]);
    data->mz = (int16_t)((buf[2] << 8) | buf[3]);
    data->my = (int16_t)((buf[4] << 8) | buf[5]);
}
