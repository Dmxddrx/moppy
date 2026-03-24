#include "hmc5883l.h"
#include <math.h>

/* I2C1: PB6 (SCL1), PB7 (SDA1) — shared with MPU6500            */
extern I2C_HandleTypeDef hi2c1;

/* ── Register map ───────────────────────────────────────────────*/
#define REG_CONFIG_A  0x00
#define REG_CONFIG_B  0x01
#define REG_MODE      0x02
#define REG_DATA_XH   0x03   /* X_H, X_L, Z_H, Z_L, Y_H, Y_L    */
#define REG_STATUS    0x09
#define REG_ID_A      0x0A   /* Should read 'H', '4', '3'         */

/* ─────────────────────────────────────────────────────────────── */
static HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, reg, 1, &val, 1, 10);
}

/* ─────────────────────────────────────────────────────────────── */
HMC_Status HMC5883L_Check(I2C_HandleTypeDef *hi2c)
{
    uint8_t id[3] = {0};
    if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR,
                          REG_ID_A, 1, id, 3, 100) != HAL_OK)
        return HMC_NO_I2C;
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3')
        return HMC_WRONG_ID;
    return HMC_OK;
}

/* ─────────────────────────────────────────────────────────────── */
void HMC5883L_Init(void)
{
    HAL_Delay(10);
    /* CRA: 8 samples avg, 15 Hz output, normal measurement       */
    WriteReg(REG_CONFIG_A, 0x70);
    /* CRB: gain = 1090 LSB/Gauss (default ±1.3 Ga range)         */
    WriteReg(REG_CONFIG_B, 0x20);
    /* Mode: continuous measurement                                */
    WriteReg(REG_MODE, 0x00);
    HAL_Delay(67);   /* wait one measurement cycle (15 Hz → ~67 ms) */
}

/* ─────────────────────────────────────────────────────────────── */
void HMC5883L_ReadRaw(HMC5883L_RawData *data)
{
    uint8_t buf[6] = {0};
    /* NOTE: HMC5883L data register order is X, Z, Y — not X, Y, Z */
    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR,
                     REG_DATA_XH, 1, buf, 6, 10);
    data->mx = (int16_t)((buf[0] << 8) | buf[1]);
    data->mz = (int16_t)((buf[2] << 8) | buf[3]);
    data->my = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ─────────────────────────────────────────────────────────────── */
/*  HMC5883L_GetHeading                                             */
/*  Accepts tilt-compensated mx2, my2 (calculated in stable.c).    */
/*  Returns 0–360 degrees, CW from magnetic North.                  */
/* ─────────────────────────────────────────────────────────────── */
float HMC5883L_GetHeading(float mx, float my)
{
    float heading = atan2f(-my, mx) * (180.0f / 3.14159265f);
    if (heading < 0.0f)    heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;
    return heading;
}

/* ─────────────────────────────────────────────────────────────── */
HMC5883L_SelfTestData HMC5883L_SelfTest(void)
{
    HMC5883L_SelfTestData result = {0};
    /* Enable positive bias self-test on X axis                    */
    WriteReg(REG_CONFIG_A, 0x71);  /* 8 samples, 15 Hz, positive bias */
    HAL_Delay(67);

    HMC5883L_RawData raw;
    HMC5883L_ReadRaw(&raw);
    result.mx = raw.mx;
    result.my = raw.my;
    result.mz = raw.mz;

    /* Restore normal operation                                    */
    HMC5883L_Init();
    return result;
}
