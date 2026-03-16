#include "general.h"
#include <stdio.h>
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;   /* MPU6500 + HMC5883L */
extern I2C_HandleTypeDef hi2c2;   /* OLED    			*/

static MPU6500_RawData  imu_raw;
static HMC5883L_RawData mag_raw;

static MPU_Status mpu_status;
static HMC_Status hmc_status;

//static uint8_t          display_page = 0;   /* 0 = MPU, 1 = HMC */

/* ================================================================
   GENERAL_Init
   ================================================================ */
void GENERAL_Init(void)
{

    mpu_status = MPU6500_Check(&hi2c1);
    if(mpu_status == MPU_OK)
        MPU6500_Init();

    hmc_status = HMC5883L_Check(&hi2c1);
    if(hmc_status == HMC_OK)
        HMC5883L_Init();

    OLED_Init(&hi2c2);
    HAL_Delay(100);
    OLED_Clear();
}


/* ================================================================
   GENERAL_OLED_Debug — MPU6500 raw values
   ================================================================ */
void GENERAL_OLED_Debug(void)
{
    char line[32];

    OLED_Clear();

    if     (mpu_status == MPU_NO_I2C)   OLED_Print(0, 0, "MPU:xI2C");
    else if(mpu_status == MPU_WRONG_ID) OLED_Print(0, 0, "MPU:xID");
    else                                OLED_Print(0, 0, "MPU:OK");

    if     (hmc_status == HMC_NO_I2C)   OLED_Print(0, 10, "HMC:xI2C");
    else if(hmc_status == HMC_WRONG_ID) OLED_Print(0, 10, "HMC:xID");
    else                                OLED_Print(0, 10, "HMC:OK");

    //OLED_Rectangle(0,10,128,1);
    snprintf(line, sizeof(line), "MX:%d", (int)mag_raw.mx);
    OLED_Print(0, 26, line);
    snprintf(line, sizeof(line), "MY:%d", (int)mag_raw.my);
    OLED_Print(0, 36, line);
    snprintf(line, sizeof(line), "MZ:%d", (int)mag_raw.mz);
    OLED_Print(0, 46, line);

    snprintf(line, sizeof(line), "AX:%d", (int)imu_raw.ax);
    OLED_Print(60, 0, line);
    snprintf(line, sizeof(line), "AY:%d", (int)imu_raw.ay);
    OLED_Print(60, 10, line);
    snprintf(line, sizeof(line), "AZ:%d", (int)imu_raw.az);
    OLED_Print(60, 20, line);
    snprintf(line, sizeof(line), "GX:%d", (int)imu_raw.gx);
    OLED_Print(60, 30, line);
    snprintf(line, sizeof(line), "GY:%d", (int)imu_raw.gy);
    OLED_Print(60, 40, line);
    snprintf(line, sizeof(line), "GZ:%d", (int)imu_raw.gz);
    OLED_Print(60, 50, line);

    OLED_Update();
}


/* ================================================================
   GENERAL_Update — call in main while(1)
   ================================================================ */
void GENERAL_Update(void)
{
    if(mpu_status == MPU_OK)
        MPU6500_ReadRaw(&imu_raw);

    if(hmc_status == HMC_OK)
        HMC5883L_ReadRaw(&mag_raw);

    GENERAL_OLED_Debug();
}
