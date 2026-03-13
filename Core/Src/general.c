#include "general.h"
#include <stdio.h>
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;   /* MPU6500 */
extern I2C_HandleTypeDef hi2c2;   /* OLED    */

static MPU6500_RawData imu_raw;

/* ================================================================
   GENERAL_Init
   ================================================================ */
void GENERAL_Init(void)
{
    MPU6500_Init(&hi2c1);

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

    /* Connection status — AZ should be ~16384 when flat and connected */
    if(imu_raw.ax == 0 && imu_raw.ay == 0 &&
       imu_raw.az == 0 && imu_raw.gx == 0)
    {
        OLED_Print(0, 0, "MPU: NO SIGNAL");
    }
    else
    {
        OLED_Print(0, 0, "MPU: OK");
    }

    snprintf(line, sizeof(line), "AX: %d", (int)imu_raw.ax);
    OLED_Print(0, 11, line);

    snprintf(line, sizeof(line), "AY: %d", (int)imu_raw.ay);
    OLED_Print(0, 21, line);

    snprintf(line, sizeof(line), "AZ: %d", (int)imu_raw.az);
    OLED_Print(0, 31, line);

    snprintf(line, sizeof(line), "GX: %d", (int)imu_raw.gx);
    OLED_Print(0, 41, line);

    snprintf(line, sizeof(line), "GY: %d", (int)imu_raw.gy);
    OLED_Print(0, 51, line);

    OLED_Update();
}


/* ================================================================
   GENERAL_Update — call in main while(1)
   ================================================================ */
void GENERAL_Update(void)
{
    MPU6500_ReadRaw(&imu_raw);
    GENERAL_OLED_Debug();
}
