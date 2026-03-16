#include "general.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;   /* MPU6500 + HMC5883L */
extern I2C_HandleTypeDef hi2c2;   /* OLED    			*/

static MPU6500_RawData  imu_raw;
static HMC5883L_RawData mag_raw;

static MPU_Status mpu_status;
static HMC_Status hmc_status;

#if ENABLE_OLED_SELFTEST
	static HMC5883L_SelfTestData st_result;

	static uint8_t boot_done = 0;
	static uint32_t boot_time = 0;
#endif


	/* ----------------------------------------------------------------
	   Page management
	   ---------------------------------------------------------------- */
	typedef enum {
	    PAGE_SENSORS    = 0,
	    PAGE_ULTRASONIC = 1,
	    PAGE_COUNT      = 2
	} OLED_Page;

	static OLED_Page current_page = PAGE_SENSORS;
	static OLED_Page last_page    = PAGE_COUNT;   /* force first draw */

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
    {
        HMC5883L_Init();

	#if ENABLE_OLED_SELFTEST
        st_result = HMC5883L_SelfTest();
    #endif/* run once, store result */

    }
    ULTRASONIC_Init();
    BTNS_Init();

    OLED_Init(&hi2c2);
    HAL_Delay(100);
    OLED_Clear();

#if ENABLE_OLED_SELFTEST
    /* Show self-test screen immediately */
    GENERAL_OLED_SelfTest();
    boot_time = HAL_GetTick();   /* start 3s timer */
#endif
}

/* ================================================================
   GENERAL_OLED_SelfTest — shown once at boot
   ================================================================ */
#if ENABLE_OLED_SELFTEST

void GENERAL_OLED_SelfTest(void)
{
    char line[32];
    OLED_Clear();

    OLED_Print(0, 0, "-- SELF TEST --");

    /* MPU status */
    if     (mpu_status == MPU_NO_I2C)   OLED_Print(0, 12, "MPU: NO I2C");
    else if(mpu_status == MPU_WRONG_ID) OLED_Print(0, 12, "MPU: WRONG ID");
    else                                OLED_Print(0, 12, "MPU: OK");

    /* HMC status + self-test values */
    if(hmc_status != HMC_OK)
    {
        if(hmc_status == HMC_NO_I2C)   OLED_Print(0, 22, "HMC: NO I2C");
        else                           OLED_Print(0, 22, "HMC: WRONG ID");
    }
    else
    {
        OLED_Print(0, 22, "HMC: OK");
        snprintf(line, sizeof(line), "X:%d", (int)st_result.mx);
        OLED_Print(0, 34, line);
        snprintf(line, sizeof(line), "Y:%d", (int)st_result.my);
        OLED_Print(45, 34, line);
        snprintf(line, sizeof(line), "Z:%d", (int)st_result.mz);
        OLED_Print(90, 34, line);

        /* pass/fail — expected 243-575 per axis */
        uint8_t pass = (st_result.mx > 243 && st_result.mx < 575) &&
                       (st_result.my > 243 && st_result.my < 575) &&
                       (st_result.mz > 243 && st_result.mz < 575);
        OLED_Print(0, 46, pass ? "HMC PASS" : "HMC FAIL");
    }

    OLED_Update();
}

#endif

/* ================================================================
   Page 0 — MPU + HMC sensor data
   ================================================================ */
static void GENERAL_OLED_Page_Sensors(void)
{
    char line[32];

    if     (mpu_status == MPU_NO_I2C)   OLED_Print(0, 0, "MPU:xI2C");
    else if(mpu_status == MPU_WRONG_ID) OLED_Print(0, 0, "MPU:xID");
    else                                OLED_Print(0, 0, "MPU:OK");

    if     (hmc_status == HMC_NO_I2C)   OLED_Print(0, 10, "HMC:xI2C");
    else if(hmc_status == HMC_WRONG_ID) OLED_Print(0, 10, "HMC:xID");
    else                                OLED_Print(0, 10, "HMC:OK");

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
}

/* ================================================================
   Page 1 — Ultrasonic raw pulse widths
   ================================================================ */
static void GENERAL_OLED_Page_Ultrasonic(void)
{
    char line[32];

    OLED_Print(0, 0, "-- ULTRASONIC --");

    snprintf(line, sizeof(line), "S1:%luus", ultrasonic[0].pulse_us);
    OLED_Print(0, 14, line);
    snprintf(line, sizeof(line), "S2:%luus", ultrasonic[1].pulse_us);
    OLED_Print(0, 24, line);
    snprintf(line, sizeof(line), "S3:%luus", ultrasonic[2].pulse_us);
    OLED_Print(0, 34, line);
    snprintf(line, sizeof(line), "S4:%luus", ultrasonic[3].pulse_us);
    OLED_Print(0, 44, line);
}


/* ================================================================
   GENERAL_OLED_Debug — MPU6500 raw values
   ================================================================ */
#if ENABLE_OLED_DEBUG

void GENERAL_OLED_Debug(void)
{
    /* Clear only on page change — avoids flicker every frame */
    if(current_page != last_page)
    {
        OLED_Clear();
        last_page = current_page;
    }

    switch(current_page)
    {
        case PAGE_SENSORS:    GENERAL_OLED_Page_Sensors();    break;
        case PAGE_ULTRASONIC: GENERAL_OLED_Page_Ultrasonic(); break;
        default: break;
    }

    OLED_Update();
}

#endif

/* ================================================================
   GENERAL_Update — call in main while(1)
   ================================================================ */
void GENERAL_Update(void)
{
#if ENABLE_OLED_SELFTEST
    if(!boot_done)
    {
        if(HAL_GetTick() - boot_time >= 3000)
            boot_done = 1;
        else
            return;   /* hold self-test screen for 3s */
    }
#endif

    if(mpu_status == MPU_OK)
        MPU6500_ReadRaw(&imu_raw);

    if(hmc_status == HMC_OK)
        HMC5883L_ReadRaw(&mag_raw);

    /* Trigger one sensor per 60ms cycle — avoids echo crosstalk */
    static uint8_t  us_index     = 0;
    static uint32_t us_last_tick = 0;

    if(HAL_GetTick() - us_last_tick >= 60)
    {
        ULTRASONIC_Trigger(us_index);
        us_index = (us_index + 1) % 4;
        us_last_tick = HAL_GetTick();
    }

    /* Button — check and advance page if pressed */
    BTNS_Update();
    if(BTNS_Get_OLEDPage() == BTN_PRESSED)
        current_page = (current_page + 1) % PAGE_COUNT;

#if ENABLE_OLED_DEBUG
    GENERAL_OLED_Debug();
#endif
}
