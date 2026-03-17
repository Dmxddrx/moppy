#include "general.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;   /* MPU6500 + HMC5883L */
extern I2C_HandleTypeDef hi2c2;   /* OLED    			*/

extern TIM_HandleTypeDef htim6;

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
		PAGE_ENCODERS   = 2,
	    PAGE_COUNT      = 3
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

    HAL_TIM_Base_Start_IT(&htim6);   /* starts 1ms tick for ENCODER_Update */
    ENCODER_Init();
    MOTOR_Init();

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

    snprintf(line, sizeof(line), "MX:%-6d", (int)mag_raw.mx);
    OLED_Print(0, 26, line);
    snprintf(line, sizeof(line), "MY:%-6d", (int)mag_raw.my);
    OLED_Print(0, 36, line);
    snprintf(line, sizeof(line), "MZ:%-6d", (int)mag_raw.mz);
    OLED_Print(0, 46, line);

    snprintf(line, sizeof(line), "AX:%-6d", (int)imu_raw.ax);
    OLED_Print(60, 0, line);
    snprintf(line, sizeof(line), "AY:%-6d", (int)imu_raw.ay);
    OLED_Print(60, 10, line);
    snprintf(line, sizeof(line), "AZ:%-6d", (int)imu_raw.az);
    OLED_Print(60, 20, line);
    snprintf(line, sizeof(line), "GX:%-6d", (int)imu_raw.gx);
    OLED_Print(60, 30, line);
    snprintf(line, sizeof(line), "GY:%-6d", (int)imu_raw.gy);
    OLED_Print(60, 40, line);
    snprintf(line, sizeof(line), "GZ:%-6d", (int)imu_raw.gz);
    OLED_Print(60, 50, line);
}

/* ================================================================
   Page 1 — Ultrasonic raw pulse widths
   ================================================================ */
static void GENERAL_OLED_Page_Ultrasonic(void)
{
    char line[32];

    /* Status line — all 4 in one row */
    char ok_line[32]  = "S";
    char no_line[32]  = "S";

    for(uint8_t i = 0; i < 4; i++)
    {
        char tag[3];
        snprintf(tag, sizeof(tag), "%d", i+1);
        if(ultrasonic[i].status == US_OK)
            strncat(ok_line, tag, sizeof(ok_line) - strlen(ok_line) - 1);
        else
            strncat(no_line, tag, sizeof(no_line) - strlen(no_line) - 1);
    }

    if(strlen(ok_line) > 2) strncat(ok_line, ":OK", sizeof(ok_line) - strlen(ok_line) - 1);
    else                     strncpy(ok_line, "        ", sizeof(ok_line));  /* blank if none */

    if(strlen(no_line) > 2) strncat(no_line, ":NO", sizeof(no_line) - strlen(no_line) - 1);
    else                     strncpy(no_line, "        ", sizeof(no_line));  /* blank if none */

    OLED_Print(0,  0, ok_line);   /* "S 123:OK" — left  column */
    OLED_Print(65, 0, no_line);   /* "S 4:NO"  — right column */


    /* Values */
    if(ultrasonic[0].status == US_NO_ECHO) OLED_Print(0, 16, "S1:NO ECHO    ");
    else { snprintf(line, sizeof(line), "S1:%-8lu", ultrasonic[0].pulse_us); OLED_Print(0, 16, line); }

    if(ultrasonic[1].status == US_NO_ECHO) OLED_Print(0, 26, "S2:NO ECHO    ");
    else { snprintf(line, sizeof(line), "S2:%-8lu", ultrasonic[1].pulse_us); OLED_Print(0, 26, line); }

    if(ultrasonic[2].status == US_NO_ECHO) OLED_Print(0, 36, "S3:NO ECHO    ");
    else { snprintf(line, sizeof(line), "S3:%-8lu", ultrasonic[2].pulse_us); OLED_Print(0, 36, line); }

    if(ultrasonic[3].status == US_NO_ECHO) OLED_Print(0, 46, "S4:NO ECHO    ");
    else { snprintf(line, sizeof(line), "S4:%-8lu", ultrasonic[3].pulse_us); OLED_Print(0, 46, line); }

}

/* ================================================================
   Page 2 — Encoder counts and speed
   ================================================================ */
static void GENERAL_OLED_Page_Encoders(void)
{
    char line[32];

    /* Status */
    if(ENCODER_GetStatus(0) == ENC_NO_SIGNAL) OLED_Print(0, 0, "E1:NO");
    else                                       OLED_Print(0, 0, "E1:OK");

    if(ENCODER_GetStatus(1) == ENC_NO_SIGNAL) OLED_Print(0, 10, "E2:NO");
    else 										OLED_Print(0, 10, "E2:OK");

    snprintf(line, sizeof(line), "E1 CNT:%-8ld", (long)ENCODER_GetCount(0));
    OLED_Print(0, 23, line);
    snprintf(line, sizeof(line), "E1 SPD:%-8.0f", ENCODER_GetSpeed(0));
    OLED_Print(0, 33, line);

    snprintf(line, sizeof(line), "E2 CNT:%-8ld", (long)ENCODER_GetCount(1));
    OLED_Print(0, 43, line);
    snprintf(line, sizeof(line), "E2 SPD:%-8.0f", ENCODER_GetSpeed(1));
    OLED_Print(0, 53, line);
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
        case PAGE_ENCODERS:   GENERAL_OLED_Page_Encoders();   break;
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

    /* ============================================================
       MOTOR CONTROL — add here, sensors already fresh above
       ============================================================ */
    MOTOR_Set(0, MOTOR_FORWARD,  500);   /* M1 forward  50% */
    MOTOR_Set(1, MOTOR_BACKWARD, 500);   /* M2 backward 75% */
    MOTOR_Set(2, MOTOR_STOP,     500);     /* M3 stop         */
    MOTOR_Set(3, MOTOR_FORWARD,  500);   /* M4 forward 100% */
    MOTOR_Set(4, MOTOR_FORWARD,  500);   /* M5 forward  50% */
    MOTOR_Set(5, MOTOR_BACKWARD, 500);   /* M6 backward 50% */
    /* ============================================================ */

    /* Button — check and advance page if pressed */
    BTNS_Update();
    if(BTNS_Get_OLEDPage() == BTN_PRESSED)
        current_page = (current_page + 1) % PAGE_COUNT;

#if ENABLE_OLED_DEBUG
    GENERAL_OLED_Debug();
#endif
}
