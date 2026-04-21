#include "lidar.h"
#include "vl53l0x.h"
#include "oled.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

static VL53L0X_Dev sensors[4];

static GPIO_TypeDef * const xshut_port[4] = {
    XSHUT1_GPIO_Port, XSHUT2_GPIO_Port, XSHUT3_GPIO_Port, XSHUT4_GPIO_Port
};

static const uint16_t xshut_pin[4] = {
    XSHUT1_Pin, XSHUT2_Pin, XSHUT3_Pin, XSHUT4_Pin
};

#define VL53L0X_DEFAULT_ADDR 0x52

static const uint8_t new_addresses[4] = {
    (LIDAR_ADDR_0 << 1), (LIDAR_ADDR_1 << 1), (LIDAR_ADDR_2 << 1), (LIDAR_ADDR_3 << 1)
};

/* --- FAST DATA SAMPLING ARRAYS --- */
/* Initialized to 8191 (Out of Range) so the robot doesn't phantom-brake on boot! */
static uint16_t s_lidar_hist[4][3] = {
    {8191, 8191, 8191},
    {8191, 8191, 8191},
    {8191, 8191, 8191},
    {8191, 8191, 8191}
};
static uint8_t s_hist_idx[4] = {0};

/* Ultra-fast hardware comparison macros */
#define FAST_MIN(x,y) ((x) < (y) ? (x) : (y))
#define FAST_MAX(x,y) ((x) > (y) ? (x) : (y))
/* -------------------------------------- */

void LIDAR_Init(void)
{
    char buf[32];

    OLED_Clear();
    OLED_Print(0, 0, "Booting LiDAR...");
    OLED_Update();

    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_RESET);
    }
    HAL_Delay(100);

    for (int i = 0; i < 4; i++) {
        snprintf(buf, sizeof(buf), "LiDAR%d...", i+1);
        OLED_Print(0, 14 + (i*10), buf);
        OLED_Update();

        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_SET);
        HAL_Delay(50);

        sensors[i].i2c = &hi2c1;
        sensors[i].address = VL53L0X_DEFAULT_ADDR;
        sensors[i].timeout_ms = 100;

        VL53L0X_SetAddress(&sensors[i], new_addresses[i]);
        HAL_Delay(20);

        if (VL53L0X_Init(&sensors[i], &hi2c1, new_addresses[i]) == 1) {
            char addr_buf[16];
            snprintf(addr_buf, sizeof(addr_buf), "0x%02X", (new_addresses[i] >> 1));
            OLED_Print(80, 14 + (i*10), addr_buf);
        } else {
            OLED_Print(80, 14 + (i*10), "FAIL");
        }
        OLED_Update();
    }

    HAL_Delay(2000);
}

/* ═══════════════════════════════════════════════════════════════ */
/* NEW: AUTOMATED HOT-REBOOT RECOVERY                              */
/* ═══════════════════════════════════════════════════════════════ */
static void LIDAR_RecoverSensor(uint8_t index)
{
    /* 1. Hard kill the sensor's power using XSHUT */
    HAL_GPIO_WritePin(xshut_port[index], xshut_pin[index], GPIO_PIN_RESET);
    HAL_Delay(5); /* Tiny delay to let capacitors drain */

    /* 2. Wake it back up */
    HAL_GPIO_WritePin(xshut_port[index], xshut_pin[index], GPIO_PIN_SET);
    HAL_Delay(10); /* Let the sensor boot up */

    /* 3. The sensor forgot its address! Point the library back to factory default */
    sensors[index].address = VL53L0X_DEFAULT_ADDR;

    /* 4. Reprogram the custom address */
    VL53L0X_SetAddress(&sensors[index], new_addresses[index]);
    HAL_Delay(5);

    /* 5. Re-initialize the base settings */
    VL53L0X_Init(&sensors[index], &hi2c1, new_addresses[index]);
}

/* ═══════════════════════════════════════════════════════════════ */
/* Reads the hardware, filters the noise, and handles crashes      */
/* ═══════════════════════════════════════════════════════════════ */
uint16_t LIDAR_GetFilteredDistance(uint8_t index)
{
    if (index >= 4) return 65535;

    /* 1. Read the raw hardware value */
    uint16_t raw_val = VL53L0X_ReadDistance(&sensors[index]);

    /* 2. FAULT DETECTION: Did the sensor crash from motor noise? */
    if (raw_val == 65535)
    {
        /* Execute emergency hot-reboot! */
        LIDAR_RecoverSensor(index);

        /* Return the LAST known good value from the history array so the robot doesn't panic while recovering */
        raw_val = s_lidar_hist[index][(s_hist_idx[index] + 2) % 3];
    }

    /* 3. SAMPLING: Store it in the rotating history array */
    s_lidar_hist[index][s_hist_idx[index]] = raw_val;
    s_hist_idx[index] = (s_hist_idx[index] + 1) % 3;

    /* 4. Grab the last 3 samples */
    uint16_t a = s_lidar_hist[index][0];
    uint16_t b = s_lidar_hist[index][1];
    uint16_t c = s_lidar_hist[index][2];

    /* 5. Calculate and return the Median */
    return FAST_MAX(FAST_MIN(a,b), FAST_MIN(FAST_MAX(a,b),c));
}
