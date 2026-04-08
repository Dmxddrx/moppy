#include "lidar.h"

/* INCLUDE YOUR DOWNLOADED LIBRARY HERE: */
// #include "vl53l0x_api.h"

extern I2C_HandleTypeDef hi2c1;

/* Reusing your Trigger pins as XSHUT (Shutdown) pins */
static GPIO_TypeDef * const xshut_port[4] = {
    TRIG1_GPIO_Port, TRIG2_GPIO_Port, TRIG3_GPIO_Port, TRIG4_GPIO_Port
};
static const uint16_t xshut_pin[4] = {
    TRIG1_Pin, TRIG2_Pin, TRIG3_Pin, TRIG4_Pin
};

/* The new addresses we want to assign */
static const uint8_t new_addresses[4] = {
    LIDAR_ADDR_0, LIDAR_ADDR_1, LIDAR_ADDR_2, LIDAR_ADDR_3
};

/* ═══════════════════════════════════════════════════════════════ */
/* LIDAR_Init: The Boot & Address Assignment Sequence             */
/* ═══════════════════════════════════════════════════════════════ */
void LIDAR_Init(void)
{
    /* 1. Reset ALL sensors by pulling XSHUT pins LOW */
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_RESET);
    }
    HAL_Delay(20); /* Give them time to turn off completely */

    /* 2. Wake them up ONE BY ONE and change their I2C address */
    for (int i = 0; i < 4; i++) {

        /* Turn on ONE sensor */
        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_SET);
        HAL_Delay(20); /* Wait for the sensor to boot up */

        /* -------------------------------------------------------------
           LIBRARY SPECIFIC CODE BELOW
           Replace these with the actual functions from your library!
           ------------------------------------------------------------- */

        /* Tell the sensor at the default address (0x29) to change to the new address */
        // VL53L0X_ChangeAddress(&hi2c1, 0x29, new_addresses[i]);

        /* Initialize the sensor now that it is listening on its new address */
        // VL53L0X_InitSensor(&hi2c1, new_addresses[i]);
    }
}

/* ═══════════════════════════════════════════════════════════════ */
/* LIDAR_GetDistance: Read the sensor and return meters           */
/* ═══════════════════════════════════════════════════════════════ */
float LIDAR_GetDistance(uint8_t index)
{
    if (index >= 4) return -1.0f;

    uint16_t distance_mm = 0;

    /* -------------------------------------------------------------
       LIBRARY SPECIFIC CODE BELOW
       Replace this with the actual read function from your library!
       ------------------------------------------------------------- */

    // distance_mm = VL53L0X_ReadDistance(&hi2c1, new_addresses[index]);

    /* Validation filter: VL53L0X usually returns 8190 or 8191 if
       it timeouts or doesn't see anything (Out of Range) */
    if (distance_mm > 2000 || distance_mm == 0) {
        return -1.0f; /* No wall detected */
    }

    /* Convert millimeters back to meters to match the rest of your robot code! */
    return (float)distance_mm / 1000.0f;
}
