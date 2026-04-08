#include "lidar.h"
#include "vl53l0x.h"  /* Your custom library */

extern I2C_HandleTypeDef hi2c1;

/* Create 4 independent sensor objects! */
static VL53L0X_Dev sensors[4];

static GPIO_TypeDef * const xshut_port[4] = {
    TRIG1_GPIO_Port, TRIG2_GPIO_Port, TRIG3_GPIO_Port, TRIG4_GPIO_Port
};
static const uint16_t xshut_pin[4] = {
    TRIG1_Pin, TRIG2_Pin, TRIG3_Pin, TRIG4_Pin
};

/* STM32 HAL requires 8-bit addresses (default 0x29 shifted left is 0x52) */
#define VL53L0X_DEFAULT_ADDR 0x52

/* The custom addresses for your 4 sensors */
static const uint8_t new_addresses[4] = {
    (LIDAR_ADDR_0 << 1), (LIDAR_ADDR_1 << 1), (LIDAR_ADDR_2 << 1), (LIDAR_ADDR_3 << 1)
};

void LIDAR_Init(void)
{
    /* 1. Reset all sensors (pull XSHUT low) */
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_RESET);
    }
    HAL_Delay(20);

    /* 2. Boot up and assign addresses */
    for (int i = 0; i < 4; i++) {

        /* Turn ON one sensor */
        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_SET);
        HAL_Delay(20);

        /* Tell the object it is temporarily at the default factory address */
        sensors[i].i2c = &hi2c1;
        sensors[i].address = VL53L0X_DEFAULT_ADDR;
        sensors[i].timeout_ms = 100;

        /* Change the hardware address */
        VL53L0X_SetAddress(&sensors[i], new_addresses[i]);

        /* Run the massive calibration/boot sequence at the new address! */
        VL53L0X_Init(&sensors[i], &hi2c1, new_addresses[i]);
    }
}

float LIDAR_GetDistance(uint8_t index)
{
    if (index >= 4) return -1.0f;

    /* Pass the specific sensor object to the read function */
    uint16_t distance_mm = VL53L0X_ReadDistance(&sensors[index]);

    /* Validation filter */
    if (distance_mm > 2000 || distance_mm == 65535 || distance_mm == 0) {
        return -1.0f; /* No wall detected / Timeout */
    }

    return (float)distance_mm / 1000.0f;
}
