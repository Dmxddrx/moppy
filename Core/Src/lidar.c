#include "lidar.h"
#include "vl53l0x.h"  /* Your custom library */
#include "oled.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

/* Create 4 independent sensor objects! */
static VL53L0X_Dev sensors[4];

/* * 1-to-1 Mapping to CubeMX User Labels:
 * Index 0 (Front) = XSHUT1
 * Index 1 (Right) = XSHUT2
 * Index 2 (Back)  = XSHUT3
 * Index 3 (Left)  = XSHUT4
 */
static GPIO_TypeDef * const xshut_port[4] = {
    XSHUT1_GPIO_Port, XSHUT2_GPIO_Port, XSHUT3_GPIO_Port, XSHUT4_GPIO_Port
};

static const uint16_t xshut_pin[4] = {
    XSHUT1_Pin, XSHUT2_Pin, XSHUT3_Pin, XSHUT4_Pin
};

/* STM32 HAL requires 8-bit addresses (default 0x29 shifted left is 0x52) */
#define VL53L0X_DEFAULT_ADDR 0x52

/* The custom addresses for your 4 sensors */
static const uint8_t new_addresses[4] = {
    (LIDAR_ADDR_0 << 1), (LIDAR_ADDR_1 << 1), (LIDAR_ADDR_2 << 1), (LIDAR_ADDR_3 << 1)
};

void LIDAR_Init(void)
{
    char buf[32];

    OLED_Clear();
    OLED_Print(0, 0, "Booting LiDAR...");
    OLED_Update();

    /* 1. Reset all sensors (pull XSHUT low to turn them OFF) */
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_RESET);
    }
    HAL_Delay(100); /* Wait 100ms to ensure they are fully dead */

    /* 2. Boot up and assign addresses one by one */
    for (int i = 0; i < 4; i++) {

        snprintf(buf, sizeof(buf), "LiDAR%d...", i+1);
        OLED_Print(0, 14 + (i*10), buf);
        OLED_Update();

        /* Turn ON one sensor */
        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_SET);
        HAL_Delay(50); /* Give it 50ms to fully wake up! */

        /* Point library to default factory address */
        sensors[i].i2c = &hi2c1;
        sensors[i].address = VL53L0X_DEFAULT_ADDR;
        sensors[i].timeout_ms = 100;

        /* Change the hardware address */
        VL53L0X_SetAddress(&sensors[i], new_addresses[i]);
        HAL_Delay(20); /* Give it time to apply the new address */

        /* Initialize the sensor at the NEW address */
        if (VL53L0X_Init(&sensors[i], &hi2c1, new_addresses[i]) == 1) {
            /* Format the original 7-bit address (e.g., 0x30) to print on the OLED */
            char addr_buf[16];
            snprintf(addr_buf, sizeof(addr_buf), "0x%02X", (new_addresses[i] >> 1));
            OLED_Print(80, 14 + (i*10), addr_buf);
        } else {
            OLED_Print(80, 14 + (i*10), "FAIL");
        }
        OLED_Update();
    }

    HAL_Delay(2000); /* Hold screen for 2 seconds so you can read it */
}

float LIDAR_GetDistance(uint8_t index)
{
    if (index >= 4) return -1.0f;

    /* (REMOVED the IsDeviceReady ping because it crashes the VL53L0X!) */

    /* 1. Pass the specific sensor object to the read function */
    uint16_t distance_mm = VL53L0X_ReadDistance(&sensors[index]);

    /* 2. Catch hardware timeouts inside the library */
    if (distance_mm == 65535) {
        return -2.0f; /* -2.0 means I2C disconnected / frozen! */
    }

    /* 3. Validation filter for open space */
    if (distance_mm > 2000 || distance_mm == 0) {
        return -1.0f; /* -1.0 means hardware is fine, just no wall detected */
    }

    return (float)distance_mm / 1000.0f;
}
