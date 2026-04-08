#ifndef VL53L0X_CUSTOM_H
#define VL53L0X_CUSTOM_H

#include "stm32f4xx_hal.h"

/* The Sensor Object Blueprint */
typedef struct {
    I2C_HandleTypeDef *i2c;      /* Which I2C bus it's on (e.g., &hi2c1) */
    uint8_t address;             /* The current 8-bit I2C address */
    uint8_t stop_variable;       /* Internal ST calibration data */
    uint16_t timeout_ms;         /* How long to wait before giving up */
} VL53L0X_Dev;

/* API Functions */
void     VL53L0X_SetAddress(VL53L0X_Dev *dev, uint8_t new_addr);
uint8_t  VL53L0X_Init(VL53L0X_Dev *dev, I2C_HandleTypeDef *hi2c, uint8_t initial_addr);
uint16_t VL53L0X_ReadDistance(VL53L0X_Dev *dev);

#endif /* VL53L0X_CUSTOM_H */
