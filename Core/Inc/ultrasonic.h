#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include <stdint.h>

// Initialize ultrasonic sensors (TIM + GPIO)
void ULTRASONIC_Init(void);

// Read distance in microseconds for sensor index 0-3
uint16_t ULTRASONIC_Read(uint8_t sensor_index);

#endif
