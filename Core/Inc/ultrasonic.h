#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"

#define NUM_ULTRASONIC 4

typedef struct {
    uint32_t distance_mm;
} UltrasonicSensor;

void US_Init(TIM_HandleTypeDef *htim);

void US_Trigger(uint8_t sensor_index);

void US_Update(void);

uint32_t US_GetDistance(uint8_t sensor_index);

#endif
