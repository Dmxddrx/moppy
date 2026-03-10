#ifndef IR_H
#define IR_H

#include "main.h"

#define NUM_IR 4

typedef struct {
    uint8_t state; // 0 = no obstacle, 1 = obstacle detected
} IRSensor;

void IR_Init(void);

uint8_t IR_Get(uint8_t sensor_index);

#endif
