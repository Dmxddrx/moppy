#ifndef IR_H
#define IR_H

#include "main.h"
#include <stdint.h>

// Initialize IR sensors
void IR_Init(void);

// Read IR sensor state (0 or 1) by index 0-3
uint8_t IR_Read(uint8_t sensor_index);

#endif
