#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "stm32f4xx_hal.h"

#define ENCODER_COUNT 2

void ENCODER_Init(void);
void ENCODER_Update(void);

int32_t ENCODER_GetCount(uint8_t encoder);
void ENCODER_Reset(uint8_t encoder_index);

#endif
