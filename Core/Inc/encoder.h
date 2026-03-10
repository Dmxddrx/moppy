#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

typedef struct {
    int32_t count;
    float speed_rps;
} Encoder;

void ENC_Init(TIM_HandleTypeDef *htim);

int32_t ENC_GetCount(void);

float ENC_GetSpeed(void);

void ENC_Update(void);

#endif
