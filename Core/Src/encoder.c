#include "encoder.h"

// TIM handle from main.c
extern TIM_HandleTypeDef htim1;

static TIM_HandleTypeDef* ENCODER_TIM[ENCODER_COUNT] = {&htim1, &htim1};

static int32_t encoder_total[ENCODER_COUNT] = {0};
static int16_t encoder_last[ENCODER_COUNT] = {0};


void ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(ENCODER_TIM[0], TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(ENCODER_TIM[1], TIM_CHANNEL_ALL);

    encoder_last[0] = __HAL_TIM_GET_COUNTER(ENCODER_TIM[0]);
    encoder_last[1] = __HAL_TIM_GET_COUNTER(ENCODER_TIM[1]);
}


void ENCODER_Update(void)
{
    for(uint8_t i=0;i<ENCODER_COUNT;i++)
    {
        int16_t now = __HAL_TIM_GET_COUNTER(ENCODER_TIM[i]);
        int16_t diff = now - encoder_last[i];

        encoder_last[i] = now;

        encoder_total[i] += diff;
    }
}


int32_t ENCODER_GetCount(uint8_t encoder)
{
    if(encoder >= ENCODER_COUNT) return 0;

    return encoder_total[encoder];
}


void ENCODER_Reset(uint8_t encoder)
{
    if(encoder >= ENCODER_COUNT) return;

    encoder_total[encoder] = 0;
    __HAL_TIM_SET_COUNTER(ENCODER_TIM[encoder],0);
}
