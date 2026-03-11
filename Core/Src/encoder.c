#include "encoder.h"

/* ================================================================
   ENCODER.C
   Encoder 0 → TIM1  PE9=CH1, PE11=CH2  (left wheel)
   Encoder 1 → TIM8  PC6=CH1, PC7=CH2   (right wheel)

   FIX: Original code had both entries pointing to &htim1,
        reading the same wheel twice. TIM8 added for encoder 1.

   HC-020K is single-channel (no direction output).
   Direction sign is applied in GENERAL_Update() using
   the commanded motor direction state.

   Both timers configured in CubeMX:
     Combined Channels → Encoder Mode
     ARR = 65535, Prescaler = 0, Filter = 4, Pull-Up
   ================================================================ */

extern TIM_HandleTypeDef htim1;   /* Encoder 0 — APB2, 168MHz timer clock */
extern TIM_HandleTypeDef htim8;   /* Encoder 1 — APB2, 168MHz timer clock */

static TIM_HandleTypeDef* ENCODER_TIM[ENCODER_COUNT] = {&htim1, &htim8};

static int32_t encoder_total[ENCODER_COUNT] = {0, 0};
static int16_t encoder_last[ENCODER_COUNT]  = {0, 0};


void ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(ENCODER_TIM[0], TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(ENCODER_TIM[1], TIM_CHANNEL_ALL);

    encoder_last[0] = (int16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIM[0]);
    encoder_last[1] = (int16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIM[1]);
}


void ENCODER_Update(void)
{
    for(uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
        int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIM[i]);
        int16_t diff = now - encoder_last[i];

        encoder_last[i]   = now;
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
    encoder_last[encoder]  = 0;
    __HAL_TIM_SET_COUNTER(ENCODER_TIM[encoder], 0);
}
