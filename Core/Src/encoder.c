#include "encoder.h"

static TIM_HandleTypeDef *htim_encoder;
static Encoder encoder;

void ENC_Init(TIM_HandleTypeDef *htim)
{
    htim_encoder = htim;
    encoder.count = 0;
    encoder.speed_rps = 0;
    HAL_TIM_Encoder_Start(htim_encoder, TIM_CHANNEL_ALL);
}

void ENC_Update(void)
{
    encoder.count = __HAL_TIM_GET_COUNTER(htim_encoder);
    // Speed calculation optional using delta count/time
}

int32_t ENC_GetCount(void)
{
    return encoder.count;
}

float ENC_GetSpeed(void)
{
    return encoder.speed_rps;
}
