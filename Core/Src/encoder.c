#include "encoder.h"
#include "math.h"
/* ================================================================
   ENCODER.C
   Encoder 0 → TIM1  ENC1_A(PE9)=CH1, ENC1_B(PE11)=CH2  — left wheel
   Encoder 1 → TIM8  ENC2_A(PC6)=CH1, ENC2_B(PC7) =CH2  — right wheel

   HC-020K is single-channel (no direction output).
   CH2 disconnected — counter counts in one direction only.
   Direction sign must be applied externally from motor command state.

   Both timers configured in CubeMX:
     Encoder Mode TI12, ARR=65535, Prescaler=0
   ================================================================ */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

static TIM_HandleTypeDef* ENC_TIM[ENCODER_COUNT] = {
    &htim1,   /* Encoder 0 — ENC1_A/ENC1_B */
    &htim8    /* Encoder 1 — ENC2_A/ENC2_B */
};

static int32_t    enc_total[ENCODER_COUNT]         = {0};
static int16_t    enc_last[ENCODER_COUNT]           = {0};
static float      enc_speed_raw[ENCODER_COUNT]      = {0.0f};
static float      enc_speed_display[ENCODER_COUNT]  = {0.0f};
static ENC_Status enc_status[ENCODER_COUNT]         = {ENC_NO_SIGNAL};
static int32_t    enc_last_total[ENCODER_COUNT]     = {0};

/* ================================================================
   ENCODER_Init
   ================================================================ */
void ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(ENC_TIM[0], TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(ENC_TIM[1], TIM_CHANNEL_ALL);

    enc_last[0] = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM[0]);
    enc_last[1] = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM[1]);
}

/* ================================================================
   ENCODER_Update
   Called from TIM6 interrupt every 1ms.
   int16_t cast handles 16-bit counter overflow automatically.
   ================================================================ */
void ENCODER_Update(void)
{
    static int32_t speed_sum[ENCODER_COUNT] = {0};
    static uint8_t speed_cnt[ENCODER_COUNT] = {0};

    for(uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
        int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM[i]);
        int16_t diff = now - enc_last[i];

        enc_last[i]   = now;
        enc_total[i] += diff;

        /* Raw speed — updated every 1ms, used by PID */
        enc_speed_raw[i] = (float)diff * 1000.0f;

        /* Smoothed speed — averaged over 10ms, used by OLED */
        speed_sum[i] += diff;
        speed_cnt[i]++;
        if(speed_cnt[i] >= 10)
        {
        	enc_speed_display[i] = fabsf((float)speed_sum[i] * 100.0f);
            speed_sum[i]         = 0;
            speed_cnt[i]         = 0;
        }

        /* Status — flips to OK on first pulse received */
        if(enc_total[i] != enc_last_total[i])
        {
            enc_status[i]     = ENC_OK;
            enc_last_total[i] = enc_total[i];
        }
    }
}

/* ================================================================
   ENCODER_GetCount — total accumulated counts since last reset
   ================================================================ */
int32_t ENCODER_GetCount(uint8_t index)
{
    if(index >= ENCODER_COUNT) return 0;
    return enc_total[index];
}

/* ================================================================
   ENCODER_GetSpeed — smoothed, for OLED display
   ================================================================ */
float ENCODER_GetSpeed(uint8_t index)
{
    if(index >= ENCODER_COUNT) return 0.0f;
    return enc_speed_display[index];
}

/* ================================================================
   ENCODER_GetSpeed_PID — raw, for PID controller
   ================================================================ */
float ENCODER_GetSpeed_PID(uint8_t index)
{
    if(index >= ENCODER_COUNT) return 0.0f;
    return enc_speed_raw[index];
}

/* ================================================================
   ENCODER_GetStatus
   ================================================================ */
ENC_Status ENCODER_GetStatus(uint8_t index)
{
    if(index >= ENCODER_COUNT) return ENC_NO_SIGNAL;
    return enc_status[index];
}

/* ================================================================
   ENCODER_Reset
   ================================================================ */
void ENCODER_Reset(uint8_t index)
{
    if(index >= ENCODER_COUNT) return;

    enc_total[index]     = 0;
    enc_last[index]      = 0;
    enc_last_total[index] = 0;
    enc_status[index]    = ENC_NO_SIGNAL;
    __HAL_TIM_SET_COUNTER(ENC_TIM[index], 0);
}
