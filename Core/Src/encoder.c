#include "encoder.h"
#include <math.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim6;

/* ----------------------------------------------------------------
   All state updated exclusively from interrupts — no polling
   volatile: read from main context, written from ISR
   ---------------------------------------------------------------- */
static volatile int32_t    enc_total[ENCODER_COUNT]        = {0};
static volatile uint32_t   enc_last_capture[ENCODER_COUNT] = {0};
static volatile float      enc_speed_display[ENCODER_COUNT]= {0.0f};
static volatile float      enc_speed_raw[ENCODER_COUNT]    = {0.0f};
static volatile ENC_Status enc_status[ENCODER_COUNT]       = {ENC_NO_SIGNAL};

/* Speed calculation — uses time between consecutive pulses
   TIM1/TIM8 run at 84MHz with prescaler=83 → 1MHz (1µs per tick) */
#define ENC_TIMER_FREQ_HZ   1000000UL
#define ENC_SPEED_TIMEOUT   500000UL   /* 0.5s no pulse = stopped  */

/* ================================================================
   ENCODER_Init
   ================================================================ */
void ENCODER_Init(void)
{
    /* Start input capture interrupt on both encoder channels */
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);   /* ENC1_A PE9 */
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);   /* ENC2_A PC6 */

    /* ADD THIS: Start the 1ms stall-detection timer */
    HAL_TIM_Base_Start_IT(&htim6);
}

/* ================================================================
   ENCODER_IC_Callback
   Called from HAL_TIM_IC_CaptureCallback in stm32f4xx_it.c
   index: 0 = ENC1 (TIM1), 1 = ENC2 (TIM8)
   ================================================================ */
void ENCODER_IC_Callback(uint8_t index, uint32_t captured)
{
    if(index >= ENCODER_COUNT) return;

    uint32_t last = enc_last_capture[index];
    uint32_t period;

    if(captured >= last)
        period = captured - last;
    else
        period = (0xFFFF - last) + captured + 1;

    /* FIX 1: Electrical Noise Filter
       If the pulse period is < 1000us (1ms), it means the wheel is spinning
       at > 3000 RPM. This is impossible, so it must be electrical noise. */
    if (period < 1000) {
        return;
    }

    /* Update ticks and save valid capture time */
    enc_total[index]++;
    enc_status[index] = ENC_OK;
    enc_last_capture[index] = captured;

    if(period < ENC_SPEED_TIMEOUT)
    {
        float pulses_per_sec = (float)ENC_TIMER_FREQ_HZ / (float)period;
        float rad_per_sec    = pulses_per_sec * (6.28318f / 20.0f);

        /* FIX 2: 16-Bit Timer Overflow Aliasing
           A normal Roomba wheel maxes out at ~30 rad/s. If the math calculates
           > 50 rad/s, the timer overflowed and tricked the math. Ignore it. */
        if (rad_per_sec < 50.0f) {
            enc_speed_raw[index]     = rad_per_sec;
            enc_speed_display[index] = rad_per_sec;
        }
    }
}

/* ================================================================
   ENCODER_Update — called from TIM6 1ms interrupt
   Only job now: detect stopped wheel (no pulses for 0.5s)
   No counter polling — speed/count come from IC interrupts
   ================================================================ */
void ENCODER_Update(void)
{
    static uint32_t stall_cnt[ENCODER_COUNT] = {0};
    static int32_t  last_total[ENCODER_COUNT] = {0};

    for(uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
        if(enc_total[i] == last_total[i])
        {
            stall_cnt[i]++;
            if(stall_cnt[i] >= 500)   /* 500ms no pulse */
            {
                enc_speed_display[i] = 0.0f;
                enc_speed_raw[i]     = 0.0f;
                stall_cnt[i]         = 0;
            }
        }
        else
        {
            stall_cnt[i]  = 0;
            last_total[i] = enc_total[i];
        }
    }
}

/* ================================================================
   Getters — safe to call from main context
   ================================================================ */
int32_t ENCODER_GetCount(uint8_t index)
{
    if(index >= ENCODER_COUNT) return 0;
    return enc_total[index];
}

float ENCODER_GetSpeed(uint8_t index)
{
    if(index >= ENCODER_COUNT) return 0.0f;
    return enc_speed_display[index];
}

float ENCODER_GetSpeed_PID(uint8_t index)
{
    if(index >= ENCODER_COUNT) return 0.0f;
    return enc_speed_raw[index];
}

ENC_Status ENCODER_GetStatus(uint8_t index)
{
    if(index >= ENCODER_COUNT) return ENC_NO_SIGNAL;
    return enc_status[index];
}

void ENCODER_Reset(uint8_t index)
{
    if(index >= ENCODER_COUNT) return;
    enc_total[index]        = 0;
    enc_last_capture[index] = 0;
    enc_speed_display[index]= 0.0f;
    enc_speed_raw[index]    = 0.0f;
    enc_status[index]       = ENC_NO_SIGNAL;
}
