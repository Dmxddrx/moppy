#include "ultrasonic.h"

extern TIM_HandleTypeDef htim2;

/* ----------------------------------------------------------------
   Trigger pins — from your main.h labels
   ---------------------------------------------------------------- */
static GPIO_TypeDef* TRIG_PORT[4] = {
    TRIG1_GPIO_Port,   /* PA8  */
    TRIG2_GPIO_Port,   /* PB1  */
    TRIG3_GPIO_Port,   /* PB2  */
    TRIG4_GPIO_Port    /* PD7  */
};

static uint16_t TRIG_PIN[4] = {
    TRIG1_Pin,
    TRIG2_Pin,
    TRIG3_Pin,
    TRIG4_Pin
};

/* ----------------------------------------------------------------
   Echo channels — TIM2 CH1-CH4 (PA0-PA3 = SR04_ECHO1-4)
   TIM2 prescaler = 84-1 → 1MHz → 1 tick = 1µs
   ---------------------------------------------------------------- */
static uint32_t ECHO_CHANNEL[4] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};

/* ----------------------------------------------------------------
   Internal state per sensor
   ---------------------------------------------------------------- */
typedef enum {
    US_IDLE      = 0,
    US_MEASURING = 1,
} US_State;

static US_State  state[4]      = {US_IDLE};
static uint32_t  rise_tick[4]  = {0};

/* Public data — read from general.c */
ULTRASONIC_Data ultrasonic[4]  = {0};

/* ================================================================
   ULTRASONIC_Init
   ================================================================ */
void ULTRASONIC_Init(void)
{
    /* Start input capture interrupt on all 4 echo channels */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
}

/* ================================================================
   ULTRASONIC_Trigger
   Send 10µs pulse on trigger pin for sensor index 0-3.
   Call this periodically from GENERAL_Update — one sensor
   per call to avoid overlap (stagger by ~60ms each).
   ================================================================ */
void ULTRASONIC_Trigger(uint8_t index)
{
    if(index > 3) return;
    if(state[index] == US_MEASURING) return;  /* previous echo not done */

    ultrasonic[index].ready = 0;
    state[index] = US_IDLE;

    /* 10µs trigger pulse — TIM2 is 1MHz so we use counter directly */
    HAL_GPIO_WritePin(TRIG_PORT[index], TRIG_PIN[index], GPIO_PIN_SET);

    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while((__HAL_TIM_GET_COUNTER(&htim2) - start) < 10);   /* wait 10µs */

    HAL_GPIO_WritePin(TRIG_PORT[index], TRIG_PIN[index], GPIO_PIN_RESET);

    state[index] = US_MEASURING;
}

/* ================================================================
   ULTRASONIC_CaptureCallback
   Call this from HAL_TIM_IC_CaptureCallback in your stm32f4xx_it.c
   or wherever you handle TIM2 IC interrupts.
   ================================================================ */
void ULTRASONIC_CaptureCallback(uint32_t channel)
{
    /* Map TIM channel to sensor index */
    uint8_t i;
    switch(channel)
    {
        case TIM_CHANNEL_1: i = 0; break;
        case TIM_CHANNEL_2: i = 1; break;
        case TIM_CHANNEL_3: i = 2; break;
        case TIM_CHANNEL_4: i = 3; break;
        default: return;
    }

    if(state[i] != US_MEASURING) return;

    uint32_t captured = HAL_TIM_ReadCapturedValue(&htim2, ECHO_CHANNEL[i]);

    if(rise_tick[i] == 0)
    {
        /* First capture = rising edge */
        rise_tick[i] = captured;

        /* Switch polarity to catch falling edge */
        TIM_IC_InitTypeDef cfg = {0};
        cfg.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
        cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
        cfg.ICPrescaler = TIM_ICPSC_DIV1;
        cfg.ICFilter    = 0;
        HAL_TIM_IC_ConfigChannel(&htim2, &cfg, ECHO_CHANNEL[i]);
        HAL_TIM_IC_Start_IT(&htim2, ECHO_CHANNEL[i]);
    }
    else
    {
        /* Second capture = falling edge — calculate width */
        uint32_t fall = captured;
        uint32_t rise = rise_tick[i];

        /* Handle 32-bit timer overflow */
        ultrasonic[i].pulse_us = (fall >= rise) ? (fall - rise)
                                                 : (0xFFFFFFFF - rise + fall);
        ultrasonic[i].ready = 1;

        /* Reset for next measurement */
        rise_tick[i] = 0;
        state[i]     = US_IDLE;

        /* Restore rising edge polarity */
        TIM_IC_InitTypeDef cfg = {0};
        cfg.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
        cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
        cfg.ICPrescaler = TIM_ICPSC_DIV1;
        cfg.ICFilter    = 0;
        HAL_TIM_IC_ConfigChannel(&htim2, &cfg, ECHO_CHANNEL[i]);
        HAL_TIM_IC_Start_IT(&htim2, ECHO_CHANNEL[i]);
    }
}
