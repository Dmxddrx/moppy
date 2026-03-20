#include "ultrasonic.h"

extern TIM_HandleTypeDef htim2;

/* ----------------------------------------------------------------
   Trigger pins — from your main.h labels
   ---------------------------------------------------------------- */
static GPIO_TypeDef* TRIG_PORT[4] = {
    TRIG1_GPIO_Port,   /* PA8  */
    TRIG2_GPIO_Port,   /* PB1  */
    TRIG3_GPIO_Port,   /* PE8  */
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

#define US_TIMEOUT_US  50000   /* 30ms — beyond this = no sensor or out of range */

static US_State  state[4]      = {US_IDLE};
static uint32_t  rise_tick[4]  = {0};
static uint32_t  trigger_tick[4]  = {0};   /* when trigger was fired */

/* Public data — read from general.c */
ULTRASONIC_Data ultrasonic[4]  = {0};

/* ================================================================
   ULTRASONIC_Init
   ================================================================ */
void ULTRASONIC_Init(void)
{
    for(uint8_t i = 0; i < 4; i++)
        ultrasonic[i].status = US_NO_ECHO;   /* unknown until first echo */

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

    /* Timeout check — echo never came from previous trigger */
    if(state[index] == US_MEASURING)
    {
        uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t elapsed = (now >= trigger_tick[index])
                         ? (now - trigger_tick[index])
                         : (0xFFFFFFFF - trigger_tick[index] + now);

        if(elapsed > US_TIMEOUT_US)
        {
            /* No echo received — mark as disconnected */
            ultrasonic[index].pulse_us = 0;
            ultrasonic[index].ready    = 0;
            ultrasonic[index].status   = US_NO_ECHO;
            rise_tick[index]           = 0;
            state[index]               = US_IDLE;
        }
        else
            return;   /* still within timeout window */
    }

    ultrasonic[index].ready = 0;

    HAL_GPIO_WritePin(TRIG_PORT[index], TRIG_PIN[index], GPIO_PIN_SET);
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while((__HAL_TIM_GET_COUNTER(&htim2) - start) < 10);
    HAL_GPIO_WritePin(TRIG_PORT[index], TRIG_PIN[index], GPIO_PIN_RESET);

    trigger_tick[index] = __HAL_TIM_GET_COUNTER(&htim2);
    state[index]        = US_MEASURING;
}

/* ================================================================
   ULTRASONIC_CaptureCallback
   Call this from HAL_TIM_IC_CaptureCallback in your stm32f4xx_it.c
   or wherever you handle TIM2 IC interrupts.
   ================================================================ */
void ULTRASONIC_CaptureCallback(uint32_t channel)
{
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
        /* Rising edge — sensor is alive */
        rise_tick[i]          = captured;
        ultrasonic[i].status  = US_OK;

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
        /* Falling edge — calculate pulse width */
        uint32_t fall = captured;
        uint32_t rise = rise_tick[i];

        ultrasonic[i].pulse_us = (fall >= rise) ? (fall - rise)
                                                 : (0xFFFFFFFF - rise + fall);
        ultrasonic[i].ready  = 1;
        rise_tick[i]         = 0;
        state[i]             = US_IDLE;

        TIM_IC_InitTypeDef cfg = {0};
        cfg.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
        cfg.ICSelection = TIM_ICSELECTION_DIRECTTI;
        cfg.ICPrescaler = TIM_ICPSC_DIV1;
        cfg.ICFilter    = 0;
        HAL_TIM_IC_ConfigChannel(&htim2, &cfg, ECHO_CHANNEL[i]);
        HAL_TIM_IC_Start_IT(&htim2, ECHO_CHANNEL[i]);
    }
}
