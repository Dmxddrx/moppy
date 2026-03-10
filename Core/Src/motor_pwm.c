#include "motor_pwm.h"
#include "tim.h"  // include your htim3/htim4 handles

#define PWM_MAX 1000
#define PWM_STEP 10  // change per update for smooth acceleration

static int16_t current_pwm[MOTOR_MAX] = {0};

static TIM_HandleTypeDef *pwm_htim[MOTOR_MAX] = {&htim3, &htim3, &htim3, &htim4, &htim4, &htim4};
static uint32_t pwm_channel[MOTOR_MAX] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
                                          TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

void MOTORPWM_Init(void)
{
    for(uint8_t i=0; i<MOTOR_MAX; i++)
    {
        HAL_TIM_PWM_Start(pwm_htim[i], pwm_channel[i]);
        __HAL_TIM_SET_COMPARE(pwm_htim[i], pwm_channel[i], 0);
        current_pwm[i] = 0;
    }
}

// Smoothly ramp to target PWM
void MOTORPWM_Update(int16_t m1, int16_t m2, int16_t m3,
                     int16_t m4, int16_t m5, int16_t m6)
{
    int16_t targets[MOTOR_MAX] = {m1, m2, m3, m4, m5, m6};

    for(uint8_t i=0; i<MOTOR_MAX; i++)
    {
        if(targets[i] > PWM_MAX) targets[i] = PWM_MAX;
        if(targets[i] < -PWM_MAX) targets[i] = -PWM_MAX;

        if(current_pwm[i] < targets[i])
        {
            current_pwm[i] += PWM_STEP;
            if(current_pwm[i] > targets[i]) current_pwm[i] = targets[i];
        }
        else if(current_pwm[i] > targets[i])
        {
            current_pwm[i] -= PWM_STEP;
            if(current_pwm[i] < targets[i]) current_pwm[i] = targets[i];
        }

        __HAL_TIM_SET_COMPARE(pwm_htim[i], pwm_channel[i], abs(current_pwm[i]));
    }
}

int16_t MOTORPWM_Get(uint8_t motor_index)
{
    if(motor_index >= MOTOR_MAX) return 0;
    return current_pwm[motor_index];
}
