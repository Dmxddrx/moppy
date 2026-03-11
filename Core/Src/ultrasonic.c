#include "ultrasonic.h"
#include "main.h"

// declare timers defined in main.c
extern TIM_HandleTypeDef htim2;


// Sensor trigger pins (define if using triggers)
GPIO_TypeDef* ULTRASONIC_GPIO[4] = {GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t ULTRASONIC_PIN[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};

// TIM2 channels correspond to echo inputs
TIM_HandleTypeDef* ULTRASONIC_TIM[4] = {&htim2, &htim2, &htim2, &htim2};
uint32_t ULTRASONIC_CHANNEL[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

// Initialize timer input capture
void ULTRASONIC_Init(void)
{
    // Start TIM2 input capture for all 4 channels
    for(uint8_t i=0;i<4;i++)
    {
        HAL_TIM_IC_Start(ULTRASONIC_TIM[i], ULTRASONIC_CHANNEL[i]);
    }
}

// Read ultrasonic pulse duration (microseconds)
uint16_t ULTRASONIC_Read(uint8_t sensor_index)
{
    if(sensor_index > 3) return 0;
    // Simple input capture reading
    return __HAL_TIM_GET_COUNTER(ULTRASONIC_TIM[sensor_index]);
}
