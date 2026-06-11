#include "ir.h"

/* IR state updated by EXTI interrupt, read by IR_Read() */
static volatile uint8_t ir_state[4] = {0, 0, 0, 0};

void IR_Init(void)
{
    /* GPIO and EXTI already configured by CubeMX MX_GPIO_Init.
       Just clear state on init.                               */
    ir_state[0] = 0;
    ir_state[1] = 0;
    ir_state[2] = 0;
    ir_state[3] = 0;
}

uint8_t IR_Read(uint8_t sensor_index)
{
    if(sensor_index > 3) return 0;
    return ir_state[sensor_index];
}

/* Called automatically by HAL when any EXTI line fires.
   GPIO_Pin is the pin mask that triggered.               */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case IR1_EXTI0_Pin: ir_state[0] = 1; break;
        case IR2_EXTI1_Pin: ir_state[1] = 1; break;
        case IR3_EXTI3_Pin: ir_state[2] = 1; break;
        case IR4_EXTI4_Pin: ir_state[3] = 1; break;
        default: break;
    }
}
