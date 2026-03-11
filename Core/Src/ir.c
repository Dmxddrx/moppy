#include "ir.h"

// IR pins mapping
GPIO_TypeDef* IR_GPIO[4] = {GPIOD, GPIOD, GPIOD, GPIOD};
uint16_t IR_PIN[4] = {IR1_EXTI0_Pin, IR2_EXTI1_Pin, IR3_EXTI3_Pin, IR4_EXTI4_Pin};

void IR_Init(void)
{
    // EXTI pins already configured in MX_GPIO_Init
    // Optionally enable IRQ here if needed
}

uint8_t IR_Read(uint8_t sensor_index)
{
    if(sensor_index > 3) return 0;
    return (HAL_GPIO_ReadPin(IR_GPIO[sensor_index], IR_PIN[sensor_index]) == GPIO_PIN_SET) ? 1 : 0;
}
