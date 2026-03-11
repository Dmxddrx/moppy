#include "ir.h"

/* ================================================================
   IR.C
   4x IR object detection sensors on GPIOD.
   Uses port and pin defines from CubeMX-generated main.h.

   IR1 → PD0  IR1_EXTI0
   IR2 → PD1  IR2_EXTI1
   IR3 → PD3  IR3_EXTI3  (PD2 skipped — not EXTI-capable cleanly)
   IR4 → PD4  IR4_EXTI4
   ================================================================ */

static GPIO_TypeDef* const IR_PORT[4] = {
    IR1_EXTI0_GPIO_Port,
    IR2_EXTI1_GPIO_Port,
    IR3_EXTI3_GPIO_Port,
    IR4_EXTI4_GPIO_Port
};

static const uint16_t IR_PIN[4] = {
    IR1_EXTI0_Pin,
    IR2_EXTI1_Pin,
    IR3_EXTI3_Pin,
    IR4_EXTI4_Pin
};

void IR_Init(void)
{
    /* GPIO configured by MX_GPIO_Init() — nothing to do here */
}

uint8_t IR_Read(uint8_t sensor_index)
{
    if(sensor_index >= 4) return 0;
    return (HAL_GPIO_ReadPin(IR_PORT[sensor_index],
                             IR_PIN[sensor_index]) == GPIO_PIN_SET) ? 1 : 0;
}
