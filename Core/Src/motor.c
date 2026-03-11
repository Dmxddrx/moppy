#include "motor.h"

// Direction pins mapping
GPIO_TypeDef* MOTOR_AIN_GPIO[6] = {GPIOC, GPIOC, GPIOC, GPIOC, GPIOE, GPIOE};
uint16_t MOTOR_AIN_PIN[6] = {GPIO_PIN_0, GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_8, GPIO_PIN_4, GPIO_PIN_6};

GPIO_TypeDef* MOTOR_BIN_GPIO[6] = {GPIOC, GPIOC, GPIOC, GPIOC, GPIOE, GPIOE};
uint16_t MOTOR_BIN_PIN[6] = {GPIO_PIN_1, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_9, GPIO_PIN_5, GPIO_PIN_7};

// Direction codes
#define MOTOR_STOP     0
#define MOTOR_FORWARD  1
#define MOTOR_BACKWARD 2

void MOTOR_Init(void)
{
	MOTORPWM_Init();
}

void MOTOR_SetDir(uint8_t motor_index, uint8_t direction)
{
    if(motor_index > 5) return;
    switch(direction)
    {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(MOTOR_AIN_GPIO[motor_index], MOTOR_AIN_PIN[motor_index], GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_BIN_GPIO[motor_index], MOTOR_BIN_PIN[motor_index], GPIO_PIN_RESET);
            break;
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(MOTOR_AIN_GPIO[motor_index], MOTOR_AIN_PIN[motor_index], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_BIN_GPIO[motor_index], MOTOR_BIN_PIN[motor_index], GPIO_PIN_SET);
            break;
        default:
            HAL_GPIO_WritePin(MOTOR_AIN_GPIO[motor_index], MOTOR_AIN_PIN[motor_index], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_BIN_GPIO[motor_index], MOTOR_BIN_PIN[motor_index], GPIO_PIN_RESET);
            break;
    }
}

void MOTOR_Set(uint8_t motor_index, uint8_t direction, uint8_t speed)
{
    MOTOR_SetDir(motor_index, direction);
    MOTOR_PWM_Set(motor_index, speed);
}
