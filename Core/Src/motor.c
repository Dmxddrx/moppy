#include "motor.h"
#include "motor_pwm.h"
#include "gpio.h"  // Your pin definitions for AIN1/AIN2/BIN1/BIN2

// Map motor index to TB6612FNG pins
typedef struct {
    GPIO_TypeDef *portA;
    uint16_t pinA1;
    uint16_t pinA2;
} MotorPins;

static MotorPins motor_pins[6] = {
    {GPIOC, GPIO_PIN_0, GPIO_PIN_1}, // M1
    {GPIOC, GPIO_PIN_4, GPIO_PIN_5}, // M2
    {GPIOC, GPIO_PIN_6, GPIO_PIN_7}, // M3
    {GPIOC, GPIO_PIN_8, GPIO_PIN_9}, // M4
    {GPIOE, GPIO_PIN_4, GPIO_PIN_5}, // M5
    {GPIOE, GPIO_PIN_6, GPIO_PIN_7}  // M6
};

// Initialize motor pins and PWM
void MOTOR_Init(void)
{
    for(int i=0;i<6;i++)
    {
        HAL_GPIO_WritePin(motor_pins[i].portA, motor_pins[i].pinA1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_pins[i].portA, motor_pins[i].pinA2, GPIO_PIN_RESET);
    }

    MOTORPWM_Init();
}

// Set individual motor speed (-1000..1000)
void MOTOR_SetSpeed(int16_t m1, int16_t m2, int16_t m3,
                    int16_t m4, int16_t m5, int16_t m6)
{
    int16_t motors[6] = {m1, m2, m3, m4, m5, m6};

    for(int i=0;i<6;i++)
    {
        if(motors[i] > 0)
        {
            HAL_GPIO_WritePin(motor_pins[i].portA, motor_pins[i].pinA1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor_pins[i].portA, motor_pins[i].pinA2, GPIO_PIN_RESET);
        }
        else if(motors[i] < 0)
        {
            HAL_GPIO_WritePin(motor_pins[i].portA, motor_pins[i].pinA1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor_pins[i].portA, motor_pins[i].pinA2, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(motor_pins[i].portA, motor_pins[i].pinA1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor_pins[i].portA, motor_pins[i].pinA2, GPIO_PIN_RESET);
        }
    }

    MOTORPWM_Update(m1, m2, m3, m4, m5, m6);
}

// Stop all motors immediately
void MOTOR_StopAll(void)
{
    MOTOR_SetSpeed(0,0,0,0,0,0);
}
