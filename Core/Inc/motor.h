#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>
#include "motor_pwm.h"

// Initialize motor GPIOs and PWM
void MOTOR_Init(void);

// Set motor direction (0: stop, 1: forward, 2: backward)
void MOTOR_SetDir(uint8_t motor_index, uint8_t direction);

// Set motor speed and direction
void MOTOR_Set(uint8_t motor_index, uint8_t direction, uint8_t speed);

#endif
