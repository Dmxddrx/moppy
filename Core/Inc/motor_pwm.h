#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "main.h"
#include <stdint.h>

#define MOTOR_MAX 6

// Initialize PWM for all motors
void MOTORPWM_Init(void);

void MOTORPWM_Update(int16_t m1, int16_t m2, int16_t m3,
                     int16_t m4, int16_t m5, int16_t m6);

int16_t MOTORPWM_Get(uint8_t motor_index);

#endif
