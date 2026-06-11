#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "main.h"
#include <stdint.h>

#define MOTOR_MAX 6
#define PWM_MAX     999

// Initialize PWM for all motors
void MOTORPWM_Init(void);

void MOTORPWM_Update(int16_t m1, int16_t m2, int16_t m3,
                     int16_t m4, int16_t m5, int16_t m6);

/* Set one motor immediately — used by MOTOR_Set()
   FIX: was MOTOR_PWM_Set() which never existed             */
void    MOTORPWM_SetOne(uint8_t motor_index, int16_t value);

int16_t MOTORPWM_Get(uint8_t motor_index);

#endif
