#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>
#include "motor_pwm.h"

/* ---------------------------------------------------------------
   Direction constants — defined here so every module can use them
   without re-defining locally (was the previous bug in motor.c)
---------------------------------------------------------------- */
#define MOTOR_STOP     0
#define MOTOR_FORWARD  1
#define MOTOR_BACKWARD 2

#define MOTOR_SPEED_MAX  999

void MOTOR_Init(void);
void MOTOR_SetDir(uint8_t motor_index, uint8_t direction);
void MOTOR_Set(uint8_t motor_index, uint8_t direction, uint16_t speed);
void MOTOR_StandbyAll(void);   /* pull all STBY LOW  — coast  */
void MOTOR_WakeAll(void);      /* pull all STBY HIGH — enable */

#endif
