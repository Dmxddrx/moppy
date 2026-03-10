#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>

void MOTOR_Init(void);

// Set motor speed in range -1000..1000, negative = reverse
void MOTOR_SetSpeed(int16_t m1, int16_t m2, int16_t m3,
                    int16_t m4, int16_t m5, int16_t m6);

// Stop all motors
void MOTOR_StopAll(void);

#endif
