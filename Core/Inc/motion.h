#ifndef MOTION_H
#define MOTION_H

#include "pid.h"

void MOTION_Init(void);

void MOTION_DriveStraight(float target_heading,
                          float current_heading,
                          float base_speed,
                          float dt,
                          int *left_motor,
                          int *right_motor);

#endif
