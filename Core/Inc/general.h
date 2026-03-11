#ifndef GENERAL_H
#define GENERAL_H

#include "coverage.h"
#include "encoder.h"
#include "hmc5883l.h"
#include "ir.h"
#include "main.h"
#include "mapping.h"
#include "motion.h"
#include "motor_pwm.h"
#include "motor.h"
#include "mpu6500.h"
#include "odometry.h"
#include "oled.h"
#include "pid.h"
#include "slam_lite.h"
#include "stable.h"
#include "ultrasonic.h"
#include "wall_follow.h"

// Initialize all robot modules
void GENERAL_Init(void);
void GENERAL_OLED_Update(void);
// Update loop: call this in main while(1)
void GENERAL_Update(void);

#endif
