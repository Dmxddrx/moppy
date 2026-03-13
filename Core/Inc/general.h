#ifndef GENERAL_H
#define GENERAL_H

/*#include "coverage.h"
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
#include "wall_follow.h"*/

#include "main.h"
#include "mpu6500.h"
#include "oled.h"

void GENERAL_Init(void);
void GENERAL_OLED_Debug(void);
void GENERAL_Update(void);

#endif
