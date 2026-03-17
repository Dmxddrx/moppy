#ifndef GENERAL_H
#define GENERAL_H

/*#include "coverage.h"

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

#include "wall_follow.h"*/

#include "main.h"
#include "mpu6500.h"
#include "hmc5883l.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "motor.h"
#include "btns.h"
#include "oled.h"

#define ENABLE_OLED_SELFTEST   0
#define ENABLE_OLED_DEBUG      1

void GENERAL_Init(void);
void GENERAL_Update(void);

#if ENABLE_OLED_SELFTEST
void GENERAL_OLED_SelfTest(void);
#endif

#if ENABLE_OLED_DEBUG
void GENERAL_OLED_Debug(void);
#endif

#endif
