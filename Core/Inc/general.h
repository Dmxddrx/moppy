#ifndef GENERAL_H
#define GENERAL_H

#include "main.h"
#include "ultrasonic.h"
#include "ir.h"
#include "encoder.h"
#include "mpu6500.h"
#include "hmc5883l.h"
#include "stable.h"
#include "motor_pwm.h"
#include "mapping.h"

// Initialize all sensors and modules
void GENERAL_Init(void);

// Call in main loop periodically
void GENERAL_Update(void);

#endif
