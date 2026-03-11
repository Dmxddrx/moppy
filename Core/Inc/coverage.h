#ifndef COVERAGE_H
#define COVERAGE_H

#include "odometry.h"

typedef enum
{
    COVERAGE_IDLE,
    COVERAGE_FORWARD,
    COVERAGE_TURN
} CoverageState;

void COVERAGE_Init(void);

/* FIX: added current_yaw so TURN state can wait for the robot
        to physically complete the turn before driving forward. */
void COVERAGE_Update(RobotPose pose,
                     float     current_yaw,
                     int       obstacle,
                     float    *target_heading,
                     int      *speed);

#endif
