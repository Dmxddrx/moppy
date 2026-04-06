#ifndef COVERAGE_H
#define COVERAGE_H

#include "odometry.h"

typedef enum
{
    COVERAGE_IDLE,
    COVERAGE_FORWARD,
    COVERAGE_TURN_1,    /* First 90-degree turn */
    COVERAGE_SHIFT,     /* Drive forward to the next row */
    COVERAGE_TURN_2     /* Second 90-degree turn to face back */
} CoverageState;

void COVERAGE_Init(void);

void COVERAGE_Update(RobotPose pose,
                     float     current_yaw,
                     int       obstacle,
                     float    *target_heading,
                     int      *speed);

#endif
