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

void COVERAGE_Update(RobotPose pose,
                     int obstacle,
                     float *target_heading,
                     int *speed);

#endif
