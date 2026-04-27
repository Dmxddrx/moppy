#ifndef COVERAGE_H
#define COVERAGE_H

#include "odometry.h"
#include <stdint.h>

/* The Commands the Captain can issue to the Driver (general.c) */
typedef enum {
    CMD_DRIVE_FORWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_STOP
} CoverageCmd;

void COVERAGE_Init(void);

/* * The Brain: Call this every 10ms.
 * It evaluates the surroundings and tells the robot what to do next.
 */
CoverageCmd COVERAGE_Update(RobotPose pose, int obs_F, int obs_R, int obs_L, float* out_target_yaw);

#endif /* COVERAGE_H */
