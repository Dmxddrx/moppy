#ifndef COVERAGE_H
#define COVERAGE_H

#include "odometry.h"
#include <stdint.h>

/* The Commands the Captain can issue to the Driver (general.c) */
typedef enum {
    CMD_DRIVE_FORWARD,
	CMD_REVERSE,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_STOP
} CoverageCmd;

void COVERAGE_Init(float start_heading);
/* Telemetry Getters */
int   COVERAGE_GetNavState(void);
int   COVERAGE_GetActiveBCDCell(void);
float COVERAGE_GetTargetHeading(void);

/* * The Brain: Call this every 10ms.
 * It evaluates the surroundings and tells the robot what to do next.
 */

CoverageCmd COVERAGE_Update(
		float current_yaw,
		int obs_F,
		int obs_R,
		int obs_L,
		int obs_B,
		int bump_detected,
		int is_stuck,
		float* out_target_yaw);

#endif /* COVERAGE_H */
