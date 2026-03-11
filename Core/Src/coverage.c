#include "coverage.h"
#include <math.h>
#include <stdlib.h>

/* ================================================================
   COVERAGE.C — Boustrophedon (lawn-mower) coverage planner

   FIX: TURN state previously committed the new heading and
        returned to FORWARD in the same tick — the robot never
        physically turned. Now it holds in TURN until the IMU
        yaw reaches the target within TURN_DONE_DEG tolerance.
   ================================================================ */

#define TURN_DONE_DEG   5.0f    /* heading tolerance to finish a turn */
#define FORWARD_SPEED   400     /* 0–999, tune to your robot          */

static CoverageState state;
static float row_heading = 0.0f;   /* current driving direction (deg) */
static float turn_target = 0.0f;   /* target heading for active turn  */
static int   direction   = 1;      /* +1 / -1  alternates each row    */


/* ---- helpers -------------------------------------------------- */

static float wrap360(float a)
{
    while(a >= 360.0f) a -= 360.0f;
    while(a <    0.0f) a += 360.0f;
    return a;
}

/* smallest signed difference, result in (-180, +180] */
static float angle_diff(float target, float current)
{
    float d = target - current;
    while(d >  180.0f) d -= 360.0f;
    while(d < -180.0f) d += 360.0f;
    return d;
}


/* ---- public API ----------------------------------------------- */

void COVERAGE_Init(void)
{
    row_heading = 0.0f;
    turn_target = 0.0f;
    direction   = 1;
    state       = COVERAGE_FORWARD;
}

void COVERAGE_Update(RobotPose pose,
                     float     current_yaw,
                     int       obstacle,
                     float    *target_heading,
                     int      *speed)
{
    switch(state)
    {
        /* --------------------------------------------------------
           FORWARD — drive straight on current row heading.
           On obstacle: pre-compute turn target and enter TURN.
        -------------------------------------------------------- */
        case COVERAGE_FORWARD:

            *target_heading = row_heading;
            *speed          = FORWARD_SPEED;

            if(obstacle)
            {
                turn_target = wrap360(row_heading + 90.0f * (float)direction);
                state       = COVERAGE_TURN;
            }

        break;


        /* --------------------------------------------------------
           TURN — stop and rotate toward turn_target.
           Stay here until IMU yaw is within TURN_DONE_DEG.
        -------------------------------------------------------- */
        case COVERAGE_TURN:

            *target_heading = turn_target;
            *speed          = 0;

            if(fabsf(angle_diff(turn_target, current_yaw)) < TURN_DONE_DEG)
            {
                row_heading = turn_target;
                direction  *= -1;
                state       = COVERAGE_FORWARD;
            }

        break;


        case COVERAGE_IDLE:
        default:

            *target_heading = row_heading;
            *speed          = 0;

        break;
    }
}
