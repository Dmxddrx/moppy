#include "coverage.h"
#include <math.h>

/* The Lawnmower Sequence States */
typedef enum {
    LAWN_SWEEPING,
    LAWN_PAUSE,
    LAWN_TURN_1,
    LAWN_STEPPING,
    LAWN_TURN_2,
    LAWN_STOPPED
} LawnState;

static LawnState s_state = LAWN_SWEEPING;
static int s_turn_dir = 1; /* 1 = Right U-Turn, -1 = Left U-Turn */

static float s_target_yaw = 0.0f;

/* For tracking the precise 15cm (0.15m) step */
static float s_step_start_x = 0.0f;
static float s_step_start_y = 0.0f;

/* Delay timer for smooth transitions */
static uint32_t s_pause_ticks = 0;

/* ═══════════════════════════════════════════════════════════════ */
/* HELPER MATH                                                     */
/* ═══════════════════════════════════════════════════════════════ */
static float wrap360(float a) {
    while(a >= 360.0f) a -= 360.0f;
    while(a <    0.0f) a += 360.0f;
    return a;
}

static float angle_diff(float target, float current) {
    float d = target - current;
    while(d >  180.0f) d -= 360.0f;
    while(d < -180.0f) d += 360.0f;
    return d;
}

/* ═══════════════════════════════════════════════════════════════ */
/* PUBLIC FUNCTIONS                                                */
/* ═══════════════════════════════════════════════════════════════ */
void COVERAGE_Init(void) {
    s_state = LAWN_SWEEPING;
    s_turn_dir = 1; /* Start by turning right at the very first wall */
}

/* Add this variable near the top with your other static variables */
static float s_row_heading = 0.0f;

CoverageCmd COVERAGE_Update(RobotPose pose, int obs_F, int obs_R, int obs_L, float* out_target_yaw) {
    CoverageCmd cmd = CMD_DRIVE_FORWARD;

    switch (s_state) {

        case LAWN_SWEEPING:
            *out_target_yaw = s_row_heading; /* IMU Heading Lock! */
            if (obs_F) {
                s_state = LAWN_PAUSE;
                s_pause_ticks = 0;
            } else {
                cmd = CMD_DRIVE_FORWARD;
            }
            break;

        case LAWN_PAUSE:
            cmd = CMD_STOP;
            s_pause_ticks++;
            if (s_pause_ticks > 50) {
                s_state = LAWN_TURN_1;
                s_target_yaw = wrap360(s_row_heading + 90.0f * (float)s_turn_dir);
            }
            break;

        case LAWN_TURN_1:
            cmd = (s_turn_dir == 1) ? CMD_TURN_RIGHT : CMD_TURN_LEFT;
            *out_target_yaw = s_target_yaw;

            if (fabsf(angle_diff(s_target_yaw, pose.theta)) < 5.0f) {
                s_state = LAWN_STEPPING;
                s_step_start_x = pose.x;
                s_step_start_y = pose.y;
                s_row_heading = s_target_yaw; /* Lock the new heading for the shift */
            }
            break;

        case LAWN_STEPPING:
            cmd = CMD_DRIVE_FORWARD;
            *out_target_yaw = s_row_heading; /* IMU Heading Lock for the shift! */

            float dx = pose.x - s_step_start_x;
            float dy = pose.y - s_step_start_y;
            float dist = sqrtf(dx*dx + dy*dy);

            if (obs_F) {
                /* YOUR OLD CODE: The Mid-Shift Abort! */
                s_state = LAWN_TURN_2;
                s_target_yaw = wrap360(s_row_heading + 90.0f * (float)s_turn_dir);
                s_turn_dir *= -1; /* Flip direction early to escape the corner */
            }
            else if (dist >= 0.15f) {
                /* Normal 15cm shift complete */
                s_state = LAWN_TURN_2;
                s_target_yaw = wrap360(s_row_heading + 90.0f * (float)s_turn_dir);
            }
            break;

        case LAWN_TURN_2:
            cmd = (s_turn_dir == 1) ? CMD_TURN_RIGHT : CMD_TURN_LEFT;
            *out_target_yaw = s_target_yaw;

            if (fabsf(angle_diff(s_target_yaw, pose.theta)) < 5.0f) {
                s_row_heading = s_target_yaw; /* Lock the new forward heading */
                s_turn_dir *= -1; /* Normal snake flip */
                s_state = LAWN_SWEEPING;
            }
            break;

        case LAWN_STOPPED:
            cmd = CMD_STOP;
            break;
    }

    return cmd;
}
