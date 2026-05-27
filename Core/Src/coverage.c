#include "coverage.h"
#include <math.h>

/* ── Reactive Navigation State Machine ────────────────────── */
typedef enum {
    NAV_FORWARD,
    NAV_REVERSE,
    NAV_TURN_RIGHT_180,
    NAV_TURN_LEFT_180,
    NAV_TURN_RIGHT_90,
    NAV_TURN_LEFT_90,
	NAV_TRAPPED
} NavState;

static NavState s_nav_state = NAV_FORWARD;
static float s_target_heading = 0.0f;

/* ═══════════════════════════════════════════════════════════════ */
/* HELPER MATH                                                     */
/* ═══════════════════════════════════════════════════════════════ */
static float angle_diff(float target, float current) {
    float d = target - current;
    while(d >  180.0f) d -= 360.0f;
    while(d < -180.0f) d += 360.0f;
    return d;
}

/* ═══════════════════════════════════════════════════════════════ */
/* PUBLIC FUNCTIONS                                                */
/* ═══════════════════════════════════════════════════════════════ */
void COVERAGE_Init(float start_heading) {
    s_nav_state = NAV_FORWARD;
    s_target_heading = start_heading;
}

CoverageCmd COVERAGE_Update(float current_yaw, int obs_F, int obs_R, int obs_L, int obs_B, float* out_target_yaw) {
    CoverageCmd cmd = CMD_DRIVE_FORWARD;

    switch (s_nav_state) {

        case NAV_FORWARD:
            cmd = CMD_DRIVE_FORWARD;
            *out_target_yaw = s_target_heading; /* Keep driving straight */

            if (obs_F && obs_L && obs_R && obs_B) {
				s_nav_state = NAV_TRAPPED; // <--- This triggers the trap state!
			}
            /* Obstacle Logic Priority */
            else if (obs_F && obs_L && obs_R) {
                s_nav_state = NAV_REVERSE;
            }
            else if (obs_F && obs_R) {
                s_nav_state = NAV_TURN_LEFT_180;
                /* Left turn = Up Counter (+180) */
                s_target_heading = fmodf(current_yaw + 180.0f, 360.0f);
            }
            else if (obs_F && obs_L) {
                s_nav_state = NAV_TURN_RIGHT_180;
                /* Right turn = Down Counter. Add 360 to prevent negative math! */
                s_target_heading = fmodf(current_yaw + 360.0f - 180.0f, 360.0f);
            }
            else if (obs_F) {
                s_nav_state = NAV_TURN_RIGHT_180;
                s_target_heading = fmodf(current_yaw + 360.0f - 180.0f, 360.0f);
            }
            break;

        case NAV_REVERSE:
            cmd = CMD_REVERSE;
            *out_target_yaw = s_target_heading;

            if (!obs_L) {
                s_nav_state = NAV_TURN_LEFT_90;
                s_target_heading = fmodf(current_yaw + 90.0f, 360.0f);
            }
            else if (!obs_R) {
                s_nav_state = NAV_TURN_RIGHT_90;
                s_target_heading = fmodf(current_yaw + 360.0f - 90.0f, 360.0f);
            }
            break;
        case NAV_TRAPPED:
			/* Halt all motors and wait for a human or object to move out of the way */
			cmd = CMD_STOP;
			*out_target_yaw = s_target_heading;

			if (!obs_F || !obs_L || !obs_R || !obs_B) {
				/* An opening appeared! Go back to Forward mode to recalculate the route */
				s_nav_state = NAV_FORWARD;
			}
			break;

        case NAV_TURN_RIGHT_180:
        case NAV_TURN_LEFT_180:
        case NAV_TURN_RIGHT_90:
        case NAV_TURN_LEFT_90:
            /* Output the correct turning command to the hardware driver */
            if (s_nav_state == NAV_TURN_LEFT_90 || s_nav_state == NAV_TURN_LEFT_180) {
                cmd = CMD_TURN_LEFT;
            } else {
                cmd = CMD_TURN_RIGHT;
            }

            *out_target_yaw = s_target_heading;

            /* If the compass is within 3 degrees of target, lock the FSM back to Forward! */
            if (fabsf(angle_diff(s_target_heading, current_yaw)) <= 3.0f) {
                s_nav_state = NAV_FORWARD;
            }
            break;
    }

    return cmd;
}
