#include "coverage.h"
#include <math.h>
#include <stdlib.h>

#define TURN_DONE_DEG    5.0f    /* tolerance to finish a turn */
#define FORWARD_SPEED    400     /* Base PWM speed */
#define SHIFT_DISTANCE_M 0.30f   /* Distance to shift to the next row (30cm) */

static CoverageState state;
static float row_heading = 0.0f;
static float turn_target = 0.0f;
static int   direction   = 1;      /* +1 for Right turns, -1 for Left turns */

/* Variables to track how far we have driven during the row shift */
static float shift_start_x = 0.0f;
static float shift_start_y = 0.0f;

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

void COVERAGE_Init(void) {
    row_heading = 0.0f;
    turn_target = 0.0f;
    direction   = 1;
    state       = COVERAGE_FORWARD;
}

void COVERAGE_Update(RobotPose pose, float current_yaw, int obstacle, float *target_heading, int *speed)
{
    switch(state)
    {
        case COVERAGE_FORWARD:
            *target_heading = row_heading;
            *speed          = FORWARD_SPEED;

            if(obstacle) {
                /* Hit a wall! Calculate 90 deg turn */
                turn_target = wrap360(row_heading + 90.0f * (float)direction);
                state       = COVERAGE_TURN_1;
            }
            break;

        case COVERAGE_TURN_1:
            *target_heading = turn_target;
            *speed          = 0; /* Turn in place */

            if(fabsf(angle_diff(turn_target, current_yaw)) < TURN_DONE_DEG) {
                shift_start_x = pose.x;
                shift_start_y = pose.y;
                row_heading   = turn_target;
                state         = COVERAGE_SHIFT;
            }
            break;

        case COVERAGE_SHIFT:
            *target_heading = row_heading;
            *speed          = FORWARD_SPEED;

            float dx = pose.x - shift_start_x;
            float dy = pose.y - shift_start_y;
            float dist_moved = sqrtf(dx*dx + dy*dy);

            if(obstacle) {
                /* IMPROVEMENT: Abort shift if blocked!
                   We hit a wall mid-shift. Force a 180-turn to escape the corner
                   and immediately head back down the room. */
                turn_target = wrap360(row_heading + 90.0f * (float)direction);
                direction  *= -1; /* Flip direction early to escape */
                state       = COVERAGE_TURN_2;
            }
            else if(dist_moved >= SHIFT_DISTANCE_M) {
                /* Normal 30cm shift complete */
                turn_target = wrap360(row_heading + 90.0f * (float)direction);
                state       = COVERAGE_TURN_2;
            }
            break;

        case COVERAGE_TURN_2:
            *target_heading = turn_target;
            *speed          = 0;

            if(fabsf(angle_diff(turn_target, current_yaw)) < TURN_DONE_DEG) {
                row_heading = turn_target;
                direction  *= -1; /* Normal snake flip */
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
