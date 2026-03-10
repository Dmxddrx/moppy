#include "coverage.h"
#include <math.h>

static CoverageState state;

static float row_heading = 0;
static int direction = 1;

void COVERAGE_Init(void)
{
    state = COVERAGE_FORWARD;
}

void COVERAGE_Update(RobotPose pose,
                     int obstacle,
                     float *target_heading,
                     int *speed)
{
    switch(state)
    {
        case COVERAGE_FORWARD:

            *target_heading = row_heading;
            *speed = 200;

            if(obstacle)
            {
                state = COVERAGE_TURN;
            }

        break;

        case COVERAGE_TURN:

            row_heading += 90 * direction;

            if(row_heading >= 360)
                row_heading -= 360;

            direction *= -1;

            *speed = 0;

            state = COVERAGE_FORWARD;

        break;

        default:
        break;
    }
}
