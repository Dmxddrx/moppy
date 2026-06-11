#include "motion.h"

static PID heading_pid;

void MOTION_Init(void)
{
    PID_Init(&heading_pid, 2.0f, 0.0f, 0.3f);
}

void MOTION_DriveStraight(float target_heading,
                          float current_heading,
                          float base_speed,
                          float dt,
                          int *left_motor,
                          int *right_motor)
{
    float correction =
        PID_Update(&heading_pid,
                   target_heading,
                   current_heading,
                   dt);

    *left_motor  = base_speed - correction;
    *right_motor = base_speed + correction;
}
