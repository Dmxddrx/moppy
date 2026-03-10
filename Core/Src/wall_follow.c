#include "wall_follow.h"

#define KP 2.0f

void WALLFOLLOW_Init(void)
{
}

void WALLFOLLOW_Update(float distance,
                       float desired,
                       int base_speed,
                       int *left,
                       int *right)
{
    float error = desired - distance;

    float correction = KP * error;

    *left  = base_speed - correction;
    *right = base_speed + correction;
}
