#include "wall_follow.h"
#include <math.h>

#define KP_DIST    2.0f   /* Aggressiveness for fixing distance */
#define KP_HEADING 3.5f   /* Aggressiveness for driving straight */

/* Helper function to calculate shortest angular distance */
static float angle_diff(float target, float current) {
    float d = target - current;
    while(d >  180.0f) d -= 360.0f;
    while(d < -180.0f) d += 360.0f;
    return d;
}

void WALLFOLLOW_Init(void)
{
}

void WALLFOLLOW_Update(float current_distance, float desired_distance,
                       float current_heading,  float wall_parallel_heading,
                       int base_speed,
                       int *left, int *right)
{
    /* 1. Calculate how far we are from the ideal line */
    float dist_error = desired_distance - current_distance;

    /* 2. Calculate how crooked we are driving relative to the wall */
    float heading_error = angle_diff(wall_parallel_heading, current_heading);

    /* 3. Fuse them!
       If we are too close, dist_error pushes us left.
       If we turn left too sharply, heading_error pushes back right to dampen it! */
    float correction = (KP_DIST * dist_error) - (KP_HEADING * heading_error);

    /* 4. Apply to motors */
    *left  = base_speed - (int)correction;
    *right = base_speed + (int)correction;
}
