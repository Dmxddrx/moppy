#include "odometry.h"
#include <math.h>

#define WHEEL_BASE 0.25f

static RobotPose pose;

void ODOM_Init(void)
{
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
}

void ODOM_Update(float left_dist,
                 float right_dist,
                 float dt)
{
    float d = (left_dist + right_dist) / 2.0f;

    float dtheta = (right_dist - left_dist) / WHEEL_BASE;

    pose.theta += dtheta;

    pose.x += d * cosf(pose.theta);
    pose.y += d * sinf(pose.theta);
}

RobotPose ODOM_GetPose(void)
{
    return pose;
}
