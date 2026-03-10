#include "slam_lite.h"
#include <math.h>

#define YAW_BLEND 0.05f

void SLAM_Init(void)
{
}

void SLAM_Update(RobotPose *pose,
                 Orientation orient)
{
    float imu_theta = orient.yaw * 0.0174533f;

    pose->theta =
        pose->theta * (1.0f - YAW_BLEND) +
        imu_theta * YAW_BLEND;
}
