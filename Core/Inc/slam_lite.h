#ifndef SLAM_LITE_H
#define SLAM_LITE_H

#include "odometry.h"
#include "stable.h"

void SLAM_Init(void);

void SLAM_Update(RobotPose *pose,
                 Orientation orient);

#endif
