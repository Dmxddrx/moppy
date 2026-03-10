#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct
{
    float x;
    float y;
    float theta;

} RobotPose;

void ODOM_Init(void);

void ODOM_Update(float left_dist,
                 float right_dist,
                 float dt);

RobotPose ODOM_GetPose(void);

#endif
