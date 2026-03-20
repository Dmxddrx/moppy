#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct
{
    float x;
    float y;
    float theta;

} RobotPose;

/* ── Tune these to your robot ──────────────────────────────────
   HC-020K: 20 slots per revolution
   Measure your actual wheel diameter with calipers            */
#define ENCODER_CPR       20       /* counts per revolution         */
#define WHEEL_DIAMETER_M  0.065f   /* 65mm — adjust to your wheel   */
#define WHEEL_CIRC_M      (3.14159f * WHEEL_DIAMETER_M)
#define DIST_PER_COUNT    (WHEEL_CIRC_M / ENCODER_CPR)

void ODOM_Init(void);

void ODOM_Update(float left_dist,
                 float right_dist,
                 float dt);

RobotPose ODOM_GetPose(void);

#endif
