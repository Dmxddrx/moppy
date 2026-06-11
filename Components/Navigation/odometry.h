#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <mpu6050.h>

/* ── RobotPose ──────────────────────────────────────────────────
   x, y  : metres in world frame (x=East, y=North)
   theta  : heading in DEGREES, 0=North, 0–360 CW (compass)      */
typedef struct {
    float x;
    float y;
    float theta;   /* degrees, 0=North CW */
} RobotPose;

/* ── Wheel constants (kept for future encoder-based mode) ──────*/
#define ENCODER_CPR       20
#define WHEEL_DIAMETER_M  0.065f
#define WHEEL_CIRC_M      (3.14159f * WHEEL_DIAMETER_M)
#define DIST_PER_COUNT    (WHEEL_CIRC_M / ENCODER_CPR)

/* ── IMU dead-band — below this, acceleration = noise ──────────*/
#define ODOM_ACCEL_DEADBAND_MS2   0.25f

/* ── Max believable speed (m/s) for a hand-pushed robot ────────*/
#define ODOM_SPEED_CLAMP_MS       1.50f

void      ODOM_Init(void);

/* Encoder-based update (motors running) */
void      ODOM_Update(float left_dist, float right_dist, float dt);

/* IMU-based update (no motors — manually pushed)
   heading_deg: absolute heading from STABLE (0–360 CW from North) */
void      ODOM_UpdateIMU(const MPU6050_RawData *imu,
                          float heading_deg, float dt);

void      ODOM_SetPose(float x, float y, float theta_deg);
void      ODOM_ResetVelocity(void);   /* call when robot is known-stationary */

RobotPose ODOM_GetPose(void);
float     ODOM_GetSpeed(void);        /* scalar m/s               */

void ODOM_UpdateEncoders(float v_left, float v_right, float yaw_deg, float dt);

#endif /* ODOMETRY_H */
