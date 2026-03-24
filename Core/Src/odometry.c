#include "odometry.h"
#include <math.h>
#include <string.h>

/* ── Coordinate convention ──────────────────────────────────────
   World frame  : x = East, y = North
   Heading      : 0 = North, 90 = East, CW (compass convention)
   MPU6500 body : ax = forward, ay = left, az = up
                  (verify with your physical mounting)

   Rotation from body → world (compass heading θ):
     ax_world =  ax_body × sin(θ) + ay_body × cos(θ)   ← East
     ay_world =  ax_body × cos(θ) − ay_body × sin(θ)   ← North    */

#define DEG_TO_RAD      (3.14159265f / 180.0f)
#define ACCEL_SCALE_MS2 (9.81f / 16384.0f)

/* ── Module state ───────────────────────────────────────────────*/
static RobotPose s_pose  = {0.0f, 0.0f, 0.0f};
static float     s_vx    = 0.0f;   /* East  velocity m/s          */
static float     s_vy    = 0.0f;   /* North velocity m/s          */
static float     s_speed = 0.0f;   /* scalar m/s                  */

/* Velocity decay factor per update (combats double-integration drift).
   At 100 Hz: 0.995^100 ≈ 0.60 per second → velocity halves in ~1.4s
   Increase towards 1.0 for faster motion; decrease for quicker decay. */
#define VEL_DECAY  0.995f

/* ─────────────────────────────────────────────────────────────── */
void ODOM_Init(void)
{
    /* Start robot at centre of the 100×100 map (15.0 m, 15.0 m)  */
    s_pose.x     = 15.0f;
    s_pose.y     = 15.0f;
    s_pose.theta = 0.0f;
    s_vx = s_vy = s_speed = 0.0f;
}

/* ─────────────────────────────────────────────────────────────── */
/*  ODOM_Update  (encoder-based — kept for future motor use)        */
/* ─────────────────────────────────────────────────────────────── */
void ODOM_Update(float left_dist, float right_dist, float dt)
{
    (void)dt;
    float dist   = (left_dist + right_dist) * 0.5f;
    float dtheta = (right_dist - left_dist) / 0.20f;   /* wheel_base = 20 cm */

    float theta_rad = s_pose.theta * DEG_TO_RAD;
    s_pose.x    += dist * sinf(theta_rad);
    s_pose.y    += dist * cosf(theta_rad);
    s_pose.theta = fmodf(s_pose.theta + dtheta * (180.0f / 3.14159265f), 360.0f);
    if (s_pose.theta < 0) s_pose.theta += 360.0f;
}

/* ─────────────────────────────────────────────────────────────── */
/*  ODOM_UpdateIMU                                                  */
/*  heading_deg: absolute heading from STABLE (tilt-compensated).   */
/*                                                                  */
/*  Step 1: update heading from magnetometer.                        */
/*  Step 2: subtract static gravity from accelerometer.             */
/*  Step 3: apply dead-band to reject sensor noise at rest.         */
/*  Step 4: rotate body acceleration into world frame.              */
/*  Step 5: integrate → velocity → position.                        */
/* ─────────────────────────────────────────────────────────────── */
void ODOM_UpdateIMU(const MPU6500_RawData *imu,
                     float heading_deg, float dt)
{
    /* ── 1. Update heading from magnetometer ─────────────────── */
    s_pose.theta = heading_deg;

    /* ── 2. Convert raw accelerometer to m/s² ────────────────── */
    float ax_body = imu->ax * ACCEL_SCALE_MS2;   /* forward        */
    float ay_body = imu->ay * ACCEL_SCALE_MS2;   /* left           */
    /* az contains gravity; for a flat floor robot az ≈ ±9.81 m/s² */
    /* We only use horizontal axes for position estimation          */

    /* ── 3. Dead-band ────────────────────────────────────────── */
    if (fabsf(ax_body) < ODOM_ACCEL_DEADBAND_MS2) ax_body = 0.0f;
    if (fabsf(ay_body) < ODOM_ACCEL_DEADBAND_MS2) ay_body = 0.0f;

    /* ── 4. Rotate body → world (compass heading convention) ─── */
    float hr = heading_deg * DEG_TO_RAD;
    float ax_world =  ax_body * sinf(hr) + ay_body * cosf(hr);  /* East  */
    float ay_world =  ax_body * cosf(hr) - ay_body * sinf(hr);  /* North */

    /* ── 5. Integrate velocity ───────────────────────────────── */
    s_vx = s_vx * VEL_DECAY + ax_world * dt;
    s_vy = s_vy * VEL_DECAY + ay_world * dt;

    /* Clamp to believable hand-push speed                         */
    if (fabsf(s_vx) > ODOM_SPEED_CLAMP_MS) s_vx = 0.0f;
    if (fabsf(s_vy) > ODOM_SPEED_CLAMP_MS) s_vy = 0.0f;

    /* Scalar speed for display                                    */
    s_speed = sqrtf(s_vx * s_vx + s_vy * s_vy);

    /* ── 6. Integrate position ───────────────────────────────── */
    s_pose.x += s_vx * dt;
    s_pose.y += s_vy * dt;

    /* Clamp position to valid map range (0–30 m)                  */
    if (s_pose.x < 0.0f)  s_pose.x = 0.0f;
    if (s_pose.y < 0.0f)  s_pose.y = 0.0f;
    if (s_pose.x > 30.0f) s_pose.x = 30.0f;
    if (s_pose.y > 30.0f) s_pose.y = 30.0f;
}

/* ─────────────────────────────────────────────────────────────── */
void ODOM_SetPose(float x, float y, float theta_deg)
{
    s_pose.x     = x;
    s_pose.y     = y;
    s_pose.theta = theta_deg;
}

void ODOM_ResetVelocity(void)
{
    s_vx = s_vy = s_speed = 0.0f;
}

RobotPose ODOM_GetPose(void)  { return s_pose;  }
float     ODOM_GetSpeed(void) { return s_speed; }
