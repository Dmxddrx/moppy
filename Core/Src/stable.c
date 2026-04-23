#include "stable.h"
#include <math.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* ── Tuning ─────────────────────────────────────────────────────
   ALPHA: weight given to gyro integration (0.95–0.98 is typical)
          higher → smoother but slower to correct gyro drift
   MAG_DECLINATION: magnetic declination for your location (degrees)
          positive = East, negative = West
          Sri Lanka ≈ –0.4° — close enough to 0 to ignore          */
#define ALPHA            0.97f
#define MAG_DECLINATION  0.0f      /* adjust to your location       */
#define RAD_TO_DEG      (180.0f / 3.14159265f)
#define DEG_TO_RAD      (3.14159265f / 180.0f)

/* Scale factors (must match MPU6050.c settings)                  */
#define ACCEL_SCALE_MS2  (9.81f / 16384.0f)
#define GYRO_SCALE_DPS   (1.0f  / 131.0f)

static Orientation s_orient = {0.0f, 0.0f, 0.0f};
static uint8_t     s_yaw_initialised = 0;

/* ─────────────────────────────────────────────────────────────── */
void STABLE_Init(void)
{
    s_orient.roll  = 0.0f;
    s_orient.pitch = 0.0f;
    s_orient.yaw   = 0.0f;
    s_yaw_initialised = 0;
}

/* ─────────────────────────────────────────────────────────────── */
/*  STABLE_Update                                                   */
/*  dt: time since last call, seconds (e.g. 0.01 for 100 Hz)       */
/*                                                                  */
/*  Algorithm:                                                      */
/*  1. Accel  → absolute roll / pitch                               */
/*  2. Gyro   → integrate roll / pitch / yaw rate                   */
/*  3. Complementary filter merges 1+2 for roll & pitch             */
/*  4. Tilt-compensate mag field using roll & pitch                  */
/*  5. Complementary filter merges mag heading with gyro yaw         */
/* ─────────────────────────────────────────────────────────────── */
void STABLE_Update(MPU6050_RawData *imu, HMC5883L_RawData *mag, float dt)
{
    /* ── Convert raw to physical ─────────────────────────────── */
    float ax = imu->ax * ACCEL_SCALE_MS2;
    float ay = imu->ay * ACCEL_SCALE_MS2;
    float az = imu->az * ACCEL_SCALE_MS2;
    float gx = imu->gx * GYRO_SCALE_DPS;   /* deg/s — roll rate   */
    float gy = imu->gy * GYRO_SCALE_DPS;   /* deg/s — pitch rate  */
    float gz = imu->gz * GYRO_SCALE_DPS;   /* deg/s — yaw rate    */

    /* ── Accelerometer absolute roll / pitch (deg) ───────────── */
    float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    /* ── Gyro-integrated roll / pitch ────────────────────────── */
    float gyro_roll  = s_orient.roll  + gx * dt;
    float gyro_pitch = s_orient.pitch + gy * dt;

    /* ── Complementary filter — roll / pitch ─────────────────── */
    s_orient.roll  = ALPHA * gyro_roll  + (1.0f - ALPHA) * accel_roll;
    s_orient.pitch = ALPHA * gyro_pitch + (1.0f - ALPHA) * accel_pitch;

    /* ── Tilt-compensated magnetometer ───────────────────────── */
    float roll_r  = s_orient.roll  * DEG_TO_RAD;
    float pitch_r = s_orient.pitch * DEG_TO_RAD;

    float mxf = (float)mag->mx;
    float myf = (float)mag->my;
    float mzf = (float)mag->mz;

    /* Project mag field into horizontal plane                     */
    float mx2 =  mxf * cosf(pitch_r)
               + mzf * sinf(pitch_r);
    float my2 =  mxf * sinf(roll_r) * sinf(pitch_r)
               + myf * cosf(roll_r)
               - mzf * sinf(roll_r) * cosf(pitch_r);

    /* Compass heading from tilt-compensated field                 */
    float mag_heading = HMC5883L_GetHeading(mx2, my2) + MAG_DECLINATION;
    if (mag_heading <   0.0f) mag_heading += 360.0f;
    if (mag_heading >= 360.0f) mag_heading -= 360.0f;

    if (!s_yaw_initialised) {
        /* Bootstrap yaw from mag on first valid reading           */
        s_orient.yaw   = mag_heading;
        s_yaw_initialised = 1;
        return;
    }

    /* ── Gyro-integrated yaw ─────────────────────────────────── */
    float gyro_yaw = s_orient.yaw + gz * dt;
    if (gyro_yaw <   0.0f) gyro_yaw += 360.0f;
    if (gyro_yaw >= 360.0f) gyro_yaw -= 360.0f;

    /* ── Complementary filter — yaw (handle 0/360 wrap) ─────── */
    float diff = mag_heading - gyro_yaw;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    s_orient.yaw = gyro_yaw + (1.0f - ALPHA) * diff;
    if (s_orient.yaw <   0.0f) s_orient.yaw += 360.0f;
    if (s_orient.yaw >= 360.0f) s_orient.yaw -= 360.0f;
}

/* ─────────────────────────────────────────────────────────────── */
Orientation STABLE_GetOrientation(void)
{
    return s_orient;
}
