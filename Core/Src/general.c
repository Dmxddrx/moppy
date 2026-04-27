#include "general.h"
#include <stdio.h>   /* Standard I/O: Gives you snprintf() to format text and numbers for your OLED screen */
#include <string.h>  /* String & Memory: Gives you memset() to instantly clear your 10,000-cell map array to 0 */
#include <math.h>    /* Math: Gives you sinf(), cosf(), and atan2f() for your compass and LiDAR trigonometry */
#include <stdlib.h>  /* Standard Library: Gives you abs() to calculate absolute values (like in your Bresenham ray-casting) */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Global Map Instance ───────────────────────────────────────*/
Map g_map;

/* ── I2C handles ─────────────────────────────────────────────── */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* ── Module-private state ───────────────────────────────────────*/
static uint32_t s_last_oled_ms = 0;
static uint32_t s_last_logic_ms  = 0;

/* OLED Page Tracker (0 = Dashboard, 1 = Compass) */
static uint8_t  s_current_page  = 0;

/* Store RAW uint16_t values (Millimeters) */
static uint32_t s_last_lidar_ms = 0;
static uint8_t  s_lidar_idx     = 0;
static uint16_t s_lidar_raw[4]  = {65535, 65535, 65535, 65535};
static uint8_t  s_lidar_hits[4] = {0, 0, 0, 0};

/* IMU Navigation State */
static float s_current_yaw = 0.0f;
static float s_target_yaw  = 0.0f;
static int   s_is_moving   = 0; /* NEW: Tracks if the motors/LEDs are active */

/* Simple State Machine */
typedef enum {
    AVOID_FORWARD,
    AVOID_CONFIRMING,
    AVOID_TURN_LEFT,
    AVOID_TURN_RIGHT,
    AVOID_BACKWARD,
    AVOID_STOP
} AvoidState;

static AvoidState s_avoid_state = AVOID_FORWARD;
static uint32_t s_pause_timer = 0;

/* Variables to share motor speeds with the OLED display */
static int s_disp_l_pwm = 0;
static int s_disp_r_pwm = 0;

/* ═══════════════════════════════════════════════════════════════ */
/* HELPER: Angle Math for perfect 90-degree turns                  */
/* ═══════════════════════════════════════════════════════════════ */
static float wrap360(float a) {
    while(a >= 360.0f) a -= 360.0f;
    while(a <    0.0f) a += 360.0f;
    return a;
}

static float angle_diff(float target, float current) {
    float d = target - current;
    while(d >  180.0f) d -= 360.0f;
    while(d < -180.0f) d += 360.0f;
    return d;
}

/* ═══════════════════════════════════════════════════════════════ */
/* HELPER: Filter out hardware errors and format the Millimeters!  */
/* ═══════════════════════════════════════════════════════════════ */
static void format_dist_mm(uint16_t raw_dist, char* out_str) {
    if (raw_dist == 65535) {
        strcpy(out_str, "NO  ");
    } else if (raw_dist >= 8190 || raw_dist == 0) {
        strcpy(out_str, "OUT ");
    } else {
        snprintf(out_str, 8, "%u", raw_dist);
    }
}

/* ═══════════════════════════════════════════════════════════════ */
/* UNIFIED DASHBOARD PAGE 0                                        */
/* ═══════════════════════════════════════════════════════════════ */
static void render_dashboard_page(void)
{
    char buf[32];
    char str_F[8], str_R[8], str_B[8], str_L[8];

    /* Pull the calculated map coordinates to show on the screen */
	RobotPose pose = ODOM_GetPose();

    OLED_Clear();

    format_dist_mm(s_lidar_raw[0], str_F);
    format_dist_mm(s_lidar_raw[1], str_R);
    format_dist_mm(s_lidar_raw[2], str_B);
    format_dist_mm(s_lidar_raw[3], str_L);

    OLED_DrawOutline(0, 0, 100, 19);

    snprintf(buf, sizeof(buf), "F:%-5s  B:%-5s", str_F, str_B);
    OLED_Print(2, 2, buf);

    snprintf(buf, sizeof(buf), "R:%-5s  L:%-5s", str_R, str_L);
    OLED_Print(2, 11, buf);

    /* SHOW THE 9-AXIS HEADING AND THE MAP COORDINATES! */
	snprintf(buf, sizeof(buf), "H:%-3.0f X:%-4.1f Y:%-4.1f", s_current_yaw, pose.x, pose.y);
	OLED_Print(0, 24, buf);

    snprintf(buf, sizeof(buf), "PWM L:%-4d R:%-4d", s_disp_l_pwm, s_disp_r_pwm);
    OLED_Print(0, 36, buf);

    const char* state_str = "STOPPED";
    if (s_avoid_state == AVOID_FORWARD)        state_str = "FORWARD";
    else if (s_avoid_state == AVOID_CONFIRMING) state_str = "WAITING...";
    else if (s_avoid_state == AVOID_BACKWARD)   state_str = "REVERSING";
    else if (s_avoid_state == AVOID_TURN_LEFT)  state_str = "TURN LEFT";
    else if (s_avoid_state == AVOID_TURN_RIGHT) state_str = "TURN RIGHT";
    else if (s_avoid_state == AVOID_STOP)       state_str = "EMERG STOP!";

    OLED_Print(0, 45, "ACT: ");
    OLED_Print(36, 45, state_str);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* COMPASS PAGE (1)                                                */
/* ═══════════════════════════════════════════════════════════════ */
static void render_compass_page(void)
{
    char buf[32];
    OLED_Clear();

    /* 1. Left Side: The 64x64 Compass Circle */
    uint8_t cx = 32; /* Center X */
    uint8_t cy = 32; /* Center Y */
    uint8_t r  = 28; /* Radius   */

    OLED_DrawCircle(cx, cy, r);
    OLED_DrawCircle(cx, cy, 2); /* Draw a tiny hub in the middle */

    /* The "North" mark at the top of the dial */
    OLED_Print(30, 0, "N");
    OLED_Print(30, 56, "S");

    /* 2. Calculate Compass Needle Math */
    /* 0 degrees is UP (North). Screen coordinates: UP is -Y. */
    float rad = s_current_yaw * (M_PI / 180.0f);

    /* Calculate the tip of the line using Sine and Cosine */
    uint8_t end_x = (uint8_t)(cx + (r * sinf(rad)));
	uint8_t end_y = (uint8_t)(cy - (r * cosf(rad)));

    /* Draw the needle from the center out to the edge */
    OLED_DrawLine(cx, cy, end_x, end_y);

    /* 3. Right Side: The Text Telemetry (Starts at X=68) */
    Orientation current_3d = STABLE_GetOrientation();

    OLED_Print(68, 0, "COMPASS");
    OLED_DrawLine(68, 10, 120, 10); /* Underline */

    snprintf(buf, sizeof(buf), "HDG:");
    OLED_Print(68, 16, buf);
    snprintf(buf, sizeof(buf), "%3.0f deg", s_current_yaw);
    OLED_Print(68, 26, buf);

    snprintf(buf, sizeof(buf), "P: %2.0f", current_3d.pitch);
    OLED_Print(68, 42, buf);

    snprintf(buf, sizeof(buf), "R: %2.0f", current_3d.roll);
    OLED_Print(68, 52, buf);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* GENERAL_Init                                                   */
/* ═══════════════════════════════════════════════════════════════ */
void GENERAL_Init(void)
{
    OLED_Init(&hi2c2);
    OLED_Clear();
    OLED_Print(16, 20, "Floor Robot");

    /* IMPORTANT WARNING FOR THE USER ON THE SCREEN */
    OLED_Print(0, 40, "DO NOT MOVE!");
    OLED_Print(0, 50, "Calibrating IMU...");
    OLED_Update();

    /* Boot up the IMU and Compass */
    BTNS_Init();
    MPU6050_Init();
    HMC5883L_Init();
    STABLE_Init();

    /* NEW: Initialize the map and odometry at (15.0m, 15.0m) */
	ODOM_Init();
	Map_Init(&g_map);

    LIDAR_Init();
    MOTOR_Init();
    MOTORPWM_Init();
    MOTOR_WakeAll();

    HAL_Delay(500);
    OLED_Clear();
    OLED_Update();

    s_last_logic_ms = HAL_GetTick();
    s_last_lidar_ms = HAL_GetTick();
    s_last_oled_ms  = HAL_GetTick();
}

/* ═══════════════════════════════════════════════════════════════ */
/* GENERAL_Update — call from main() while(1) loop               */
/* ═══════════════════════════════════════════════════════════════ */
void GENERAL_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ── 1. Autonomous Loop (10 ms period = 100 Hz) ──────────── */
    if ((now - s_last_logic_ms) >= 10) {
        s_last_logic_ms = now;

        /* A. Read Buttons for UI Changes! */
        BTNS_Update();
        if (BTNS_Get_OLEDPage() == BTN_PRESSED) {
            /* Toggle between Page 0 and Page 1 */
            s_current_page = !s_current_page;
        }

        /* B. Read IMU and update real-time Orientation */
        MPU6050_RawData imu;
        HMC5883L_RawData mag;

        if (MPU6050_ReadRaw(&imu) == HAL_OK) {
            HMC5883L_ReadRaw(&mag);
            STABLE_Update(&imu, &mag, 0.01f); /* 10ms = 0.01 seconds */
            s_current_yaw = STABLE_GetOrientation().yaw;
        }

        /* C. Evaluate Obstacles */
        int obs_F = (s_lidar_hits[0] >= 2);
        int obs_R = (s_lidar_hits[1] >= 2);
        int obs_B = (s_lidar_hits[2] >= 2);
        int obs_L = (s_lidar_hits[3] >= 2);

        int left_pwm = 0;
        int right_pwm = 0;

        /* D. MASTER LOGIC STATE MACHINE */
        switch (s_avoid_state)
        {
            case AVOID_FORWARD:
                if (obs_F) {
                    s_avoid_state = AVOID_CONFIRMING;
                    s_pause_timer = now;
                } else {
                    left_pwm  = 300;
                    right_pwm = 300;
                }
                break;

            case AVOID_CONFIRMING:
                left_pwm  = 0;
                right_pwm = 0;

                if (now - s_pause_timer >= 1000) {
                    if (obs_F) {
                        /* Calculate EXACT 90-degree targets based on current heading! */
                        if (!obs_R) {
                            s_avoid_state = AVOID_TURN_RIGHT;
                            s_target_yaw = wrap360(s_current_yaw + 90.0f);
                        } else if (!obs_L) {
                            s_avoid_state = AVOID_TURN_LEFT;
                            s_target_yaw = wrap360(s_current_yaw - 90.0f);
                        } else if (!obs_B) {
                            s_avoid_state = AVOID_BACKWARD;
                        } else {
                            s_avoid_state = AVOID_STOP;
                        }
                    } else {
                        s_avoid_state = AVOID_FORWARD;
                    }
                }
                break;

            case AVOID_TURN_RIGHT:
                left_pwm  =  350;
                right_pwm = -350;
                /* Stop turning when we are within 5 degrees of the 90-deg target */
                if (fabsf(angle_diff(s_target_yaw, s_current_yaw)) < 5.0f) {
                    s_avoid_state = AVOID_FORWARD;
                }
                break;

            case AVOID_TURN_LEFT:
                left_pwm  = -350;
                right_pwm =  350;
                if (fabsf(angle_diff(s_target_yaw, s_current_yaw)) < 5.0f) {
                    s_avoid_state = AVOID_FORWARD;
                }
                break;

            case AVOID_BACKWARD:
                left_pwm  = -300;
                right_pwm = -300;
                if (!obs_R && !obs_L) {
                    s_avoid_state = AVOID_TURN_RIGHT;
                    s_target_yaw = wrap360(s_current_yaw + 90.0f);
                } else if (!obs_R) {
                    s_avoid_state = AVOID_TURN_RIGHT;
                    s_target_yaw = wrap360(s_current_yaw + 90.0f);
                } else if (!obs_L) {
                    s_avoid_state = AVOID_TURN_LEFT;
                    s_target_yaw = wrap360(s_current_yaw - 90.0f);
                } else if (obs_B) {
                    s_avoid_state = AVOID_STOP;
                }
                break;

            case AVOID_STOP:
                left_pwm = 0;
                right_pwm = 0;
                if (!obs_F) s_avoid_state = AVOID_FORWARD;
                break;
        }

        uint8_t left_dir  = (left_pwm >= 0)  ? MOTOR_FORWARD : MOTOR_BACKWARD;
        uint8_t right_dir = (right_pwm >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

        MOTOR_Set(0, left_dir,  abs(left_pwm));
        MOTOR_Set(1, right_dir, abs(right_pwm));

        s_disp_l_pwm = left_pwm;
        s_disp_r_pwm = right_pwm;

        /* E. ODOMETRY & MAPPING (The Kinematic Simulator) */
        s_is_moving = (left_pwm != 0 || right_pwm != 0);

        if (s_is_moving) {
            Orientation current_3d = STABLE_GetOrientation();
            float pitch_deg = current_3d.pitch;

            float sim_v_left  = (left_pwm  / 999.0f) * 0.4f;
            float sim_v_right = (right_pwm / 999.0f) * 0.4f;

            float gravity_pull = sinf(pitch_deg * (M_PI / 180.0f)) * 0.15f;

            sim_v_left  -= gravity_pull;
            sim_v_right -= gravity_pull;

            if (left_pwm > 0 && sim_v_left < 0.0f) sim_v_left = 0.0f;
            if (right_pwm > 0 && sim_v_right < 0.0f) sim_v_right = 0.0f;
            if (left_pwm < 0 && sim_v_left > 0.0f) sim_v_left = 0.0f;
            if (right_pwm < 0 && sim_v_right > 0.0f) sim_v_right = 0.0f;

            ODOM_UpdateEncoders(sim_v_left, sim_v_right, current_3d.yaw, 0.01f);

            RobotPose pose = ODOM_GetPose();
            Map_UpdateRobotPose(&g_map, pose.x, pose.y, pose.theta);
        }
    }

    /* ── 2. LiDAR round-robin ──── */
    if ((now - s_last_lidar_ms) >= 35) {
        s_last_lidar_ms = now;
        uint16_t dist = LIDAR_GetFilteredDistance(s_lidar_idx);
        s_lidar_raw[s_lidar_idx] = dist;

        if (dist > 40 && dist <= 200) {
            if (s_lidar_hits[s_lidar_idx] < 2) s_lidar_hits[s_lidar_idx]++;
        } else {
            s_lidar_hits[s_lidar_idx] = 0;
        }

        /* Plot LiDAR Obstacles on the Map ONLY if the robot is moving! */
		if (s_is_moving && dist > 40 && dist < 3500) {
			float angle = 0.0f;
			if(s_lidar_idx == 0) angle = 0.0f;   /* Front */
			if(s_lidar_idx == 1) angle = 90.0f;  /* Right */
			if(s_lidar_idx == 2) angle = 180.0f; /* Back  */
			if(s_lidar_idx == 3) angle = 270.0f; /* Left  */

			/* Call the updated LiDAR mapping function! */
			Map_UpdateLiDAR(&g_map, dist / 1000.0f, angle);
		}

        s_lidar_idx = (s_lidar_idx + 1) % 4;
    }

    /* ── 3. OLED refresh ─────────────────────── */
    if ((now - s_last_oled_ms) >= 200) {
        s_last_oled_ms = now;

        /* The Button controls which screen gets rendered! */
        if (s_current_page == 0) {
            render_dashboard_page();
        } else {
            render_compass_page();
        }
    }
}
