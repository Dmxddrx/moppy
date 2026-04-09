#include "general.h"
#include "lidar.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ── I2C handles ─────────────────────────────────────────────── */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* ── Module-private state ───────────────────────────────────────*/
static uint32_t s_last_oled_ms = 0;
static uint32_t s_last_logic_ms  = 0;

/* LiDAR Caching & Round Robin State */
static uint32_t s_last_lidar_ms = 0;
static uint8_t  s_lidar_idx     = 0;
static float    s_lidar_dist[4] = {-1.0f, -1.0f, -1.0f, -1.0f};

/* Simple State Machine for pure LiDAR navigation */
typedef enum {
    AVOID_FORWARD,
    AVOID_TURN_LEFT,
    AVOID_TURN_RIGHT,
    AVOID_BACKWARD,
    AVOID_STOP
} AvoidState;

static AvoidState s_avoid_state = AVOID_FORWARD;
static uint32_t s_turn_timer = 0; /* Used to force the robot to turn for 1 second */

/* Variables to share motor speeds with the OLED display */
static int s_disp_l_pwm = 0;
static int s_disp_r_pwm = 0;

/* ========================================================================= */
/* BLOCKED OUT OLD COMPLEX LOGIC (Kept for future use)                       */
/* ========================================================================= */
#if 0
Map g_map;
static MPU6050_RawData  s_imu_raw  = {0};
static HMC5883L_RawData s_mag_raw  = {0};
static Orientation      s_orient   = {0};
static RobotPose        s_pose     = {0};
static uint8_t s_mpu_ok = 0;
static uint8_t s_hmc_ok = 0;

static void render_map_page(void) { /* ... */ }
static void render_imu_page(void) { /* ... */ }
static void render_position_page(void) { /* ... */ }
void GENERAL_I2C_Scan(void) { /* ... */ }
void GENERAL_ResetPose(void) { /* ... */ }
#endif
/* ========================================================================= */

/* ═══════════════════════════════════════════════════════════════ */
/* HELPER: Format LiDAR distance strings cleanly for the Dashboard */
/* ═══════════════════════════════════════════════════════════════ */
static void format_dist(float dist, char* out_str)
{
    if (dist <= -1.5f) {
        strcpy(out_str, "NO ");
    } else if (dist < 0.0f) {
        strcpy(out_str, "OUT");
    } else {
        snprintf(out_str, 8, "%.2f", (double)dist); /* Format to 2 decimal places to save screen space */
    }
}

/* ═══════════════════════════════════════════════════════════════ */
/* UNIFIED DASHBOARD PAGE (LiDAR + Motors)                         */
/* ═══════════════════════════════════════════════════════════════ */
static void render_dashboard_page(void)
{
    char buf[32];
    char str_F[8], str_R[8], str_B[8], str_L[8];

    OLED_Clear();

    /* 1. Format all 4 distances */
    format_dist(s_lidar_dist[0], str_F);
    format_dist(s_lidar_dist[1], str_R);
    format_dist(s_lidar_dist[2], str_B);
    format_dist(s_lidar_dist[3], str_L);

    /* 2. Print LiDAR Row 1 (Front and Back) */
    snprintf(buf, sizeof(buf), "F:%-5s  B:%-5s", str_F, str_B);
    OLED_Print(0, 0, buf);

    /* 3. Print LiDAR Row 2 (Right and Left) */
    snprintf(buf, sizeof(buf), "R:%-5s  L:%-5s", str_R, str_L);
    OLED_Print(0, 12, buf);

    /* 4. Screen Separator */
    OLED_Print(0, 24, "----------------");

    /* 5. Print Motor PWMs */
    snprintf(buf, sizeof(buf), "PWM L:%-4d R:%-4d", s_disp_l_pwm, s_disp_r_pwm);
    OLED_Print(0, 36, buf);

    /* 6. Print Current Action State */
    const char* state_str = "STOPPED";
    if (s_avoid_state == AVOID_FORWARD)      state_str = "FORWARD";
    else if (s_avoid_state == AVOID_BACKWARD) state_str = "REVERSING";
    else if (s_avoid_state == AVOID_TURN_LEFT) state_str = "TURN LEFT";
    else if (s_avoid_state == AVOID_TURN_RIGHT) state_str = "TURN RIGHT";
    else if (s_avoid_state == AVOID_STOP)     state_str = "EMERG STOP!";

    OLED_Print(0, 50, "ACT: ");
    OLED_Print(36, 50, state_str);

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
    OLED_Print(20, 44, "Booting...");
    OLED_Update();

    /* ── Initialize ONLY what we need right now ──────────────── */
    LIDAR_Init();

    /* (Button initialization removed to ignore floating pin!) */

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

        /* Read the CACHED LiDAR distances */
        float dist_F = s_lidar_dist[0]; /* SR1: Front */
        float dist_R = s_lidar_dist[1]; /* SR2: Right */
        float dist_B = s_lidar_dist[2]; /* SR3: Back  */
        float dist_L = s_lidar_dist[3]; /* SR4: Left  */

        /* Check Obstacles: True if distance is between 0cm and 15cm */
        /* (We ignore -1.0 and -2.0 because those mean clear path or unplugged) */
        int obs_F = (dist_F >= 0.0f && dist_F <= 0.15f);
        int obs_R = (dist_R >= 0.0f && dist_R <= 0.15f);
        int obs_B = (dist_B >= 0.0f && dist_B <= 0.15f);
        int obs_L = (dist_L >= 0.0f && dist_L <= 0.15f);

        int left_pwm = 0;
        int right_pwm = 0;

        /* MASTER LOGIC STATE MACHINE */
        switch (s_avoid_state)
        {
            case AVOID_FORWARD:
                if (obs_F) {
                    /* Front is blocked! Decide what to do: */
                    if (!obs_R) {
                        /* Right is clear -> Turn Right for 1 second */
                        s_avoid_state = AVOID_TURN_RIGHT;
                        s_turn_timer = now;
                    }
                    else if (!obs_L) {
                        /* Right is blocked, but Left is clear -> Turn Left for 1 second */
                        s_avoid_state = AVOID_TURN_LEFT;
                        s_turn_timer = now;
                    }
                    else if (!obs_B) {
                        /* Front, Right, and Left are blocked. Back is clear -> Reverse! */
                        s_avoid_state = AVOID_BACKWARD;
                    }
                    else {
                        /* All 4 Lidars are blocked! Emergency Stop! */
                        s_avoid_state = AVOID_STOP;
                    }
                }
                else {
                    /* Front is clear. Drive straight! */
                    left_pwm  = 300;
                    right_pwm = 300;
                }
                break;

            case AVOID_TURN_RIGHT:
                left_pwm  =  250;
                right_pwm = -250;
                /* Keep turning until 1000ms (1 second) has passed */
                if (now - s_turn_timer >= 1000) {
                    s_avoid_state = AVOID_FORWARD; /* Go back to checking sensors */
                }
                break;

            case AVOID_TURN_LEFT:
                left_pwm  = -250;
                right_pwm =  250;
                /* Keep turning until 1000ms (1 second) has passed */
                if (now - s_turn_timer >= 1000) {
                    s_avoid_state = AVOID_FORWARD; /* Go back to checking sensors */
                }
                break;

            case AVOID_BACKWARD:
                left_pwm  = -250;
                right_pwm = -250;

                /* Keep backing up until the sides clear */
                if (!obs_R && !obs_L) {
                    /* Both clear at the same time -> Just Turn Right */
                    s_avoid_state = AVOID_TURN_RIGHT;
                    s_turn_timer = now;
                }
                else if (!obs_R) {
                    /* Only Right cleared -> Turn Right */
                    s_avoid_state = AVOID_TURN_RIGHT;
                    s_turn_timer = now;
                }
                else if (!obs_L) {
                    /* Only Left cleared -> Turn Left */
                    s_avoid_state = AVOID_TURN_LEFT;
                    s_turn_timer = now;
                }
                else if (obs_B) {
                    /* We backed into a wall! All 4 are blocked. STOP. */
                    s_avoid_state = AVOID_STOP;
                }
                break;

            case AVOID_STOP:
                left_pwm = 0;
                right_pwm = 0;
                /* Only escape STOP if the Front magically clears (e.g. someone moved their leg) */
                if (!obs_F) {
                    s_avoid_state = AVOID_FORWARD;
                }
                break;
        }

        /* Drive Physical Motors */
        uint8_t left_dir  = (left_pwm >= 0)  ? MOTOR_FORWARD : MOTOR_BACKWARD;
        uint8_t right_dir = (right_pwm >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

        MOTOR_Set(0, left_dir,  abs(left_pwm));
        MOTOR_Set(1, right_dir, abs(right_pwm));

        /* Save for OLED */
        s_disp_l_pwm = left_pwm;
        s_disp_r_pwm = right_pwm;
    }

    /* ── 2. LiDAR round-robin (Read one sensor every 35 ms) ──── */
    if ((now - s_last_lidar_ms) >= 35) {
        s_last_lidar_ms = now;
        s_lidar_dist[s_lidar_idx] = LIDAR_GetDistance(s_lidar_idx);
        s_lidar_idx = (s_lidar_idx + 1) % 4;
    }

    /* ── 3. OLED refresh (200 ms = 5 Hz) ─────────────────────── */
    if ((now - s_last_oled_ms) >= 200) {
        s_last_oled_ms = now;

        /* Force exactly one screen, ignoring any floating buttons! */
        render_dashboard_page();
    }
}
