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

/* Store RAW uint16_t values (Millimeters) */
static uint32_t s_last_lidar_ms = 0;
static uint8_t  s_lidar_idx     = 0;
static uint16_t s_lidar_raw[4]  = {65535, 65535, 65535, 65535};

/* Smoothing Hit Counters to eliminate false positive stops! */
static uint8_t s_lidar_hits[4]  = {0, 0, 0, 0};

/* Simple State Machine for pure LiDAR navigation */
typedef enum {
    AVOID_FORWARD,
    AVOID_CONFIRMING, /* NEW: The 1-second evaluation pause! */
    AVOID_TURN_LEFT,
    AVOID_TURN_RIGHT,
    AVOID_BACKWARD,
    AVOID_STOP
} AvoidState;

static AvoidState s_avoid_state = AVOID_FORWARD;
static uint32_t s_turn_timer = 0;
static uint32_t s_pause_timer = 0; /* NEW: Tracks the 1-second pause */

/* Variables to share motor speeds with the OLED display */
static int s_disp_l_pwm = 0;
static int s_disp_r_pwm = 0;

/* ═══════════════════════════════════════════════════════════════ */
/* HELPER: Filter out hardware errors and format the Millimeters!  */
/* ═══════════════════════════════════════════════════════════════ */
static void format_dist_mm(uint16_t raw_dist, char* out_str)
{
    if (raw_dist == 65535) {
        strcpy(out_str, "NO  ");
    } else if (raw_dist >= 8190 || raw_dist == 0) {
        strcpy(out_str, "OUT ");
    } else {
        snprintf(out_str, 8, "%u", raw_dist);
    }
}

/* ═══════════════════════════════════════════════════════════════ */
/* UNIFIED DASHBOARD PAGE (Millimeters + Motors)                   */
/* ═══════════════════════════════════════════════════════════════ */
static void render_dashboard_page(void)
{
    char buf[32];
    char str_F[8], str_R[8], str_B[8], str_L[8];

    OLED_Clear();

    format_dist_mm(s_lidar_raw[0], str_F);
    format_dist_mm(s_lidar_raw[1], str_R);
    format_dist_mm(s_lidar_raw[2], str_B);
    format_dist_mm(s_lidar_raw[3], str_L);

    snprintf(buf, sizeof(buf), "F:%-5s  B:%-5s", str_F, str_B);
    OLED_Print(0, 0, buf);

    snprintf(buf, sizeof(buf), "R:%-5s  L:%-5s", str_R, str_L);
    OLED_Print(0, 12, buf);

    OLED_Print(0, 24, "----------------");

    snprintf(buf, sizeof(buf), "PWM L:%-4d R:%-4d", s_disp_l_pwm, s_disp_r_pwm);
    OLED_Print(0, 36, buf);

    const char* state_str = "STOPPED";
    if (s_avoid_state == AVOID_FORWARD)        state_str = "FORWARD";
    else if (s_avoid_state == AVOID_CONFIRMING) state_str = "WAITING...";
    else if (s_avoid_state == AVOID_BACKWARD)   state_str = "REVERSING";
    else if (s_avoid_state == AVOID_TURN_LEFT)  state_str = "TURN LEFT";
    else if (s_avoid_state == AVOID_TURN_RIGHT) state_str = "TURN RIGHT";
    else if (s_avoid_state == AVOID_STOP)       state_str = "EMERG STOP!";

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

        /* Evaluate Obstacles based on the SMOOTHED CONFIRMATION HITS! */
        int obs_F = (s_lidar_hits[0] >= 2);
        int obs_R = (s_lidar_hits[1] >= 2);
        int obs_B = (s_lidar_hits[2] >= 2);
        int obs_L = (s_lidar_hits[3] >= 2);

        int left_pwm = 0;
        int right_pwm = 0;

        /* MASTER LOGIC STATE MACHINE */
        switch (s_avoid_state)
        {
            case AVOID_FORWARD:
                if (obs_F) {
                    /* Front is blocked! Hit the brakes and start the 1-second timer */
                    s_avoid_state = AVOID_CONFIRMING;
                    s_pause_timer = now;
                } else {
                    /* Front is clear. Drive straight! */
                    left_pwm  = 300;
                    right_pwm = 300;
                }
                break;

            case AVOID_CONFIRMING:
                left_pwm  = 0;
                right_pwm = 0;

                /* Wait exactly 1000ms (1 second) before making a decision */
                if (now - s_pause_timer >= 1000) {
                    if (obs_F) {
                        /* The obstacle is STILL there. It's a permanent wall! Turn! */
                        if (!obs_R) {
                            s_avoid_state = AVOID_TURN_RIGHT;
                            s_turn_timer = now;
                        } else if (!obs_L) {
                            s_avoid_state = AVOID_TURN_LEFT;
                            s_turn_timer = now;
                        } else if (!obs_B) {
                            s_avoid_state = AVOID_BACKWARD;
                        } else {
                            s_avoid_state = AVOID_STOP;
                        }
                    } else {
                        /* The obstacle moved! (e.g. a person walked away) Resume driving! */
                        s_avoid_state = AVOID_FORWARD;
                    }
                }
                break;

            case AVOID_TURN_RIGHT:
                left_pwm  =  350;
                right_pwm = -350;
                if (now - s_turn_timer >= 1000) s_avoid_state = AVOID_FORWARD;
                break;

            case AVOID_TURN_LEFT:
                left_pwm  = -350;
                right_pwm =  350;
                if (now - s_turn_timer >= 1000) s_avoid_state = AVOID_FORWARD;
                break;

            case AVOID_BACKWARD:
                left_pwm  = -300;
                right_pwm = -300;
                if (!obs_R && !obs_L) {
                    s_avoid_state = AVOID_TURN_RIGHT;
                    s_turn_timer = now;
                } else if (!obs_R) {
                    s_avoid_state = AVOID_TURN_RIGHT;
                    s_turn_timer = now;
                } else if (!obs_L) {
                    s_avoid_state = AVOID_TURN_LEFT;
                    s_turn_timer = now;
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

        /* Drive Physical Motors */
        uint8_t left_dir  = (left_pwm >= 0)  ? MOTOR_FORWARD : MOTOR_BACKWARD;
        uint8_t right_dir = (right_pwm >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

        MOTOR_Set(0, left_dir,  abs(left_pwm));
        MOTOR_Set(1, right_dir, abs(right_pwm));

        s_disp_l_pwm = left_pwm;
        s_disp_r_pwm = right_pwm;
    }

    /* ── 2. LiDAR round-robin (Read one sensor every 35 ms) ──── */
    if ((now - s_last_lidar_ms) >= 35) {
        s_last_lidar_ms = now;

        uint16_t dist = LIDAR_GetFilteredDistance(s_lidar_idx);
        s_lidar_raw[s_lidar_idx] = dist;

        /* CONFIRMATION FILTER: Valid obstacle between 40mm and 200mm. */
        if (dist > 40 && dist <= 200) {
            if (s_lidar_hits[s_lidar_idx] < 2) {
                s_lidar_hits[s_lidar_idx]++;
            }
        } else {
            s_lidar_hits[s_lidar_idx] = 0;
        }

        s_lidar_idx = (s_lidar_idx + 1) % 4;
    }

    /* ── 3. OLED refresh (200 ms = 5 Hz) ─────────────────────── */
    if ((now - s_last_oled_ms) >= 200) {
        s_last_oled_ms = now;
        render_dashboard_page();
    }
}
