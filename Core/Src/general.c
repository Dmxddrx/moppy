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

/* Simple State Machine for pure LiDAR navigation */
typedef enum {
    AVOID_FORWARD,
    AVOID_TURN_LEFT,
    AVOID_TURN_RIGHT,
    AVOID_BACKWARD,
    AVOID_STOP
} AvoidState;

static AvoidState s_avoid_state = AVOID_FORWARD;
static uint32_t s_turn_timer = 0;

/* Variables to share motor speeds with the OLED display */
static int s_disp_l_pwm = 0;
static int s_disp_r_pwm = 0;

/* ═══════════════════════════════════════════════════════════════ */
/* HELPER: Filter out hardware errors and format the Millimeters!  */
/* ═══════════════════════════════════════════════════════════════ */
static void format_dist_mm(uint16_t raw_dist, char* out_str)
{
    if (raw_dist == 65535) {
        /* I2C disconnected or sensor timeout */
        strcpy(out_str, "NO  ");
    } else if (raw_dist >= 8190 || raw_dist == 0) {
        /* Sensor is working but path is clear, or material is dark */
        strcpy(out_str, "OUT ");
    } else {
        /* Valid measurement! Print the raw Millimeter integer */
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

    /* 1. Format the raw numbers into safe strings */
    format_dist_mm(s_lidar_raw[0], str_F);
    format_dist_mm(s_lidar_raw[1], str_R);
    format_dist_mm(s_lidar_raw[2], str_B);
    format_dist_mm(s_lidar_raw[3], str_L);

    /* 2. Print LiDAR Rows */
    snprintf(buf, sizeof(buf), "F:%-5s  B:%-5s", str_F, str_B);
    OLED_Print(0, 0, buf);

    snprintf(buf, sizeof(buf), "R:%-5s  L:%-5s", str_R, str_L);
    OLED_Print(0, 12, buf);

    /* 3. Screen Separator */
    OLED_Print(0, 24, "----------------");

    /* 4. Print Motor PWMs */
    snprintf(buf, sizeof(buf), "PWM L:%-4d R:%-4d", s_disp_l_pwm, s_disp_r_pwm);
    OLED_Print(0, 36, buf);

    /* 5. Print Current Action State */
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

        /* Extract Raw Millimeters */
        uint16_t rF = s_lidar_raw[0];
        uint16_t rR = s_lidar_raw[1];
        uint16_t rB = s_lidar_raw[2];
        uint16_t rL = s_lidar_raw[3];

        /* Check Obstacles: 15cm = 150mm. Only trigger if greater than 0 and less than 150! */
        int obs_F = (rF > 30 && rF <= 150);
        int obs_R = (rR > 30 && rR <= 150);
        int obs_B = (rB > 30 && rB <= 150);
        int obs_L = (rL > 30 && rL <= 150);

        int left_pwm = 0;
        int right_pwm = 0;

        /* MASTER LOGIC STATE MACHINE */
        switch (s_avoid_state)
        {
            case AVOID_FORWARD:
                if (obs_F) {
                    /* Front is blocked! Decide what to do: */
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
                    /* Front is clear. Drive straight! */
                    left_pwm  = 300;
                    right_pwm = 300;
                }
                break;

            case AVOID_TURN_RIGHT:
                left_pwm  =  250;
                right_pwm = -250;
                /* Keep turning until 1000ms (1 second) has passed */
                if (now - s_turn_timer >= 1000) s_avoid_state = AVOID_FORWARD;
                break;

            case AVOID_TURN_LEFT:
                left_pwm  = -250;
                right_pwm =  250;
                if (now - s_turn_timer >= 1000) s_avoid_state = AVOID_FORWARD;
                break;

            case AVOID_BACKWARD:
                left_pwm  = -250;
                right_pwm = -250;
                /* Escape reversing if sides clear up */
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

            /* The brain just asks for the filtered data, no math required here! */
            s_lidar_raw[s_lidar_idx] = LIDAR_GetFilteredDistance(s_lidar_idx);
            s_lidar_idx = (s_lidar_idx + 1) % 4;
        }

    /* ── 3. OLED refresh (200 ms = 5 Hz) ─────────────────────── */
    if ((now - s_last_oled_ms) >= 200) {
        s_last_oled_ms = now;
        render_dashboard_page();
    }
}
