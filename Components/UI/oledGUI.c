#include "oledGUI.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Extern Variables from general.c ─────────────────────────── */
/* These tell the compiler "these variables exist somewhere else" */
extern Map g_map;
extern uint16_t s_lidar_raw[4];
extern float s_current_yaw;
extern CoverageCmd s_current_cmd;
extern int s_disp_l_pwm;
extern int s_disp_r_pwm;
extern HMC5883L_RawData s_last_mag;
extern MPU6050_PhysData s_last_imu_phys;

extern uint8_t wifi_is_connected;
extern uint8_t esp_is_ready;
extern char    current_ip[16];
extern char    current_ssid[32];
extern char    current_pc_ip[16];

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
void render_dashboard_page(void)
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

    OLED_DrawOutline(0, 0, 100, 22);

    snprintf(buf, sizeof(buf), "F:%-5s  B:%-5s", str_F, str_B);
    OLED_Print(4, 3, buf);

    snprintf(buf, sizeof(buf), "R:%-5s  L:%-5s", str_R, str_L);
    OLED_Print(4, 12, buf);

    /* SHOW THE 9-AXIS HEADING AND THE MAP COORDINATES! */
	snprintf(buf, sizeof(buf), "H:%-3.0f X:%-4.1f Y:%-4.1f", s_current_yaw, pose.x, pose.y);
	OLED_Print(0, 30, buf);

    snprintf(buf, sizeof(buf), "PWM L:%-4d R:%-4d", s_disp_l_pwm, s_disp_r_pwm);
    OLED_Print(0, 42, buf);

    /* UPGRADED: Map the new Coverage Commands to display strings */
        const char* state_str = "IDLE";
        if (s_current_cmd == CMD_DRIVE_FORWARD)     state_str = "FORWARD";
		else if (s_current_cmd == CMD_REVERSE)      state_str = "REVERSE";
		else if (s_current_cmd == CMD_TURN_LEFT)    state_str = "TURN L";
		else if (s_current_cmd == CMD_TURN_RIGHT)   state_str = "TURN R";
		else if (s_current_cmd == CMD_STOP)         state_str = "STOPPED";

    OLED_Print(0, 55, "ACT: ");
    OLED_Print(36, 55, state_str);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* COMPASS PAGE (1)                                                */
/* ═══════════════════════════════════════════════════════════════ */
void render_compass_page(void)
{
    char buf[32];
    OLED_Clear();

    /* 1. Left Side: The 64x64 Compass Circle */
    uint8_t cx = 32; /* Center X */
    uint8_t cy = 32; /* Center Y */
    uint8_t r  = 28; /* Radius   */

    OLED_DrawCircle(cx, cy, r);
    OLED_DrawCircle(cx, cy, 2); /* Draw a tiny hub in the middle */
    OLED_Print(30, 0, "N");
    OLED_Print(30, 56, "S");

    float rad = s_current_yaw * (M_PI / 180.0f);
    uint8_t end_x = (uint8_t)(cx + (r * sinf(rad)));
	uint8_t end_y = (uint8_t)(cy - (r * cosf(rad)));
    OLED_DrawLine(cx, cy, end_x, end_y);

    Orientation current_3d = STABLE_GetOrientation();
    OLED_Print(68, 0, "COMPASS");
    OLED_DrawLine(68, 10, 128, 10);
    snprintf(buf, sizeof(buf), "%3.0f deg", s_current_yaw);
    OLED_Print(68, 15, buf);
    snprintf(buf, sizeof(buf), "P: %2.0f", current_3d.pitch);
    OLED_Print(68, 30, buf);
    snprintf(buf, sizeof(buf), "R: %2.0f", current_3d.roll);
    OLED_Print(68, 40, buf);

	const char* state_str = "IDLE";
	if (s_current_cmd == CMD_DRIVE_FORWARD)     state_str = "FORWARD";
	else if (s_current_cmd == CMD_REVERSE)      state_str = "REVERSE";
	else if (s_current_cmd == CMD_TURN_LEFT)    state_str = "TURN L";
	else if (s_current_cmd == CMD_TURN_RIGHT)   state_str = "TURN R";
	else if (s_current_cmd == CMD_STOP)         state_str = "STOPPED";
	OLED_Print(68, 55, state_str);

    OLED_Update();
}


/* ═══════════════════════════════════════════════════════════════ */
/* MAP DISPLAY PAGE (2)                                            */
/* ═══════════════════════════════════════════════════════════════ */
void render_map_page(void)
{
    OLED_Clear();

    /* 1. Get the viewport (the 64x64 slice of the map around the robot) */
    int vx0, vy0;
    Map_GetViewport(&g_map, &vx0, &vy0);

    /* NEW: Create a blink state that flips between 1 and 0 every 400ms */
    uint8_t blink_state = (HAL_GetTick() / 400) % 2;

    /* 2. Draw the Grid Pixels */
    for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 64; x++) {
            uint8_t cell = Map_GetCellByIndex(&g_map, vx0 + x, vy0 + y);

            if      (cell >= CELL_OBSTACLE_BASE)         OLED_DrawPixel(x, y, 0);
			else if (cell == CELL_SEEN_FREE)             OLED_DrawPixel(x, y, blink_state);
			else if (cell > 0 && cell < CELL_OBSTACLE_BASE) OLED_DrawPixel(x, y, 1); // Valid floor
        }
    }

    /* 3. Draw Robot Icon (With Directional Nose!) */
	int rx = Map_RobotCellX(&g_map) - vx0;
	int ry = Map_RobotCellY(&g_map) - vy0;

	/* Draw a small hollow square for the robot body */
	OLED_DrawPixel(rx-1, ry-1, 1); OLED_DrawPixel(rx+1, ry-1, 1);
	OLED_DrawPixel(rx-1, ry+1, 1); OLED_DrawPixel(rx+1, ry+1, 1);
	OLED_DrawPixel(rx, ry, !blink_state); /* Flashing core */

	/* Draw a line pointing exactly where the compass is facing! */
	float rad = s_current_yaw * (M_PI / 180.0f);
	int nose_x = rx + (int)(5.0f * sinf(rad));
	int nose_y = ry - (int)(5.0f * cosf(rad)); /* -Y is UP on the OLED */

	OLED_DrawLine(rx, ry, nose_x, nose_y);

    /* 4. Right side Telemetry Info */
    OLED_Print(68, 0, "MAP");
    OLED_DrawLine(68, 9, 128, 9);
    char buf[16];
    RobotPose pose = ODOM_GetPose();
    snprintf(buf, sizeof(buf), "X:%0.1f", pose.x);
    OLED_Print(68, 12, buf);
    snprintf(buf, sizeof(buf), "Y:%0.1f", pose.y);
    OLED_Print(68, 22, buf);

    float progress = (g_map.cells_cleaned / 10000.0f) * 100.0f;
    snprintf(buf, sizeof(buf), "Cln:%0.1f%%", progress);
    OLED_Print(68, 35, buf);

	snprintf(buf, sizeof(buf), "%3.0f deg", s_current_yaw);
	OLED_Print(68, 45, buf);

    /* Grab the Captain's Current Command */
	const char* state_str = "IDLE";
	if (s_current_cmd == CMD_DRIVE_FORWARD)     state_str = "FORWARD";
	else if (s_current_cmd == CMD_REVERSE)      state_str = "REVERSE";
	else if (s_current_cmd == CMD_TURN_LEFT)    state_str = "TURN L";
	else if (s_current_cmd == CMD_TURN_RIGHT)   state_str = "TURN R";
	else if (s_current_cmd == CMD_STOP)         state_str = "STOPPED";
	OLED_Print(68, 55, state_str);



    OLED_Update();
}

#if WIFI_BRIDGE
/* ═══════════════════════════════════════════════════════════════ */
/* WI-FI / NETWORK PAGE (3)                                        */
/* ═══════════════════════════════════════════════════════════════ */
void render_wifi_page(void) {
    OLED_Clear();
    OLED_Print(0, 0, "NETWORK");

    OLED_DrawOutline(0, 11, 87, 22);

    OLED_Print(4, 14, "ESP-01S");
    if (esp_is_ready) {
        OLED_Print(4, 24, "READY");
    } else {
        OLED_Print(4, 24, "ERROR");
    }

    OLED_Print(50, 14, "Wi-Fi");
    if (wifi_is_connected) {
        OLED_Print(50, 24, "ONLINE");
        OLED_Print(50, 0, current_ip);
    } else {
        OLED_Print(50, 24, "OFFLINE");
        OLED_Print(50, 0, "Not Assigned");
    }

    if (wifi_is_connected) {
		OLED_Print(0, 38, current_ssid);
		OLED_Print(0, 48, current_pc_ip);
	} else {
		OLED_Print(0, 38, "AP: N/A");
		OLED_Print(0, 48, "IP: 0.0.0.0");
	}

    OLED_Update();
}
#endif

/* ═══════════════════════════════════════════════════════════════ */
/* SENSOR CALIBRATION PAGE (4)                                     */
/* ═══════════════════════════════════════════════════════════════ */
void render_calib_page(void) {
    char buf[32];
    OLED_Clear();

    /* Header */
    OLED_Print(0, 0, "MAG & ACCEL CALIB");
    OLED_DrawLine(0, 10, 128, 10);

    /* Print RAW Mag and Physical Accel side-by-side
       %-5d adds spaces to clear old digits, %5.2f formats to 2 decimal places */

    snprintf(buf, sizeof(buf), "Mx %-5d Ax %5.2f", s_last_mag.mx, s_last_imu_phys.ax);
    OLED_Print(0, 20, buf);

    snprintf(buf, sizeof(buf), "My %-5d Ay %5.2f", s_last_mag.my, s_last_imu_phys.ay);
    OLED_Print(0, 30, buf);

    snprintf(buf, sizeof(buf), "Mz %-5d Az %5.2f", s_last_mag.mz, s_last_imu_phys.az);
    OLED_Print(0, 40, buf);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* ENCODER & MOTOR PAGE (5)                                        */
/* ═══════════════════════════════════════════════════════════════ */
void render_encoder_page(void) {
    char buf[32];
    OLED_Clear();

    /* Header */
    OLED_Print(0, 0, "MOTOR & ENCODERS");
    OLED_DrawLine(0, 10, 128, 10);

    /* Fetch speeds (rad/s or RPM depending on your encoder.c math) */
    float speed_L = ENCODER_GetSpeed(0);
    float speed_R = ENCODER_GetSpeed(1);

    /* Print Left Motor Data */
    OLED_Print(0, 18, "LEFT WHEEL:");
    snprintf(buf, sizeof(buf), "PWM: %-4d", s_disp_l_pwm);
    OLED_Print(10, 28, buf);
    snprintf(buf, sizeof(buf), "SPD: %5.2f", speed_L);
    OLED_Print(64, 28, buf);

    /* Print Right Motor Data */
    OLED_Print(0, 42, "RIGHT WHEEL:");
    snprintf(buf, sizeof(buf), "PWM: %-4d", s_disp_r_pwm);
    OLED_Print(10, 52, buf);
    snprintf(buf, sizeof(buf), "SPD: %5.2f", speed_R);
    OLED_Print(64, 52, buf);

    OLED_Update();
}
