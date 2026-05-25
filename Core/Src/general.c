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

/* ── Hardware Handles (Imported from main.c) ──────────────────── */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

/* ── Global Network Tracking Variables ───────────────────────── */
uint8_t wifi_is_connected = 0;
uint8_t esp_is_ready       = 0;
char    current_ip[16]    = "0.0.0.0";

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
static CoverageCmd s_current_cmd = CMD_STOP; /* The Captain's Orders! */
static int s_first_run = 1;
static float s_heading_integral = 0.0f;

/* Store the latest physical IMU data for Wi-Fi broadcast */
static MPU6050_PhysData s_last_imu_phys = {0};

/* Variables to share motor speeds with the OLED display */
static int s_disp_l_pwm = 0;
static int s_disp_r_pwm = 0;

static float angle_diff(float target, float current) {
    float d = target - current;
    while(d >  180.0f) d -= 360.0f;
    while(d < -180.0f) d += 360.0f;
    return d;
}

/* ═══════════════════════════════════════════════════════════════ */
/* TELEMETRY BROADCAST ENGINE                                      */
/* ═══════════════════════════════════════════════════════════════ */
static void send_telemetry_wifi(void) {
    if (!wifi_is_connected) return;

    char tx_buf[512];
    RobotPose pose = ODOM_GetPose();

    snprintf(tx_buf, sizeof(tx_buf),
            "{\"F\":%u, \"R\":%u, \"B\":%u, \"L\":%u, \"yaw\":%.1f, \"X\":%.1f, \"Y\":%.1f, \"pwmL\":%d, \"pwmR\":%d, \"cmd\":%d, \"ax\":%.2f, \"ay\":%.2f, \"az\":%.2f, \"gx\":%.2f, \"gy\":%.2f, \"gz\":%.2f}\n",
            s_lidar_raw[0], s_lidar_raw[1], s_lidar_raw[2], s_lidar_raw[3],
            s_current_yaw, pose.x, pose.y, s_disp_l_pwm, s_disp_r_pwm, s_current_cmd,
            s_last_imu_phys.ax, s_last_imu_phys.ay, s_last_imu_phys.az,
            s_last_imu_phys.gx, s_last_imu_phys.gy, s_last_imu_phys.gz);

        WIFI_SendUDPData(tx_buf);
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
        if (s_current_cmd == CMD_DRIVE_FORWARD)     state_str = "SWEEPING";
        else if (s_current_cmd == CMD_TURN_LEFT)    state_str = "TURNING L";
        else if (s_current_cmd == CMD_TURN_RIGHT)   state_str = "TURNING R";
        else if (s_current_cmd == CMD_STOP)         state_str = "STOPPED";

    OLED_Print(0, 55, "ACT: ");
    OLED_Print(36, 55, state_str);

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
    OLED_Print(30, 0, "N");
    OLED_Print(30, 56, "S");

    float rad = s_current_yaw * (M_PI / 180.0f);
    uint8_t end_x = (uint8_t)(cx + (r * sinf(rad)));
	uint8_t end_y = (uint8_t)(cy - (r * cosf(rad)));
    OLED_DrawLine(cx, cy, end_x, end_y);

    Orientation current_3d = STABLE_GetOrientation();
    OLED_Print(68, 0, "COMPASS");
    snprintf(buf, sizeof(buf), "%3.0f deg", s_current_yaw);
    OLED_Print(68, 13, buf);
    snprintf(buf, sizeof(buf), "P: %2.0f", current_3d.pitch);
    OLED_Print(68, 30, buf);
    snprintf(buf, sizeof(buf), "R: %2.0f", current_3d.roll);
    OLED_Print(68, 40, buf);

	const char* state_str = "IDLE";
	if (s_current_cmd == CMD_DRIVE_FORWARD)     state_str = "SWEEPING";
	else if (s_current_cmd == CMD_TURN_LEFT)    state_str = "TURNING L";
	else if (s_current_cmd == CMD_TURN_RIGHT)   state_str = "TURNING R";
	else if (s_current_cmd == CMD_STOP)         state_str = "STOPPED";
	OLED_Print(68, 55, state_str);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* MAP DISPLAY PAGE (2)                                            */
/* ═══════════════════════════════════════════════════════════════ */
static void render_map_page(void)
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

            if (cell == CELL_OBSTACLE) {
                OLED_DrawPixel(x, y, 0); /* User requested: Obstacles OFF */
            }
            else if (cell == CELL_SEEN_FREE) {
                OLED_DrawPixel(x, y, blink_state); /* Seen but not clean: BLINK */
            }
            else if (cell > 0 && cell < 254) {
                OLED_DrawPixel(x, y, 1); /* Cleaned: Solid ON */
            }
            /* CELL_UNCLEANED (0) remains OFF automatically by OLED_Clear */
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
    char buf[16];
    RobotPose pose = ODOM_GetPose();
    snprintf(buf, sizeof(buf), "X:%0.1f", pose.x);
    OLED_Print(68, 0, buf);
    snprintf(buf, sizeof(buf), "Y:%0.1f", pose.y);
    OLED_Print(68, 10, buf);

    float progress = (g_map.cells_cleaned / 10000.0f) * 100.0f;
    snprintf(buf, sizeof(buf), "Cln:%0.1f%%", progress);
    OLED_Print(68, 25, buf);

    /* Grab the Captain's Current Command */
	const char* state_str = "IDLE";
	if (s_current_cmd == CMD_DRIVE_FORWARD)     state_str = "SWEEPING";
	else if (s_current_cmd == CMD_TURN_LEFT)    state_str = "TURNING L";
	else if (s_current_cmd == CMD_TURN_RIGHT)   state_str = "TURNING R";
	else if (s_current_cmd == CMD_STOP)         state_str = "STOPPED";
	OLED_Print(68, 55, state_str);

	snprintf(buf, sizeof(buf), "%3.0f deg", s_current_yaw);
	OLED_Print(68, 45, buf);

    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* WI-FI / NETWORK PAGE (3)                                        */
/* ═══════════════════════════════════════════════════════════════ */
static void render_wifi_page(void) {
    OLED_Clear();
    OLED_Print(0, 0, "── MOPPY NETWORK ──");

    OLED_Print(0, 20, "ESP-01S Status:");
    if (esp_is_ready) {
        OLED_Print(90, 20, "READY");
    } else {
        OLED_Print(90, 20, "ERROR");
    }

    OLED_Print(0, 32, "Wi-Fi Link:");
    if (wifi_is_connected) {
        OLED_Print(90, 32, "ONLINE");
        OLED_Print(0, 48, "IP Address:");
        OLED_Print(0, 56, current_ip);
    } else {
        OLED_Print(90, 32, "OFFLINE");
        OLED_Print(0, 48, "IP Address:");
        OLED_Print(0, 56, "Not Assigned");
    }
    OLED_Update();
}

/* ═══════════════════════════════════════════════════════════════ */
/* GENERAL_Init                                                   */
/* ═══════════════════════════════════════════════════════════════ */
void GENERAL_Init(void)
{
    OLED_Init(&hi2c2);
    OLED_Clear();
    OLED_Print(16, 10, "Moppy Cleaner");
    OLED_Print(0, 30, "DO NOT MOVE!");
    OLED_Print(0, 45, "Calibrating IMU...");
    OLED_Update();

    BTNS_Init();
    MPU6050_Init();
    HMC5883L_Init();
    STABLE_Init();
	ODOM_Init();
	Map_Init(&g_map);
    LIDAR_Init();
    MOTOR_Init();
    MOTORPWM_Init();
    MOTOR_WakeAll();

    HAL_Delay(500);

    OLED_Clear();
	OLED_Print(0, 0, "WI-FI BRIDGE");
	OLED_Print(0, 20, "Init ESP-01S");
	OLED_Update();

	if (WIFI_Init()) {
		OLED_Print(95, 20, "OK");
		esp_is_ready = 1;
		OLED_Update();
	} else {
		OLED_Print(95, 20, "FAIL");
		esp_is_ready = 0;
		OLED_Update();
		HAL_Delay(2000);
		return; /* Skip AP lock cycle if module is un-responsive */
	}

	/* Connect to designated Access Point */
	OLED_Print(0, 35, "Connecting AP...");
	OLED_Update();

	/* Tries Primary, shifts dynamically to Fallback if needed */
	if (WIFI_Connect(WIFI_SSID_1, WIFI_PASS_1)) {
		OLED_Print(0, 48, "Connected to AP 1!");
		wifi_is_connected = 1;
	} else {
		WIFI_Disconnect();
		HAL_Delay(500);
		if (WIFI_Connect(WIFI_SSID_2, WIFI_PASS_2)) {
			OLED_Print(0, 48, "Connected to AP 2!");
			wifi_is_connected = 1;
		} else {
			OLED_Print(0, 48, "APs Failed. Solo Mode");
			wifi_is_connected = 0;
		}
	}
	OLED_Update();
	HAL_Delay(500);

	/* Fetch Host IP and open telemetry socket channel */
	if (wifi_is_connected) {
		WIFI_GetIP(current_ip);
		OLED_Clear();
		OLED_Print(0, 0, "WI-FI BRIDGE");
		OLED_Print(0, 20, "IP obtained.");
		OLED_Print(0, 32, current_ip);
		OLED_Print(0, 48, "Binding UDP socket");
		OLED_Update();

		if (WIFI_StartUDP(PC_IP, UDP_PORT)) {
			OLED_Print(0, 56, "UDP Ready!");
		} else {
			OLED_Print(0, 56, "Socket Failure");
		}
		OLED_Update();
		HAL_Delay(1000);
	}

	OLED_Clear();
	OLED_Update();

    s_last_logic_ms = HAL_GetTick();
    s_last_lidar_ms = HAL_GetTick();
    s_last_oled_ms  = HAL_GetTick();
}

/* ═══════════════════════════════════════════════════════════════ */
/* GENERAL_Update                                                 */
/* ═══════════════════════════════════════════════════════════════ */
void GENERAL_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. Autonomous Loop (10 ms period = 100 Hz) ──────────── */
    if ((now - s_last_logic_ms) >= 10) {
        s_last_logic_ms = now;

        /* 1.1 Watch button press to increment across 4 distinct pages */
        BTNS_Update();
        if (BTNS_Get_OLEDPage() == BTN_PRESSED) {
            s_current_page = (s_current_page + 1) % 4;
        }

        /* 1.2 Fetch Navigation IMU Readings */
        MPU6050_RawData imu;
        HMC5883L_RawData mag;
        if (MPU6050_ReadRaw(&imu) == HAL_OK) {

        	s_last_imu_phys = MPU6050_GetPhysical(&imu);

            HMC5883L_ReadRaw(&mag);
            STABLE_Update(&imu, &mag, 0.01f);
            s_current_yaw = STABLE_GetOrientation().yaw;
        }

        if (s_first_run) {
			COVERAGE_Init(s_current_yaw); /* Captain locks onto the start direction! */
			s_first_run = 0;
		}

        /* 1.3 Check Sensor Array Blocks */
        int obs_F = (s_lidar_hits[0] >= 2);
        int obs_R = (s_lidar_hits[1] >= 2);
        int obs_B = (s_lidar_hits[2] >= 2);
        int obs_L = (s_lidar_hits[3] >= 2);

        int left_pwm = 0;
        int right_pwm = 0;

        /* 1.4 MASTER LOGIC (The Captain & The Driver) */
        RobotPose pose = ODOM_GetPose();
        float commanded_yaw = 0.0f;

        /* 1.5 Generate coverage tracking route profiles */
        s_current_cmd = COVERAGE_Update(pose, obs_F, obs_R, obs_L, &commanded_yaw);

        if (s_current_cmd == CMD_DRIVE_FORWARD) {
			if (obs_F) {
				/* SAFETY OVERRIDE: Human stepped in front! Slam brakes! */
				left_pwm = 0;
				right_pwm = 0;
			} else {
				/* ── TEMPORARY OPEN-LOOP CONTROL ── */
				/* Since the IMU is unlevel and we lack encoders, we use a static bias. */

				int base_speed = 300;

				/* TUNE THIS VALUE!
				   If the robot drifts LEFT  -> Right motor is too fast -> Make this negative (e.g., -20)
				   If the robot drifts RIGHT -> Left motor is too fast  -> Make this positive (e.g., +20) */
				int right_motor_offset = 0;

				left_pwm  = base_speed;
				right_pwm = base_speed + right_motor_offset;

				/* Clamp PWM limits just in case */
				if (left_pwm > 400) left_pwm = 400;
				if (right_pwm > 400) right_pwm = 400;

				/* Reset the IMU memory so it doesn't explode in the background */
				s_heading_integral = 0.0f;
			}
		}
        else if (s_current_cmd == CMD_TURN_RIGHT || s_current_cmd == CMD_TURN_LEFT) {
			/* PROPORTIONAL TURN CONTROLLER ── */
			float heading_error = angle_diff(commanded_yaw, s_current_yaw);

			/* Calculate speed based on how far we have left to turn.
			   Gain of 2.5 + Base speed of 180 to overcome friction. */
			int turn_speed = (int)(fabsf(heading_error) * 2.5f) + 180;

			/* Clamp the speeds so it doesn't spin too fast or stall */
			if (turn_speed > 350) turn_speed = 350;
			if (turn_speed < 180) turn_speed = 180;

			/* Apply the speed based on the Captain's direction */
			if (s_current_cmd == CMD_TURN_RIGHT) {
				left_pwm = turn_speed; right_pwm = -turn_speed;
			} else {
				left_pwm = -turn_speed; right_pwm = turn_speed;
			}
			s_target_yaw = commanded_yaw;
		}
        else {
            /* CMD_STOP */
            left_pwm = 0; right_pwm = 0;
        }

        uint8_t left_dir  = (left_pwm >= 0)  ? MOTOR_FORWARD : MOTOR_BACKWARD;
        uint8_t right_dir = (right_pwm >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

        MOTOR_Set(0, left_dir,  abs(left_pwm));
        MOTOR_Set(1, right_dir, abs(right_pwm));

        s_disp_l_pwm = left_pwm;
        s_disp_r_pwm = right_pwm;

        /* 1.6 Step Kinematics Physics Engine */
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

            /* Update the physics simulator */
			ODOM_UpdateEncoders(sim_v_left, sim_v_right, current_3d.yaw, 0.01f);

			pose = ODOM_GetPose(); /* Get pose (Odometry naturally starts at 0,0) */

			/* ── THE CENTER FIX ── */
			/* Add 7.5 meters so the physical (0,0) matches the center of the Map Array! */
			float map_center_x = pose.x + 7.5f;
			float map_center_y = pose.y + 7.5f;

			Map_UpdateRobotPose(&g_map, map_center_x, map_center_y, pose.theta);
        }
    }

    /* ── 2. LiDAR Scanning Round-Robin (35 ms Execution Interval) ── */
    if ((now - s_last_lidar_ms) >= 35) {
        s_last_lidar_ms = now;
        uint16_t dist = LIDAR_GetFilteredDistance(s_lidar_idx);
        s_lidar_raw[s_lidar_idx] = dist;

        if (dist > 40 && dist <= 200) {
            if (s_lidar_hits[s_lidar_idx] < 2) s_lidar_hits[s_lidar_idx]++;
        } else {
            s_lidar_hits[s_lidar_idx] = 0;
        }

        if (s_is_moving && dist > 40 && dist < 3500) {
            float angle = 0.0f;
            if(s_lidar_idx == 0) angle = 0.0f;   /* Front */
            if(s_lidar_idx == 1) angle = 90.0f;  /* Right */
            if(s_lidar_idx == 2) angle = 180.0f; /* Back  */
            if(s_lidar_idx == 3) angle = 270.0f; /* Left  */

            /* FIXED: Uses your updated Map_UpdateLiDAR function! */
            Map_UpdateLiDAR(&g_map, dist / 1000.0f, angle);
        }

        s_lidar_idx = (s_lidar_idx + 1) % 4;
    }

    /* ── 3. OLED Refresh & Wi-Fi Transmission Loop (200 ms Loop) ── */
		if ((now - s_last_oled_ms) >= 200) {
			s_last_oled_ms = now;

			/* Automatically transmit telemetry via JSON to PC Dashboard */
			send_telemetry_wifi();

			/* Screen state machine rendering routes */
			if (s_current_page == 0) {
				render_dashboard_page();
			} else if (s_current_page == 1) {
				render_compass_page();
			} else if (s_current_page == 2) {
				render_map_page();
			} else {
				render_wifi_page(); /* Page 3 handles network status info */
			}
		}
}
