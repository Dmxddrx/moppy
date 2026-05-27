//ultra fast
#include "general.h"
#include <stdio.h>   /* Standard I/O: Gives you snprintf() to format text and numbers for your OLED screen */
#include <string.h>  /* String & Memory: Gives you memset() to instantly clear your 10,000-cell map array to 0 */
#include <math.h>    /* Math: Gives you sinf(), cosf(), and atan2f() for your compass and LiDAR trigonometry */
#include <stdlib.h>  /* Standard Library: Gives you abs() to calculate absolute values (like in your Bresenham ray-casting) */

#define MOTOR_INIT 1
#define WIFI_BRIDGE 0

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Global Map Instance ───────────────────────────────────────*/
Map g_map;

/* ── Hardware Handles (Imported from main.c) ──────────────────── */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

/* ── I2C1 DMA Traffic Controller ──────────────────────────────── */
/* 0 = Idle, 1 = MPU Reading, 2 = HMC Reading, 3 = LiDAR Blocking */
volatile uint8_t s_i2c1_bus_state = 0;
uint32_t s_i2c1_watchdog = 0; /* NEW: Failsafe timer for stuck bus */

/* DMA Data Buffers */
static uint8_t s_mpu_dma_buf[14] = {0};
static uint8_t s_hmc_dma_buf[6]  = {0};

/* Flag to tell the main loop new math is ready */
volatile uint8_t s_imu_math_ready = 0;

/* ── Global Network Tracking Variables ───────────────────────── */
uint8_t wifi_is_connected = 0;
uint8_t esp_is_ready       = 0;
char    current_ip[16]    = "0.0.0.0";

/* ── Module-private state ───────────────────────────────────────*/
static uint32_t s_last_oled_ms = 0;
static uint32_t s_last_logic_ms  = 0;
static uint8_t  s_current_page  = 0;

/* Store RAW uint16_t values (Millimeters) */
static uint32_t s_last_lidar_ms = 0;
static uint8_t  s_lidar_idx     = 0;
static uint16_t s_lidar_raw[4]  = {65535, 65535, 65535, 65535};
static uint8_t  s_lidar_hits[4] = {0, 0, 0, 0};

/* IMU Navigation State */
static float s_current_yaw = 0.0f;
static int   s_is_moving   = 0; /* NEW: Tracks if the motors/LEDs are active */
static CoverageCmd s_current_cmd = CMD_STOP; /* The Captain's Orders! */
static int s_first_run = 1;

/* Store the latest physical IMU data for Wi-Fi broadcast */
static MPU6050_PhysData s_last_imu_phys = {0};
static HMC5883L_RawData s_last_mag = {0};

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
/* I2C1 DMA CHAIN (The Background Traffic Cop)                     */
/* ═══════════════════════════════════════════════════════════════ */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {

        /* 1. If MPU6050 just finished reading... */
        if (s_i2c1_bus_state == 1) {
            s_i2c1_bus_state = 2; /* Switch traffic light to Compass */

            /* Immediately trigger the Compass DMA read */
            HAL_I2C_Mem_Read_DMA(&hi2c1, 0x1E << 1, 0x03, 1, s_hmc_dma_buf, 6);
        }

        /* 2. If HMC5883L just finished reading... */
        else if (s_i2c1_bus_state == 2) {
            s_i2c1_bus_state = 0; /* FREE THE BUS! */
            s_imu_math_ready = 1; /* Tell the 100Hz loop to calculate the heading */
        }
    }
}
#if WIFI_BRIDGE
/* ═══════════════════════════════════════════════════════════════ */
/* TELEMETRY BROADCAST ENGINE                                      */
/* ═══════════════════════════════════════════════════════════════ */
static void send_telemetry_wifi(void) {
    if (!wifi_is_connected) return;

    char tx_buf[512];
    RobotPose pose = ODOM_GetPose();
    Orientation current_3d = STABLE_GetOrientation();

    snprintf(tx_buf, sizeof(tx_buf),
		"{\"F\":%u, \"R\":%u, \"B\":%u, \"L\":%u, \"yaw\":%.1f, \"X\":%.1f, \"Y\":%.1f, "
		"\"pwmL\":%d, \"pwmR\":%d, \"cmd\":%d, \"ax\":%.2f, \"ay\":%.2f, \"az\":%.2f, "
		"\"gx\":%.2f, \"gy\":%.2f, \"gz\":%.2f, \"mx\":%d, \"my\":%d, \"mz\":%d, "
		"\"head\":%.1f, \"roll\":%.1f, \"pitch\":%.1f}\n",
		s_lidar_raw[0], s_lidar_raw[1], s_lidar_raw[2], s_lidar_raw[3],
		s_current_yaw, pose.x, pose.y, s_disp_l_pwm, s_disp_r_pwm, s_current_cmd,
		s_last_imu_phys.ax, s_last_imu_phys.ay, s_last_imu_phys.az,
		s_last_imu_phys.gx, s_last_imu_phys.gy, s_last_imu_phys.gz,
		s_last_mag.mx, s_last_mag.my, s_last_mag.mz,
		s_current_yaw, current_3d.roll, current_3d.pitch);

	WIFI_SendUDPData(tx_buf);
}
#endif

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
	if (s_current_cmd == CMD_DRIVE_FORWARD)     state_str = "FORWARD";
	else if (s_current_cmd == CMD_REVERSE)      state_str = "REVERSE";
	else if (s_current_cmd == CMD_TURN_LEFT)    state_str = "TURN L";
	else if (s_current_cmd == CMD_TURN_RIGHT)   state_str = "TURN R";
	else if (s_current_cmd == CMD_STOP)         state_str = "STOPPED";
	OLED_Print(68, 55, state_str);

    OLED_Update();
}
#if MOTOR_INIT
/* ═══════════════════════════════════════════════════════════════ */
/* LIVE COMPASS DELAY HELPER (Runs motors while updating screen)   */
/* ═══════════════════════════════════════════════════════════════ */
static void live_compass_delay(uint32_t delay_ms) {
		uint32_t start_time = HAL_GetTick();

		while ((HAL_GetTick() - start_time) < delay_ms) {
			MPU6050_RawData imu;
			HMC5883L_RawData mag;

			/* 1. Read the sensors */
			if (MPU6050_ReadRaw(&imu) == HAL_OK) {
				HMC5883L_ReadRaw(&mag);
				STABLE_Update(&imu, &mag, 0.05f);
				s_current_yaw = STABLE_GetOrientation().yaw;
			}

			/* 2. Redraw the compass screen with the new yaw */
			render_compass_page();
			HAL_Delay(50);
		}
}

/* ═══════════════════════════════════════════════════════════════ */
/* CLOSED-LOOP TURN TEST HELPER (Proportional Navigation)          */
/* ═══════════════════════════════════════════════════════════════ */
static void test_turn_by_angle(float angle_change, int is_left) {
		/* Let the compass stabilize for a moment before picking our target */
		for(int i=0; i<5; i++) { live_compass_delay(50); }

		float start_yaw = s_current_yaw;
		float target_yaw;

		if (is_left) {
			s_current_cmd = CMD_TURN_LEFT;
			target_yaw = fmodf(start_yaw + angle_change, 360.0f);
		} else {
			s_current_cmd = CMD_TURN_RIGHT;
			target_yaw = fmodf(start_yaw + 360.0f - angle_change, 360.0f);
		}

		/* Loop until we reach the target angle! */
		while (1) {
			MPU6050_RawData imu;
			HMC5883L_RawData mag;
			if (MPU6050_ReadRaw(&imu) == HAL_OK) {
				HMC5883L_ReadRaw(&mag);
				STABLE_Update(&imu, &mag, 0.05f);
				s_current_yaw = STABLE_GetOrientation().yaw;
			}

			/* Proportional Control Math */
			float heading_error = angle_diff(target_yaw, s_current_yaw);

			/* Stop if we are within 3 degrees of the perfect angle */
			if (fabsf(heading_error) <= 3.0f) {
				MOTOR_Set(0, MOTOR_FORWARD, 0);
				MOTOR_Set(1, MOTOR_FORWARD, 0);
				s_current_cmd = CMD_STOP;
				render_compass_page();
				break;
			}

			/* Proportional Slowdown (Matches your main loop!) */
			int turn_speed = (int)(fabsf(heading_error) * 2.0f) + 160;
			if (turn_speed > 300) turn_speed = 300;

			if (is_left) {
				MOTOR_Set(0, MOTOR_BACKWARD, turn_speed);
				MOTOR_Set(1, MOTOR_FORWARD,  turn_speed);
			} else {
				MOTOR_Set(0, MOTOR_FORWARD,  turn_speed);
				MOTOR_Set(1, MOTOR_BACKWARD, turn_speed);
			}

			render_compass_page();
			HAL_Delay(50);
		}
}
#endif

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
	if (s_current_cmd == CMD_DRIVE_FORWARD)     state_str = "FORWARD";
	else if (s_current_cmd == CMD_REVERSE)      state_str = "REVERSE";
	else if (s_current_cmd == CMD_TURN_LEFT)    state_str = "TURN L";
	else if (s_current_cmd == CMD_TURN_RIGHT)   state_str = "TURN R";
	else if (s_current_cmd == CMD_STOP)         state_str = "STOPPED";
	OLED_Print(68, 55, state_str);

	snprintf(buf, sizeof(buf), "%3.0f deg", s_current_yaw);
	OLED_Print(68, 45, buf);

    OLED_Update();
}

#if WIFI_BRIDGE
/* ═══════════════════════════════════════════════════════════════ */
/* WI-FI / NETWORK PAGE (3)                                        */
/* ═══════════════════════════════════════════════════════════════ */
static void render_wifi_page(void) {
    OLED_Clear();
    OLED_Print(0, 0, "MOPPY NETWORK");

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
#endif

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

    MOTOR_Init();
    MOTORPWM_Init();
    MOTOR_WakeAll();

#if MOTOR_INIT
		/* ═══════════════════════════════════════════════════════════════ */
		/* MOTOR & COMPASS LIVE TEST (PROPORTIONAL)                        */
		/* ═══════════════════════════════════════════════════════════════ */
		OLED_Clear();
		OLED_Print(0, 0, "MOTOR TEST!");
		OLED_Update();
		HAL_Delay(1500);

		int test_speed = 300;

		// 1. FORWARD (2 Seconds)
		s_current_cmd = CMD_DRIVE_FORWARD;
		MOTOR_Set(0, MOTOR_FORWARD, test_speed);
		MOTOR_Set(1, MOTOR_FORWARD, test_speed);
		live_compass_delay(2000);

		s_current_cmd = CMD_STOP;
		MOTOR_Set(0, MOTOR_FORWARD, 0);
		MOTOR_Set(1, MOTOR_FORWARD, 0);
		live_compass_delay(1000);

		// 2. REVERSE (2 Seconds)
		s_current_cmd = CMD_REVERSE;
		MOTOR_Set(0, MOTOR_BACKWARD, test_speed);
		MOTOR_Set(1, MOTOR_BACKWARD, test_speed);
		live_compass_delay(2000);

		// Stop to stabilize before starting the maze simulation
		s_current_cmd = CMD_STOP;
		MOTOR_Set(0, MOTOR_FORWARD, 0);
		MOTOR_Set(1, MOTOR_FORWARD, 0);
		live_compass_delay(1000);

		// ─── 90 DEGREE PRECISION TESTS ───
		test_turn_by_angle(90.0f, 1);   // Turn Left 90
		live_compass_delay(1000);

		test_turn_by_angle(90.0f, 0);   // Turn Right 90 (Back to start)
		live_compass_delay(1000);

		test_turn_by_angle(90.0f, 0);   // Turn Right 90
		live_compass_delay(1000);

		test_turn_by_angle(90.0f, 1);   // Turn Left 90 (Back to start)
		live_compass_delay(1000);

		// ─── 180 DEGREE PRECISION TESTS ───
		test_turn_by_angle(180.0f, 1);  // Turn Left 180
		live_compass_delay(1000);

		test_turn_by_angle(180.0f, 0);  // Turn Right 180 (Back to start)
		live_compass_delay(1000);

		test_turn_by_angle(180.0f, 0);  // Turn Right 180
		live_compass_delay(1000);

		test_turn_by_angle(180.0f, 1);  // Turn Left 180 (Back to start)
		live_compass_delay(1000);

		// 5. TEST COMPLETE
		s_current_cmd = CMD_STOP;
		MOTOR_Set(0, MOTOR_FORWARD, 0);
		MOTOR_Set(1, MOTOR_FORWARD, 0);
		HAL_Delay(1000);
		/* ═══════════════════════════════════════════════════════════════ */
#endif

	LIDAR_Init();

    HAL_Delay(500);

#if WIFI_BRIDGE
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
#endif

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

		/* ── 0.0 I2C1 DMA WATCHDOG (The Failsafe) ── */
		/* If the DMA gets stuck for more than 20ms, force an unlock!
		   Note: We ignore state 4 because LiDAR is blocking and has its own timeouts */
		if (s_i2c1_bus_state >= 1 && s_i2c1_bus_state <= 3) {
			if ((now - s_i2c1_watchdog) > 20) {
				s_i2c1_bus_state = 0; /* FREE THE BUS! */
			}
		}

		/* ── 0.1 Catch Callback Signal for Compass ── */
		if (s_i2c1_bus_state == 2) {
			s_i2c1_bus_state = 3;
			s_i2c1_watchdog = now; /* Reset watchdog for Compass */

			/* If HAL_BUSY is returned, abort and free the bus */
			if (HAL_I2C_Mem_Read_DMA(&hi2c1, 0x3C, 0x03, 1, s_hmc_dma_buf, 6) != HAL_OK) {
				s_i2c1_bus_state = 0;
			}
		}

		/* 1. Autonomous Loop (10 ms period = 100 Hz) ──────────── */
		if ((now - s_last_logic_ms) >= 10) {
			s_last_logic_ms = now;

			BTNS_Update();
			if (BTNS_Get_OLEDPage() == BTN_PRESSED) {
				s_current_page = (s_current_page + 1) % 4;
			}

			/* 1.2 Fetch Navigation IMU Readings */
			if (s_i2c1_bus_state == 0) {
				s_i2c1_bus_state = 1;
				s_i2c1_watchdog = now; /* Reset watchdog for MPU */

				/* If HAL_BUSY is returned, abort and free the bus */
				if (HAL_I2C_Mem_Read_DMA(&hi2c1, 0x68 << 1, 0x3B, 1, s_mpu_dma_buf, 14) != HAL_OK) {
					s_i2c1_bus_state = 0;
				}
		}

		/* 1.2.5 CALCULATE MATH (Only runs after callback finishes!) */
		if (s_imu_math_ready == 1) {
			s_imu_math_ready = 0; // Reset flag

			// A. Manually parse the DMA buffers into your existing structs
			MPU6050_RawData imu;
			imu.ax = (int16_t)((s_mpu_dma_buf[0] << 8) | s_mpu_dma_buf[1]);
			imu.ay = (int16_t)((s_mpu_dma_buf[2] << 8) | s_mpu_dma_buf[3]);
			imu.az = (int16_t)((s_mpu_dma_buf[4] << 8) | s_mpu_dma_buf[5]);
			imu.gx = (int16_t)((s_mpu_dma_buf[8] << 8) | s_mpu_dma_buf[9]);
			imu.gy = (int16_t)((s_mpu_dma_buf[10] << 8) | s_mpu_dma_buf[11]);
			imu.gz = (int16_t)((s_mpu_dma_buf[12] << 8) | s_mpu_dma_buf[13]);

			// FIX: Save directly to the GLOBAL s_last_mag so Wi-Fi can see it!
			s_last_mag.mx = (int16_t)((s_hmc_dma_buf[0] << 8) | s_hmc_dma_buf[1]);
			s_last_mag.mz = (int16_t)((s_hmc_dma_buf[2] << 8) | s_hmc_dma_buf[3]);
			s_last_mag.my = (int16_t)((s_hmc_dma_buf[4] << 8) | s_hmc_dma_buf[5]);

			// B. Run your math!
			s_last_imu_phys = MPU6050_GetPhysical(&imu);
			STABLE_Update(&imu, &s_last_mag, 0.01f);
			s_current_yaw = STABLE_GetOrientation().yaw;
		}

        if (s_first_run) {
			COVERAGE_Init(s_current_yaw); /* Captain locks onto the start direction! */
			s_first_run = 0;
		}

        /* 1.3 Check Sensor Array Blocks */
        int obs_F = (s_lidar_raw[0] > 20 && s_lidar_raw[0] < 150);
        int obs_B = (s_lidar_raw[2] > 20 && s_lidar_raw[2] < 150);
		int obs_R = (s_lidar_raw[1] > 20 && s_lidar_raw[1] < 150);
		int obs_L = (s_lidar_raw[3] > 20 && s_lidar_raw[3] < 150);

		int left_pwm = 0;
		int right_pwm = 0;
		float commanded_yaw = 0.0f;

		/* 1.4 Get Command from Brain (coverage.c) */
		s_current_cmd = COVERAGE_Update(s_current_yaw, obs_F, obs_R, obs_L, obs_B, &commanded_yaw);

        /* 1.5 Generate coverage tracking route profiles */
		if (s_current_cmd == CMD_DRIVE_FORWARD) {
			left_pwm = 300;
			right_pwm = 300;
		}
		else if (s_current_cmd == CMD_REVERSE) {
			left_pwm = -250;
			right_pwm = -250;
		}
		else if (s_current_cmd == CMD_TURN_RIGHT || s_current_cmd == CMD_TURN_LEFT) {
			/* THE PROPORTIONAL SLOWDOWN SOLUTION */
			float heading_error = angle_diff(commanded_yaw, s_current_yaw);

			/* Multiply remaining degrees by 2.0. Add 160 to overcome wheel friction. */
			int turn_speed = (int)(fabsf(heading_error) * 2.0f) + 160;

			/* Cap speed to 300 so it doesn't spin out of control */
			if (turn_speed > 300) turn_speed = 300;

			if (s_current_cmd == CMD_TURN_RIGHT) {
				left_pwm = turn_speed; right_pwm = -turn_speed;
			} else {
				left_pwm = -turn_speed; right_pwm = turn_speed;
			}
		}
		else {
			left_pwm = 0; right_pwm = 0;
		}

		/* Clamp absolute limits */
		if (left_pwm > 400){
			left_pwm = 400; if (left_pwm < -400) left_pwm = -400;
		}
		if (right_pwm > 400){
			right_pwm = 400; if (right_pwm < -400) right_pwm = -400;
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

					ODOM_UpdateEncoders(sim_v_left, sim_v_right, current_3d.yaw, 0.01f);
					RobotPose pose = ODOM_GetPose();

					/* ── THE CENTER FIX ── */
					/* Add 7.5 meters so the physical (0,0) matches the center of the Map Array! */
					float map_center_x = pose.x + 7.5f;
					float map_center_y = pose.y + 7.5f;

					Map_UpdateRobotPose(&g_map, map_center_x, map_center_y, pose.theta);
		}
	}

    /* ── 2. LiDAR Scanning Round-Robin (35 ms Execution Interval) ── */
        if ((now - s_last_lidar_ms) >= 35) {

            /* 1. TRAFFIC COP: Only proceed if the IMU/Compass DMA is NOT currently using the I2C1 bus! */
            if (s_i2c1_bus_state == 0) {

                /* 2. LOCK THE BUS: Tell the IMU to wait, the LiDAR is talking! */
                s_i2c1_bus_state = 4;

                /* Reset the timer so it waits another 35ms for the next scan */
                s_last_lidar_ms = now;

                /* 3. Run the standard Blocking VL53L0X Library */
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

                    /* Map the obstacle */
                    Map_UpdateLiDAR(&g_map, dist / 1000.0f, angle);
                }

                /* Move to the next sensor for the next round */
                s_lidar_idx = (s_lidar_idx + 1) % 4;

                /* 4. UNLOCK THE BUS: The LiDAR is done, the IMU DMA is free to run again! */
                s_i2c1_bus_state = 0;
            }
            /* Note: If the bus was locked (state 1 or 2), the loop just skips this millisecond
               and will try reading the LiDAR again on the very next cycle! */
        }

    /* ── 3. OLED Refresh & Wi-Fi Transmission Loop (200 ms Loop) ── */
		if ((now - s_last_oled_ms) >= 200) {
			s_last_oled_ms = now;

			#if WIFI_BRIDGE
			/* Automatically transmit telemetry via JSON to PC Dashboard */
			send_telemetry_wifi();
			#endif
			/* Screen state machine rendering routes */
			if (s_current_page == 0) {
				render_dashboard_page();
			} else if (s_current_page == 1) {
				render_compass_page();
			} else if (s_current_page == 2) {
				render_map_page();
			} else {
				#if WIFI_BRIDGE
				render_wifi_page(); /* Page 3 handles network status info */
				#endif
			}
		}
}
