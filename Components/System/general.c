//ultra fast
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
extern I2C_HandleTypeDef hi2c3;
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
char    current_ssid[32]  = "Not Connected";
char    current_pc_ip[16] = "0.0.0.0";

/* ── Module-private state ───────────────────────────────────────*/
static uint32_t s_last_oled_ms = 0;
static uint32_t s_last_logic_ms  = 0;
static uint8_t  s_current_page  = 0;

/* Store RAW uint16_t values (Millimeters) */
static uint32_t s_last_lidar_ms = 0;
static uint32_t s_last_async_lidar_ms = 0;
 uint16_t s_lidar_raw[4]  = {65535, 65535, 65535, 65535};
 uint8_t  s_lidar_hits[4] = {0, 0, 0, 0};

/* IMU Navigation State */
 float s_current_yaw = 0.0f;
 int   s_is_moving   = 0; /* NEW: Tracks if the motors/LEDs are active */
 CoverageCmd s_current_cmd = CMD_STOP; /* The Captain's Orders! */
static int s_first_run = 1;

/* Store the latest physical IMU data for Wi-Fi broadcast */
 MPU6050_PhysData s_last_imu_phys = {0};
 HMC5883L_RawData s_last_mag = {0};

/* Variables to share motor speeds with the OLED display */
 int s_disp_l_pwm = 0;
 int s_disp_r_pwm = 0;

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
    ENCODER_Init();

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

	/* 1. Map your definitions and corresponding PC IPs into arrays */
	const char* ssids[4]      = {WIFI_SSID_1, WIFI_SSID_2, WIFI_SSID_3, WIFI_SSID_4};
	const char* passes[4]     = {WIFI_PASS_1, WIFI_PASS_2, WIFI_PASS_3, WIFI_PASS_4};
	const char* target_ips[4] = {PC_IP_1, PC_IP_2, PC_IP_3, PC_IP_4};

	const char* active_pc_ip = NULL; /* Will hold the correct IP once connected */
	wifi_is_connected = 0;

	/* 2. Loop through the list dynamically */
	for (int i = 0; i < 4; i++) {
		char buf[32];

		snprintf(buf, sizeof(buf), "Trying AP %d...     ", i + 1);
		OLED_Print(0, 48, buf);
		OLED_Update();

		if (WIFI_Connect(ssids[i], passes[i])) {
			snprintf(buf, sizeof(buf), "Connected AP %d!    ", i + 1);
			OLED_Print(0, 48, buf);

			wifi_is_connected = 1;
			active_pc_ip = target_ips[i]; /* Save the matching PC IP! */

			strncpy(current_ssid, ssids[i], sizeof(current_ssid) - 1);
			strncpy(current_pc_ip, target_ips[i], sizeof(current_pc_ip) - 1);

			break;
		} else {
			WIFI_Disconnect();
			HAL_Delay(500);
		}
	}

	if (!wifi_is_connected) {
		OLED_Print(0, 48, "APs Failed. Solo Mode");
	}

	OLED_Update();
	HAL_Delay(500);

	/* Fetch Host IP and open telemetry socket channel */
	if (wifi_is_connected && active_pc_ip != NULL) {
		WIFI_GetIP(current_ip);
		OLED_Clear();
		OLED_Print(0, 0, "WI-FI BRIDGE");
		OLED_Print(0, 20, "IP obtained.");
		OLED_Print(0, 30, current_ip);
		OLED_Print(0, 46, "Binding UDP socket");
		OLED_Update();

		/* 3. Use the dynamically selected PC IP here */
		if (WIFI_StartUDP(active_pc_ip, UDP_PORT)) {
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

void GENERAL_100Hz_ControlLoop(void) {
    s_imu_math_ready = 0;
    static CoverageCmd s_prev_cmd = CMD_STOP;

    /* 1. Parse DMA Buffers */
	MPU6050_RawData imu;
	imu.ax = (int16_t)((s_mpu_dma_buf[0] << 8) | s_mpu_dma_buf[1]);
	imu.ay = (int16_t)((s_mpu_dma_buf[2] << 8) | s_mpu_dma_buf[3]);
	imu.az = (int16_t)((s_mpu_dma_buf[4] << 8) | s_mpu_dma_buf[5]);
	imu.gx = (int16_t)((s_mpu_dma_buf[8] << 8) | s_mpu_dma_buf[9]);
	imu.gy = (int16_t)((s_mpu_dma_buf[10] << 8) | s_mpu_dma_buf[11]);
	imu.gz = (int16_t)((s_mpu_dma_buf[12] << 8) | s_mpu_dma_buf[13]);

	s_last_mag.mx = (int16_t)((s_hmc_dma_buf[0] << 8) | s_hmc_dma_buf[1]);
	s_last_mag.mz = (int16_t)((s_hmc_dma_buf[2] << 8) | s_hmc_dma_buf[3]);
	s_last_mag.my = (int16_t)((s_hmc_dma_buf[4] << 8) | s_hmc_dma_buf[5]);


    /* 2. Run IMU Math */
    s_last_imu_phys = MPU6050_GetPhysical(&imu);
    STABLE_Update(&imu, &s_last_mag, 0.01f);
    s_current_yaw = STABLE_GetOrientation().yaw;

    if (s_first_run) {
            COVERAGE_Init(s_current_yaw);
            s_first_run = 0;
        }

	/* 3. Evaluate Sensor Logic */
    /* A hit is only valid if it was seen at least 2 times in a row */
	int obs_F = (s_lidar_hits[0] >= 2);
	int obs_R = (s_lidar_hits[1] >= 2);
	int obs_B = (s_lidar_hits[2] >= 2);
	int obs_L = (s_lidar_hits[3] >= 2);

	/* 3.1 MPU6050 Bump & Stuck Detection */
	int bump_detected = 0;
	int is_stuck = 0;
	static uint16_t stuck_timer = 0;
	static uint16_t blind_timer = 0;

	if (s_is_moving) {
		/* NEW: Ignore all G-force spikes for the first 500ms of acceleration */
		if (blind_timer < 50) {
			blind_timer++;
		} else {
			/* BUMP DETECTION: Sudden impact spike over ~0.3G (3.0 m/s^2) */
			if (fabsf(s_last_imu_phys.ax) > 3.0f || fabsf(s_last_imu_phys.ay) > 16.0f) {
				bump_detected = 1;
			}
		}

		/* STUCK DETECTION: Wheels have power, but IMU detects zero vibration/movement */
		if (fabsf(s_last_imu_phys.ax) < 0.15f && fabsf(s_last_imu_phys.ay) < 0.15f) {
			stuck_timer++;
			/* 200 ticks at 100Hz = 2.0 Seconds of being beached */
			if (stuck_timer > 200) {
				is_stuck = 1;
			}
		} else {
			stuck_timer = 0; /* Reset timer if normal driving vibration returns */
		}
	} else {
		stuck_timer = 0;
		blind_timer = 0; /* Reset blindfold whenever the robot completely stops */
	}

	int left_pwm = 0, right_pwm = 0;
	float commanded_yaw = 0.0f;

	/* 4. Execute Route Plan */
	s_current_cmd = COVERAGE_Update(s_current_yaw, obs_F, obs_R, obs_L, obs_B, bump_detected, is_stuck, &commanded_yaw);

	if (s_current_cmd != s_prev_cmd) {
		blind_timer = 0;
	}

	if ((s_prev_cmd == CMD_TURN_LEFT || s_prev_cmd == CMD_TURN_RIGHT) && s_current_cmd == CMD_DRIVE_FORWARD) {
		memset((void*)s_lidar_hits, 0, sizeof(s_lidar_hits));
	}
	s_prev_cmd = s_current_cmd;

	if (s_current_cmd == CMD_DRIVE_FORWARD) {
		left_pwm = 300; right_pwm = 300;
	}
	else if (s_current_cmd == CMD_REVERSE) {
		left_pwm = -250; right_pwm = -250;
	}
	else if (s_current_cmd == CMD_TURN_RIGHT || s_current_cmd == CMD_TURN_LEFT) {
		float heading_error = angle_diff(commanded_yaw, s_current_yaw);

		/* Hard-stop deadband */
		if (fabsf(heading_error) <= 3.0f) {
			left_pwm = 0;
			right_pwm = 0;
		} else {
			int turn_speed = (int)(fabsf(heading_error) * 2.0f) + 140;
			if (turn_speed > 280) turn_speed = 280;

			if (s_current_cmd == CMD_TURN_RIGHT) {
				left_pwm = turn_speed; right_pwm = -turn_speed;
			} else {
				left_pwm = -turn_speed; right_pwm = turn_speed;
			}
		}
	}

	if (left_pwm > 400) left_pwm = 400; else if (left_pwm < -400) left_pwm = -400;
	if (right_pwm > 400) right_pwm = 400; else if (right_pwm < -400) right_pwm = -400;

	uint8_t left_dir  = (left_pwm >= 0)  ? MOTOR_FORWARD : MOTOR_BACKWARD;
	uint8_t right_dir = (right_pwm >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

	MOTOR_Set(0, left_dir,  abs(left_pwm));
	MOTOR_Set(1, right_dir, abs(right_pwm));

	s_disp_l_pwm = left_pwm;
	s_disp_r_pwm = right_pwm;

	/* 5. Update Real-World Kinematics via Hardware Encoders */
	s_is_moving = (left_pwm != 0 || right_pwm != 0);

	if (s_is_moving) {
		float radius = 0.0325f;
		float real_v_left  = ENCODER_GetSpeed(0) * radius;
		float real_v_right = ENCODER_GetSpeed(1) * radius;

		if (MOTOR_GetDirection(0) == -1) real_v_left *= -1.0f;
		if (MOTOR_GetDirection(1) == -1) real_v_right *= -1.0f;

		Orientation current_3d = STABLE_GetOrientation();
		ODOM_UpdateEncoders(real_v_left, real_v_right, current_3d.yaw, 0.01f);

		RobotPose pose = ODOM_GetPose();
		Map_UpdateRobotPose(&g_map, pose.x + 7.5f, pose.y + 7.5f, pose.theta);
	}
}

/* ═══════════════════════════════════════════════════════════════ */
/* GENERAL_Update (The Low-Priority Background Task)          	   */
/* ═══════════════════════════════════════════════════════════════ */
void GENERAL_Update(void)
{
	uint32_t now = HAL_GetTick();

	/* ── 0.0 I2C1 DMA WATCHDOG ── */
	if (s_i2c1_bus_state >= 1 && s_i2c1_bus_state <= 3) {
		if ((now - s_i2c1_watchdog) > 20) {
			s_i2c1_bus_state = 0; /* FREE THE BUS! */
		}
	}

	/* ── 0.1 Catch Callback Signal for Compass ── */

	/* ── 1. Low Priority Hardware Triggers (100 Hz) ──────────── */
	if ((now - s_last_logic_ms) >= 10) {
		s_last_logic_ms = now;

		BTNS_Update();
		if (BTNS_Get_OLEDPage() == BTN_PRESSED) {
			s_current_page = (s_current_page + 1) % 5;
		}

		/* Trigger next DMA sequence */
		if (s_i2c1_bus_state == 0) {
			s_i2c1_bus_state = 1;
			s_i2c1_watchdog = now;
			if (HAL_I2C_Mem_Read_DMA(&hi2c1, 0x68 << 1, 0x3B, 1, s_mpu_dma_buf, 14) != HAL_OK) {
				s_i2c1_bus_state = 0;
			}
		}
	}

    /* ── 2. LiDAR Scanning Round-Robin (35 ms Execution Interval) ── */
	if ((now - s_last_async_lidar_ms) >= 4) {
		s_last_async_lidar_ms = now;
		Process_LiDAR_Asynchronous();
	}

    /* ── 3. OLED Refresh & Wi-Fi Transmission Loop (200 ms Loop) ── */
	if ((now - s_last_oled_ms) >= 200) {
		s_last_oled_ms = now;


#if WIFI_BRIDGE
        send_telemetry_wifi();
#endif
        if (s_current_page == 0)      render_dashboard_page();
        else if (s_current_page == 1) render_compass_page();
        else if (s_current_page == 2) render_map_page();
#if WIFI_BRIDGE
        else if (s_current_page == 3) render_wifi_page();
#endif
        else if (s_current_page == 4) render_calib_page();

	}
}
