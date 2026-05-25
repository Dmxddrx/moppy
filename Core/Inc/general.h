#ifndef GENERAL_H
#define GENERAL_H

#include "mpu6050.h"
#include "main.h"
#include "hmc5883l.h"
#include "encoder.h"
#include "motor.h"
#include "btns.h"
#include "oled.h"
#include "odometry.h"
#include "mapping.h"
#include "ir.h"
#include "stable.h"
#include "fonts.h"
#include "coverage.h"
#include "motion.h"
#include "wall_follow.h"
#include "lidar.h"
#include "wifi.h"

/* ── Feature switches ───────────────────────────────────────────*/
#define ENABLE_OLED_DEBUG      1

/* ── OLED pages ─────────────────────────────────────────────────*/
#define OLED_PAGE_DASHBOARD 0
#define OLED_PAGE_COMPASS   1
#define OLED_PAGE_MAP       2
#define OLED_PAGE_WIFI      3
#define OLED_NUM_PAGES      4 /* Set this to 3 to match your code */

/* ── Timing ─────────────────────────────────────────────────────
   OLED    : 5 Hz  (200 ms) — SSD1306 I2C update takes ~23 ms
   US      : one sensor per 25 ms = full 4-sensor cycle = 100 ms
   IMU     : 100 Hz (10 ms)                                      */
#define OLED_UPDATE_INTERVAL_MS   200U
#define US_TRIGGER_INTERVAL_MS     25U
#define IMU_UPDATE_INTERVAL_MS     10U






#define WIFI_SSID_1 "Dmx's Note20 Ultra"
#define WIFI_PASS_1 "11111129"

#define WIFI_SSID_2 "Dialog 4G 208"
#define WIFI_PASS_2 "Hasith2001"

#define PC_IP "192.168.8.198"  // Dialog 4G 208
//#define PC_IP "10.178.78.199"  // Dmx's Note20 Ultra
//#define PC_IP "192.168.8.155"
#define UDP_PORT 8080         // Must match the port in your C# App




/* Global map — accessible by slam_lite and other modules         */
extern Map g_map;

/* ─────────────────────────────────────────────────────────────── */
void GENERAL_Init(void);
void GENERAL_Update(void);

/* Long-press button resets position to map centre                */
void GENERAL_ResetPose(void);

void GENERAL_I2C_Scan(void);

#if ENABLE_OLED_SELFTEST
void GENERAL_OLED_SelfTest(void);
#endif

#if ENABLE_OLED_DEBUG
void GENERAL_OLED_Debug(void);
#endif

#endif /* GENERAL_H */
