#ifndef GENERAL_H
#define GENERAL_H

#include <mpu6050.h>
#include "main.h"
#include "hmc5883l.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "motor.h"
#include "btns.h"
#include "oled.h"
#include "odometry.h"
#include "mapping.h"
#include "ir.h"
#include "stable.h"
#include "ssd1306.h"
#include "fonts.h"

/* ── Feature switches ───────────────────────────────────────────*/
#define ENABLE_OLED_SELFTEST   0
#define ENABLE_OLED_DEBUG      1

/* ── OLED pages ─────────────────────────────────────────────────
   Page 0: Map viewport (32 blocks of 16×16 px)
   Page 1: Ultrasonic distances (4 sensors)
   Page 2: IMU data (accel, gyro, heading)
   Page 3: Position & speed                                       */
#define OLED_PAGE_MAP       0
#define OLED_PAGE_ULTRASONIC 1
#define OLED_PAGE_IMU       2
#define OLED_PAGE_POSITION  3
#define OLED_NUM_PAGES      4

/* ── OLED update rate ───────────────────────────────────────────
   SSD1306 I2C update takes ~23 ms at 400 kHz.
   Limit to 5 Hz to avoid stalling the main loop.               */
#define OLED_UPDATE_INTERVAL_MS   200U

/* ── Ultrasonic trigger interval ────────────────────────────────
   Round-robin one sensor per period = 4 × 25 ms = 100 ms / full cycle */
#define US_TRIGGER_INTERVAL_MS    25U

/* ── IMU update interval ────────────────────────────────────────*/
#define IMU_UPDATE_INTERVAL_MS    10U

/* Global map — accessible by slam_lite and other modules         */
extern Map g_map;

/* ─────────────────────────────────────────────────────────────── */
void GENERAL_Init(void);
void GENERAL_Update(void);

/* Long-press button resets position to map centre                */
void GENERAL_ResetPose(void);

#if ENABLE_OLED_SELFTEST
void GENERAL_OLED_SelfTest(void);
#endif

#if ENABLE_OLED_DEBUG
void GENERAL_OLED_Debug(void);
#endif

#endif /* GENERAL_H */
