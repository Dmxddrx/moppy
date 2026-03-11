#include "general.h"
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
   GENERAL.C — Top-level robot loop combinator

   Pipeline:
     SENSORS
       → STABLE   (IMU fusion — roll/pitch/yaw)
       → ODOM     (wheel odometry — x, y, theta)
       → SLAM     (yaw correction — fuse IMU into odom theta)
       → COVERAGE (planner — target heading + speed)
       → MOTION   (heading PID — per-wheel speed)
       → MOTOR    (PWM output)
   ================================================================ */

extern I2C_HandleTypeDef hi2c1;   /* MPU6500 + HMC5883L + ADS1115 */
extern I2C_HandleTypeDef hi2c2;   /* OLED                         */

/* ----------------------------------------------------------------
   TICKS_TO_METERS
   HC-020K disc: 20 slots per revolution
   Encoder mode TI1+TI2 on a single-channel sensor = 20 counts/rev
   Wheel diameter: measure yours and substitute.
   Example: wheel ⌀ 65mm → circumference = π × 0.065 = 0.2042m
            0.2042 / 20 = 0.01021 m/tick
   !! Measure your actual wheel and update this value !!
---------------------------------------------------------------- */
#define TICKS_TO_METERS   0.01021f

/* Motor output clamp — must match ARR = 999 */
#define MOTOR_OUT_MAX     999

/* HC-SR04 obstacle threshold (cm) */
#define OBSTACLE_CM       20.0f

/* ----------------------------------------------------------------
   Sensor snapshot — updated every loop
---------------------------------------------------------------- */
typedef struct {
    int32_t enc[2];
    uint8_t ir[4];
    float   us_cm[4];
} RobotState;

static RobotState robot;

static MPU6500_RawData  imu_raw;
static HMC5883L_RawData mag_raw;

/* Globals exposed for OLED debug */
float heading          = 0.0f;
float robot_x          = 0.0f;
float robot_y          = 0.0f;
int   motion_state     = 0;
int   coverage_percent = 0;
int   map_cells        = 0;

/* Track commanded direction per wheel for encoder sign correction */
static uint8_t motor_dir[2] = {MOTOR_FORWARD, MOTOR_FORWARD};


/* ================================================================
   GENERAL_Init
   ================================================================ */
void GENERAL_Init(void)
{
    MOTOR_Init();
    MOTION_Init();
    ENCODER_Init();
    IR_Init();
    ULTRASONIC_Init();
    MPU6500_Init(&hi2c1);
    HMC5883L_Init(&hi2c1);
    STABLE_Init();
    ODOM_Init();
    SLAM_Init();
    COVERAGE_Init();

    OLED_Init(&hi2c2);
    OLED_Clear();
}


/* ================================================================
   GENERAL_OLED_Debug
   ================================================================ */
void GENERAL_OLED_Debug(void)
{
    char line[32];

    OLED_Clear();

    // Static 1px line across top — always visible, proves display is alive
    OLED_Rectangle(0, 0, 128, 5);

    snprintf(line, sizeof(line), "HD:%3d M:%d", (int)heading, motion_state);
    OLED_Print(0, 2, line);   // shifted down 2px to clear the line

    snprintf(line, sizeof(line), "IR:%d %d %d %d",
             robot.ir[0], robot.ir[1], robot.ir[2], robot.ir[3]);
    OLED_Print(0, 12, line);

    snprintf(line, sizeof(line), "US:%2d %2d %2d %2d",
             (int)robot.us_cm[0], (int)robot.us_cm[1],
             (int)robot.us_cm[2], (int)robot.us_cm[3]);
    OLED_Print(0, 22, line);

    snprintf(line, sizeof(line), "ENC:%ld %ld",
             robot.enc[0], robot.enc[1]);
    OLED_Print(0, 32, line);

    snprintf(line, sizeof(line), "X:%ld.%02ld Y:%ld.%02ld",
             (int32_t)robot_x,
             (int32_t)(fabsf(robot_x - (int32_t)robot_x) * 100),
             (int32_t)robot_y,
             (int32_t)(fabsf(robot_y - (int32_t)robot_y) * 100));
    OLED_Print(0, 42, line);

    snprintf(line, sizeof(line), "COV:%d%% MAP:%d",
             coverage_percent, map_cells);
    OLED_Print(0, 52, line);

    OLED_Update();
}


/* ================================================================
   GENERAL_Update — call this every loop iteration from main()
   ================================================================ */
void GENERAL_Update(void)
{
    /* ----------------------------------------------------------
       0.  REAL dt
           FIX: was hardcoded 0.01f — now measured from tick.
    ---------------------------------------------------------- */
    static uint32_t last_tick = 0;
    uint32_t now_tick = HAL_GetTick();
    float dt = (float)(now_tick - last_tick) * 0.001f;
    if(dt < 0.0001f) dt = 0.001f;   /* first call guard       */
    if(dt > 0.1f)    dt = 0.1f;     /* cap at 100ms           */
    last_tick = now_tick;


    /* ----------------------------------------------------------
       1.  READ SENSORS
    ---------------------------------------------------------- */

    ENCODER_Update();
    robot.enc[0] = ENCODER_GetCount(0);
    robot.enc[1] = ENCODER_GetCount(1);

    for(uint8_t i = 0; i < 4; i++)
        robot.ir[i] = IR_Read(i);

    /* FIX: ULTRASONIC_Read() returns raw timer ticks (µs).
       HC-SR04: distance_cm = pulse_µs / 58
       Assumes TIM2 prescaler = 83 → 1MHz → 1 tick = 1µs       */
    for(uint8_t i = 0; i < 4; i++)
        robot.us_cm[i] = (float)ULTRASONIC_Read(i) / 58.0f;

    MPU6500_ReadRaw(&imu_raw);
    HMC5883L_ReadRaw(&mag_raw);


    /* ----------------------------------------------------------
       2.  IMU FUSION
    ---------------------------------------------------------- */

    STABLE_Update(&imu_raw, &mag_raw, dt);
    Orientation orient = STABLE_GetOrientation();


    /* ----------------------------------------------------------
       3.  WHEEL ODOMETRY
           FIX: was passing cumulative totals — now passes delta
                (distance since last call) as odometry requires.
           FIX: HC-020K has no direction pin — apply sign from
                commanded motor direction.
    ---------------------------------------------------------- */

    static int32_t prev_enc[2] = {0, 0};

    int32_t delta[2];
    delta[0] = robot.enc[0] - prev_enc[0];
    delta[1] = robot.enc[1] - prev_enc[1];
    prev_enc[0] = robot.enc[0];
    prev_enc[1] = robot.enc[1];

    /* Apply direction sign — HC-020K counts up regardless of direction */
    if(motor_dir[0] == MOTOR_BACKWARD) delta[0] = -delta[0];
    if(motor_dir[1] == MOTOR_BACKWARD) delta[1] = -delta[1];

    float left_dist  = (float)delta[0] * TICKS_TO_METERS;
    float right_dist = (float)delta[1] * TICKS_TO_METERS;

    ODOM_Update(left_dist, right_dist, dt);
    RobotPose pose = ODOM_GetPose();


    /* ----------------------------------------------------------
       4.  SLAM — fuse IMU yaw into odometry theta
    ---------------------------------------------------------- */

    SLAM_Update(&pose, orient);


    /* ----------------------------------------------------------
       5.  OBSTACLE DETECTION  (cm, not raw ticks)
    ---------------------------------------------------------- */

    int obstacle = (robot.us_cm[0] < OBSTACLE_CM ||
                    robot.us_cm[1] < OBSTACLE_CM) ? 1 : 0;


    /* ----------------------------------------------------------
       6.  COVERAGE PLANNER
           FIX: pass orient.yaw so TURN state can detect when
                the physical turn is complete.
    ---------------------------------------------------------- */

    float target_heading = 0.0f;
    int   base_speed     = 0;

    COVERAGE_Update(pose,
                    orient.yaw,
                    obstacle,
                    &target_heading,
                    &base_speed);


    /* ----------------------------------------------------------
       7.  HEADING CONTROL — PID → per-wheel speeds
    ---------------------------------------------------------- */

    int left_out  = 0;
    int right_out = 0;

    MOTION_DriveStraight(target_heading,
                         orient.yaw,
                         (float)base_speed,
                         dt,
                         &left_out,
                         &right_out);


    /* ----------------------------------------------------------
       8.  MOTOR OUTPUT
           FIX: was passing unclamped int to uint8_t — wraps
                above 255.  Now clamped to ±999 (ARR value).
    ---------------------------------------------------------- */

    if(left_out  >  MOTOR_OUT_MAX) left_out  =  MOTOR_OUT_MAX;
    if(left_out  < -MOTOR_OUT_MAX) left_out  = -MOTOR_OUT_MAX;
    if(right_out >  MOTOR_OUT_MAX) right_out =  MOTOR_OUT_MAX;
    if(right_out < -MOTOR_OUT_MAX) right_out = -MOTOR_OUT_MAX;

    motor_dir[0] = (left_out  >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    motor_dir[1] = (right_out >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

    MOTOR_Set(0, motor_dir[0], (uint16_t)abs(left_out));
    MOTOR_Set(1, motor_dir[1], (uint16_t)abs(right_out));


    /* ----------------------------------------------------------
       9.  UPDATE GLOBALS + OLED
    ---------------------------------------------------------- */

    heading = orient.yaw;
    robot_x = pose.x;
    robot_y = pose.y;

    GENERAL_OLED_Debug();
}
