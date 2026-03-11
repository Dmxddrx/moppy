#include "general.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

typedef struct {
    int16_t encoder_counts[2];
    uint8_t ir_values[4];
    float ultrasonic_distances[4];
} RobotState;

RobotState robot;

static uint8_t oled_page = 0;
static uint32_t last_oled_switch = 0;

static MPU6500_RawData imu_raw;
static HMC5883L_RawData mag_raw;

extern float heading;
extern float robot_x;
extern float robot_y;
extern int motion_state;
extern int coverage_percent;
extern int map_cells;

char line[32];


// Initialize all modules
void GENERAL_Init(void)
{
    MOTOR_Init();
    ENCODER_Init();
    IR_Init();
    ULTRASONIC_Init();

    OLED_Init(&hi2c1);
    OLED_Clear();
}

#include <stdio.h>

extern float heading;
extern float robot_x;
extern float robot_y;
extern int motion_state;
extern int coverage_percent;
extern int map_cells;

void GENERAL_OLED_Debug(void)
{
    char line[32];

    OLED_Clear();

    // Heading + motion
    sprintf(line,"HD:%3d M:%d",(int)heading,motion_state);
    OLED_Print(0,0,line);

    // IR sensors
    sprintf(line,"IR:%d %d %d %d",
            robot.ir_values[0],
            robot.ir_values[1],
            robot.ir_values[2],
            robot.ir_values[3]);
    OLED_Print(0,10,line);

    // Ultrasonic
    sprintf(line,"US:%2d %2d %2d %2d",
            (int)robot.ultrasonic_distances[0],
            (int)robot.ultrasonic_distances[1],
            (int)robot.ultrasonic_distances[2],
            (int)robot.ultrasonic_distances[3]);
    OLED_Print(0,20,line);

    // Encoders
    sprintf(line,"ENC:%ld %ld",
            robot.encoder_counts[0],
            robot.encoder_counts[1]);
    OLED_Print(0,30,line);

    // Odometry
    sprintf(line,"XY:%d.%02d %d.%02d",
            (int)robot_x,(int)(robot_x*100)%100,
            (int)robot_y,(int)(robot_y*100)%100);
    OLED_Print(0,40,line);

    // Coverage + mapping
    sprintf(line,"COV:%d%% MAP:%d",
            coverage_percent,
            map_cells);
    OLED_Print(0,50,line);

    OLED_Update();
}

// OLED diagnostic display
/*void GENERAL_OLED_Update(void)
{
    if(HAL_GetTick() - last_oled_switch > 1500)
    {
        oled_page++;
        if(oled_page > 3) oled_page = 0;
        last_oled_switch = HAL_GetTick();
    }

    OLED_Clear();

    switch(oled_page)
    {

        // PAGE 0 — IR + ULTRASONIC
        case 0:

            sprintf(line,"IR %d %d %d %d",
                    robot.ir_values[0],
                    robot.ir_values[1],
                    robot.ir_values[2],
                    robot.ir_values[3]);
            OLED_Print(0,0,line);

            sprintf(line,"US %.1f %.1f",
                    robot.ultrasonic_distances[0],
                    robot.ultrasonic_distances[1]);
            OLED_Print(0,12,line);

            sprintf(line,"US %.1f %.1f",
                    robot.ultrasonic_distances[2],
                    robot.ultrasonic_distances[3]);
            OLED_Print(0,24,line);

        break;



        // PAGE 1 — ENCODER + MOTION + PID
        case 1:

            sprintf(line,"ENC L:%ld R:%ld",
                    robot.encoder_counts[0],
                    robot.encoder_counts[1]);
            OLED_Print(0,0,line);

            sprintf(line,"PID %.2f %.2f",
                    PID_GetLeft(),
                    PID_GetRight());
            OLED_Print(0,12,line);

            sprintf(line,"MOTION %d",
                    MOTION_GetState());
            OLED_Print(0,24,line);

        break;



        // PAGE 2 — IMU
        case 2:

            sprintf(line,"MPU A %.2f %.2f",
                    MPU6500.ax,
                    MPU6500.ay);
            OLED_Print(0,0,line);

            sprintf(line,"G %.2f %.2f",
                    MPU6500.gx,
                    MPU6500.gy);
            OLED_Print(0,12,line);

            sprintf(line,"MAG %.1f",
                    HMC5883L.heading);
            OLED_Print(0,24,line);

        break;



        // PAGE 3 — SLAM + MAPPING
        case 3:

            sprintf(line,"X %.1f Y %.1f",
                    ODOMETRY_GetX(),
                    ODOMETRY_GetY());
            OLED_Print(0,0,line);

            sprintf(line,"TH %.1f",
                    ODOMETRY_GetTheta());
            OLED_Print(0,12,line);

            sprintf(line,"COV %d",
                    COVERAGE_GetProgress());
            OLED_Print(0,24,line);

            sprintf(line,"MAP %d",
                    MAPPING_GetCells());
            OLED_Print(0,36,line);

        break;

    }

    OLED_Update();
}*/


// Call this in main loop
void GENERAL_Update(void)
{
    float dt = 0.01f;

    /* ---------------------------
       1. READ SENSORS
    ----------------------------*/

    ENCODER_Update();

    int32_t enc_left  = ENCODER_GetCount(0);
    int32_t enc_right = ENCODER_GetCount(1);

    for(uint8_t i=0;i<4;i++)
        robot.ir_values[i] = IR_Read(i);

    for(uint8_t i=0;i<4;i++)
        robot.ultrasonic_distances[i] = ULTRASONIC_Read(i);

    MPU6500_ReadRaw(&imu_raw);
    HMC5883L_ReadRaw(&mag_raw);


    /* ---------------------------
       2. IMU ORIENTATION
    ----------------------------*/

    STABLE_Update(&imu_raw, &mag_raw, dt);

    Orientation orient = STABLE_GetOrientation();


    /* ---------------------------
       3. WHEEL ODOMETRY
    ----------------------------*/

    float left_dist  = enc_left  * 0.001f;
    float right_dist = enc_right * 0.001f;

    ODOM_Update(left_dist, right_dist, dt);

    RobotPose pose = ODOM_GetPose();


    /* ---------------------------
       4. SLAM YAW CORRECTION
    ----------------------------*/

    SLAM_Update(&pose, orient);


    /* ---------------------------
       5. OBSTACLE DETECTION
    ----------------------------*/

    int obstacle = 0;

    if(robot.ultrasonic_distances[0] < 20 ||
       robot.ultrasonic_distances[1] < 20)
        obstacle = 1;


    /* ---------------------------
       6. COVERAGE PLANNER
    ----------------------------*/

    float target_heading = 0;
    int base_speed = 0;

    COVERAGE_Update(pose,
                    obstacle,
                    &target_heading,
                    &base_speed);


    /* ---------------------------
       7. HEADING CONTROL
    ----------------------------*/

    int left_motor = 0;
    int right_motor = 0;

    MOTION_DriveStraight(
        target_heading,
        orient.yaw,
        base_speed,
        dt,
        &left_motor,
        &right_motor
    );


    /* ---------------------------
       8. MOTOR OUTPUT
    ----------------------------*/

    MOTOR_Set(0, left_motor > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD, abs(left_motor));
    MOTOR_Set(1, right_motor > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD, abs(right_motor));


    /* ---------------------------
       9. DEBUG / OLED
    ----------------------------*/

    heading = orient.yaw;
    robot_x = pose.x;
    robot_y = pose.y;

    GENERAL_OLED_Debug();
}
