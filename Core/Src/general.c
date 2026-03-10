#include "general.h"

// Example global structures
Map map;
UltrasonicSensor us[NUM_ULTRASONIC];

void GENERAL_Init(void)
{
    // Sensors
    US_Init(&htim2);
    IR_Init();
    ENC_Init(&htim1);

    // IMU
    MPU6500_Init(&hi2c1);
    HMC5883L_Init(&hi2c1);

    // Mapping
    Map_Init(&map);

    // Motor PWM
    MOTORPWM_Init();
}

void GENERAL_Update(void)
{
    US_Update();

    for(int i=0;i<NUM_ULTRASONIC;i++)
    {
        Map_UpdateUltrasonic(&map, US_GetDistance(i)/1000.0f, 0); // 0 = sensor relative angle
    }

    // IR update done via EXTI

    ENC_Update();

    // Stable fusion
    STABLE_Update();

    // Odometry update
    ODOM_Update();
}
