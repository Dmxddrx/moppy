#include "motor.h"

/* ================================================================
   MOTOR.C
   Uses pin name defines from CubeMX-generated main.h directly.
   No raw GPIO_PIN_X values — all names match main.h exactly.

   Motor → TB6612FNG mapping:
   M1  TB6612#1 PWMA  AIN1=PC0  AIN2=PC1   PWM=PA6  TIM3_CH1
   M2  TB6612#1 PWMB  BIN1=PC4  BIN2=PC5   PWM=PA7  TIM3_CH2
   M3  TB6612#2 PWMA  AIN1=PD5  AIN2=PD6   PWM=PB0  TIM3_CH3
   M4  TB6612#2 PWMB  BIN1=PC8  BIN2=PC9   PWM=PD12 TIM4_CH1
   M5  TB6612#3 PWMA  AIN1=PE4  AIN2=PE5   PWM=PD13 TIM4_CH2
   M6  TB6612#3 PWMB  BIN1=PE6  BIN2=PE7   PWM=PD14 TIM4_CH3

   STBY mapping:
   MD1 (M1,M2) → PD2
   MD2 (M3,M4) → PD8
   MD3 (M5,M6) → PD9
   ================================================================ */

/* ----------------------------------------------------------------
   STBY pins — one per driver, controls 2 motors each
   ---------------------------------------------------------------- */
static GPIO_TypeDef* const STBY_PORT[3] = {
    M1_STBY_GPIO_Port,   /* MD1 → M1, M2 */
    M2_STBY_GPIO_Port,   /* MD2 → M3, M4 */
    M3_STBY_GPIO_Port    /* MD3 → M5, M6 */
};

static const uint16_t STBY_PIN[3] = {
    M1_STBY_Pin,         /* PD2 */
    M2_STBY_Pin,         /* PD8 */
    M3_STBY_Pin          /* PD9 */
};

/* Motor index → driver index lookup */
static const uint8_t MOTOR_TO_DRIVER[6] = {
    0,   /* M1 → MD1 */
    0,   /* M2 → MD1 */
    1,   /* M3 → MD2 */
    1,   /* M4 → MD2 */
    2,   /* M5 → MD3 */
    2    /* M6 → MD3 */
};

/* Direction pin 1 per motor (AIN1 or BIN1) */
static GPIO_TypeDef* const DIR1_PORT[6] = {
    M1_AIN1_GPIO_Port,   /* M1 */
    M2_BIN1_GPIO_Port,   /* M2 */
    M3_AIN1_GPIO_Port,   /* M3 */
    M4_BIN1_GPIO_Port,   /* M4 */
    M5_AIN1_GPIO_Port,   /* M5 */
    M6_BIN1_GPIO_Port    /* M6 */
};

static const uint16_t DIR1_PIN[6] = {
    M1_AIN1_Pin,         /* M1 PC0 */
    M2_BIN1_Pin,         /* M2 PC4 */
    M3_AIN1_Pin,         /* M3 PD5 */
    M4_BIN1_Pin,         /* M4 PC8 */
    M5_AIN1_Pin,         /* M5 PE4 */
    M6_BIN1_Pin          /* M6 PE6 */
};

/* Direction pin 2 per motor (AIN2 or BIN2) */
static GPIO_TypeDef* const DIR2_PORT[6] = {
    M1_AIN2_GPIO_Port,   /* M1 */
    M2_BIN2_GPIO_Port,   /* M2 */
    M3_AIN2_GPIO_Port,   /* M3 */
    M4_BIN2_GPIO_Port,   /* M4 */
    M5_AIN2_GPIO_Port,   /* M5 */
    M6_BIN2_GPIO_Port    /* M6 */
};

static const uint16_t DIR2_PIN[6] = {
    M1_AIN2_Pin,         /* M1 PC1 */
    M2_BIN2_Pin,         /* M2 PC5 */
    M3_AIN2_Pin,         /* M3 PD6 */
    M4_BIN2_Pin,         /* M4 PC9 */
    M5_AIN2_Pin,         /* M5 PE5 */
    M6_BIN2_Pin          /* M6 PE7 */
};

/* ================================================================
   MOTOR_EnableDriver — pull STBY HIGH for one driver
   ================================================================ */
static void MOTOR_EnableDriver(uint8_t driver_index)
{
    if(driver_index >= 3) return;
    HAL_GPIO_WritePin(STBY_PORT[driver_index],
                      STBY_PIN[driver_index],
                      GPIO_PIN_SET);
}

/* ================================================================
   MOTOR_DisableDriver — pull STBY LOW (standby/coast)
   ================================================================ */
static void MOTOR_DisableDriver(uint8_t driver_index)
{
    if(driver_index >= 3) return;
    HAL_GPIO_WritePin(STBY_PORT[driver_index],
                      STBY_PIN[driver_index],
                      GPIO_PIN_RESET);
}

void MOTOR_Init(void)
{
    /* Enable all 3 drivers immediately */
    for(uint8_t i = 0; i < 3; i++)
        MOTOR_EnableDriver(i);

    MOTORPWM_Init();
}

void MOTOR_SetDir(uint8_t motor_index, uint8_t direction)
{
    if(motor_index >= 6) return;

    switch(direction)
    {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(DIR1_PORT[motor_index], DIR1_PIN[motor_index], GPIO_PIN_SET);
            HAL_GPIO_WritePin(DIR2_PORT[motor_index], DIR2_PIN[motor_index], GPIO_PIN_RESET);
            break;

        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(DIR1_PORT[motor_index], DIR1_PIN[motor_index], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DIR2_PORT[motor_index], DIR2_PIN[motor_index], GPIO_PIN_SET);
            break;

        default: /* MOTOR_STOP */
            HAL_GPIO_WritePin(DIR1_PORT[motor_index], DIR1_PIN[motor_index], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DIR2_PORT[motor_index], DIR2_PIN[motor_index], GPIO_PIN_RESET);
            break;
    }
}

/* ================================================================
   MOTOR_Set — speed: 0 – MOTOR_SPEED_MAX (999)
   ================================================================ */
void MOTOR_Set(uint8_t motor_index, uint8_t direction, uint16_t speed)
{
    if(motor_index >= 6) return;
    if(speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;

    MOTOR_SetDir(motor_index, direction);
    MOTORPWM_SetOne(motor_index, (int16_t)speed);
}

/* ================================================================
   MOTOR_StandbyAll — disable all drivers (coast all motors)
   ================================================================ */
void MOTOR_StandbyAll(void)
{
    for(uint8_t i = 0; i < 3; i++)
        MOTOR_DisableDriver(i);
}

/* ================================================================
   MOTOR_WakeAll — re-enable all drivers
   ================================================================ */
void MOTOR_WakeAll(void)
{
    for(uint8_t i = 0; i < 3; i++)
        MOTOR_EnableDriver(i);
}

/* ================================================================
   MOTOR_GetDirection — returns +1 forward, -1 backward, 0 stop
   Used by odometry to apply sign to encoder counts
   ================================================================ */
int8_t MOTOR_GetDirection(uint8_t motor_index)
{
    if(motor_index >= 6) return 0;

    uint8_t in1 = HAL_GPIO_ReadPin(DIR1_PORT[motor_index],
                                    DIR1_PIN[motor_index]);
    uint8_t in2 = HAL_GPIO_ReadPin(DIR2_PORT[motor_index],
                                    DIR2_PIN[motor_index]);

    if(in1 == GPIO_PIN_SET   && in2 == GPIO_PIN_RESET) return  1;
    if(in1 == GPIO_PIN_RESET && in2 == GPIO_PIN_SET)   return -1;
    return 0;   /* stop or brake */
}
