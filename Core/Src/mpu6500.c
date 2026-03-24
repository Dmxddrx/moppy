#include "mpu6500.h"

/* I2C1: PB6 (SCL1), PB7 (SDA1) — shared with HMC5883L          */
extern I2C_HandleTypeDef hi2c1;

/* ── Register map ───────────────────────────────────────────────*/
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_GYRO_XOUT_H   0x43
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

/* ── Scale ──────────────────────────────────────────────────────*/
#define ACCEL_SCALE_MS2   (9.81f / 16384.0f)  /* ±2 g range       */
#define GYRO_SCALE_DPS    (1.0f  / 131.0f)    /* ±250 °/s range   */

/* ── State ──────────────────────────────────────────────────────*/
static int16_t s_gyro_bias[3] = {0, 0, 0};

/* ─────────────────────────────────────────────────────────────── */
static HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, reg, 1, &val, 1, 10);
}
static HAL_StatusTypeDef ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, reg, 1, buf, len, 20);
}

/* ─────────────────────────────────────────────────────────────── */
MPU_Status MPU6500_Check(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR,
                          REG_WHO_AM_I, 1, &id, 1, 100) != HAL_OK)
        return MPU_NO_I2C;
    /* MPU-6500 = 0x70, MPU-6050 = 0x68, MPU-9250 = 0x71         */
    if (id != 0x70 && id != 0x68 && id != 0x71)
        return MPU_WRONG_ID;
    return MPU_OK;
}

/* ─────────────────────────────────────────────────────────────── */
void MPU6500_Init(void)
{
    HAL_Delay(100);
    WriteReg(REG_PWR_MGMT_1, 0x80);   /* device reset              */
    HAL_Delay(100);
    WriteReg(REG_PWR_MGMT_1, 0x01);   /* clock = PLL gyro X        */
    HAL_Delay(10);
    WriteReg(REG_SMPLRT_DIV,  0x07);  /* 8 kHz / 8 = 1 kHz sample  */
    WriteReg(REG_CONFIG,       0x03); /* DLPF_CFG 3 → 44 Hz BW     */
    WriteReg(REG_GYRO_CONFIG,  0x00); /* ±250 °/s                   */
    WriteReg(REG_ACCEL_CONFIG, 0x00); /* ±2 g                       */
    HAL_Delay(100);

    /* Calibrate gyro on power-up (keep robot stationary!)         */
    MPU6500_CalibrateGyro();
}

/* ─────────────────────────────────────────────────────────────── */
/*  MPU6500_CalibrateGyro                                           */
/*  Averages 200 gyro readings to find static bias.                 */
/*  Robot MUST be motionless during this call (~1 second).          */
/* ─────────────────────────────────────────────────────────────── */
void MPU6500_CalibrateGyro(void)
{
    int32_t sum[3] = {0, 0, 0};
    uint8_t buf[6];
    const int N = 200;

    for (int i = 0; i < N; i++) {
        if (ReadRegs(REG_GYRO_XOUT_H, buf, 6) == HAL_OK) {
            sum[0] += (int16_t)((buf[0] << 8) | buf[1]);
            sum[1] += (int16_t)((buf[2] << 8) | buf[3]);
            sum[2] += (int16_t)((buf[4] << 8) | buf[5]);
        }
        HAL_Delay(5);
    }
    s_gyro_bias[0] = (int16_t)(sum[0] / N);
    s_gyro_bias[1] = (int16_t)(sum[1] / N);
    s_gyro_bias[2] = (int16_t)(sum[2] / N);
}

/* ─────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef MPU6500_ReadRaw(MPU6500_RawData *data)
{
    uint8_t buf[14];
    HAL_StatusTypeDef st = ReadRegs(REG_ACCEL_XOUT_H, buf, 14);
    if (st != HAL_OK) return st;

    data->ax = (int16_t)((buf[0]  << 8) | buf[1]);
    data->ay = (int16_t)((buf[2]  << 8) | buf[3]);
    data->az = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature — not used here                     */
    data->gx = (int16_t)((buf[8]  << 8) | buf[9])  - s_gyro_bias[0];
    data->gy = (int16_t)((buf[10] << 8) | buf[11]) - s_gyro_bias[1];
    data->gz = (int16_t)((buf[12] << 8) | buf[13]) - s_gyro_bias[2];

    return HAL_OK;
}

/* ─────────────────────────────────────────────────────────────── */
MPU6500_PhysData MPU6500_GetPhysical(const MPU6500_RawData *raw)
{
    MPU6500_PhysData p;
    p.ax = raw->ax * ACCEL_SCALE_MS2;
    p.ay = raw->ay * ACCEL_SCALE_MS2;
    p.az = raw->az * ACCEL_SCALE_MS2;
    p.gx = raw->gx * GYRO_SCALE_DPS;
    p.gy = raw->gy * GYRO_SCALE_DPS;
    p.gz = raw->gz * GYRO_SCALE_DPS;
    return p;
}
