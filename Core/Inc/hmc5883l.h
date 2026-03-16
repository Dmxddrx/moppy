#ifndef HMC5883L_H
#define HMC5883L_H

#include "main.h"
#include <stdint.h>

#define HMC5883L_ADDR (0x1E << 1)

typedef enum {
    HMC_OK       = 0,
    HMC_NO_I2C   = 1,
    HMC_WRONG_ID = 2,
} HMC_Status;

typedef struct
{
    int16_t mx;
    int16_t my;
    int16_t mz;

} HMC5883L_RawData;

typedef struct
{
    int16_t mx;
    int16_t my;
    int16_t mz;
} HMC5883L_SelfTestData;

HMC_Status HMC5883L_Check(I2C_HandleTypeDef *hi2c);
void HMC5883L_Init(void);
void HMC5883L_ReadRaw(HMC5883L_RawData *data);
HMC5883L_SelfTestData HMC5883L_SelfTest(void);

#endif
