#ifndef HMC5883L_H
#define HMC5883L_H

#include "main.h"
#include <stdint.h>

#define HMC5883L_ADDR (0x1E << 1)

typedef struct
{
    int16_t mx;
    int16_t my;
    int16_t mz;

} HMC5883L_RawData;

void HMC5883L_Init(I2C_HandleTypeDef *hi2c);
void HMC5883L_ReadRaw(HMC5883L_RawData *data);

#endif
