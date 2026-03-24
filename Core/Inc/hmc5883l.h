#ifndef HMC5883L_H
#define HMC5883L_H

#include "main.h"
#include <stdint.h>

#define HMC5883L_ADDR  (0x1E << 1)   /* 0x3C write, 0x3D read    */

typedef enum {
    HMC_OK       = 0,
    HMC_NO_I2C   = 1,
    HMC_WRONG_ID = 2,
} HMC_Status;

typedef struct {
    int16_t mx, my, mz;
} HMC5883L_RawData;

typedef struct {
    int16_t mx, my, mz;
} HMC5883L_SelfTestData;

HMC_Status            HMC5883L_Check(I2C_HandleTypeDef *hi2c);
void                  HMC5883L_Init(void);
void                  HMC5883L_ReadRaw(HMC5883L_RawData *data);
HMC5883L_SelfTestData HMC5883L_SelfTest(void);

/* Returns heading in degrees, 0 = magnetic North, 0–360 CW.
   Pass tilt-compensated mx2/my2 values from STABLE_Update.      */
float HMC5883L_GetHeading(float mx, float my);

#endif /* HMC5883L_H */
