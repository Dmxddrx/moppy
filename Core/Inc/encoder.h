#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

/*
   Encoder 0 → TIM1  ENC1_A = PE9 (CH1), ENC1_B = PE11 (CH2 — disconnected)
   Encoder 1 → TIM8  ENC2_A = PC6 (CH1), ENC2_B = PC7  (CH2 — disconnected)
*/

#define ENCODER_COUNT   2

typedef enum {
    ENC_NO_SIGNAL = 0,   /* no pulses detected since init */
    ENC_OK        = 1,   /* pulses are being received     */
} ENC_Status;

ENC_Status ENCODER_GetStatus(uint8_t index);

void    ENCODER_Init(void);
void    ENCODER_IC_Callback(uint8_t index, uint32_t captured);
void    ENCODER_Update(void);             /* call from TIM6 1ms interrupt */

int32_t    ENCODER_GetCount(uint8_t index);
float      ENCODER_GetSpeed(uint8_t index);   /* smoothed — for OLED          */
float      ENCODER_GetSpeed_PID(uint8_t index); /* raw — for PID              */
ENC_Status ENCODER_GetStatus(uint8_t index);
void       ENCODER_Reset(uint8_t index);
#endif
