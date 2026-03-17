#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include <stdint.h>

typedef enum {
    US_NO_ECHO  = 0,   /* triggered but no echo — disconnected or out of range */
    US_OK       = 1,   /* echo received — sensor responding */
} US_Status ;

typedef struct
{
    uint32_t pulse_us;   /* raw echo pulse width in microseconds */
    uint8_t  ready;      /* 1 = new measurement available        */
    US_Status status;     /* connection status                    */
} ULTRASONIC_Data;

void ULTRASONIC_Init(void);
void ULTRASONIC_Trigger(uint8_t index);
void ULTRASONIC_CaptureCallback(uint32_t channel);   /* call from HAL IC callback */

extern ULTRASONIC_Data ultrasonic[4];

#endif
