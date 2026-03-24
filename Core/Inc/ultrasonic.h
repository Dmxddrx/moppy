#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include <stdint.h>

/* ── Physics ────────────────────────────────────────────────────
   V = 331.3 + 0.606 × T    (T in °C)
   At T = 30°C:  V = 331.3 + 18.18 = 349.48 m/s
   Distance = (pulse_µs × V) / 2 000 000                        */
#define US_SOUND_SPEED_MPS   349.48f    /* m/s at 30 °C           */
#define US_MAX_DISTANCE_M      4.00f    /* SR04 practical max      */
#define US_TIMEOUT_MS           50U     /* no-echo timeout         */

typedef enum {
    US_NO_ECHO = 0,   /* triggered but no echo — OOR or disconnected */
    US_OK      = 1,   /* echo received                               */
} US_Status;

typedef struct {
    uint32_t  pulse_us;   /* raw echo pulse width (µs)               */
    float     distance_m; /* converted distance (m), -1 if invalid   */
    uint8_t   ready;      /* 1 = new measurement available           */
    US_Status status;
} ULTRASONIC_Data;

/* ── CubeMX notes ───────────────────────────────────────────────
   TIM2 configured as Input Capture, 4 channels on PA0–PA3
   Prescaler = 167  →  1 MHz clock  →  1 count = 1 µs
   IC polarity = TIM_ICPOLARITY_BOTHEDGE on every channel
   TIM2_IRQn enabled                                             */

void      ULTRASONIC_Init(void);
void      ULTRASONIC_Trigger(uint8_t index);  /* 0–3              */
void      ULTRASONIC_CaptureCallback(uint32_t channel); /* from HAL IC cb */
void      ULTRASONIC_CheckTimeout(void);      /* call from main loop */
float     ULTRASONIC_GetDistance(uint8_t index); /* metres, -1=invalid */

extern ULTRASONIC_Data ultrasonic[4];

#endif /* ULTRASONIC_H */
