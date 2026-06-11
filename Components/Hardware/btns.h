#ifndef BTNS_H
#define BTNS_H

#include "main.h"
#include <stdint.h>

typedef enum {
    BTN_IDLE    = 0,
    BTN_PRESSED = 1,
} BTN_State;

void      BTNS_Init(void);
void      BTNS_Update(void);         /* call every loop in GENERAL_Update */
BTN_State BTNS_Get_OLEDPage(void);   /* returns BTN_PRESSED once per press */

#endif
