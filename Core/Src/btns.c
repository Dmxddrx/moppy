#include "btns.h"

#define DEBOUNCE_MS   200

static uint8_t   last_btn_oledpage  = 1;   /* PULLUP idle = 1 */
static uint32_t  debounce_tick      = 0;
static BTN_State oledpage_event     = BTN_IDLE;

/* ================================================================
   BTNS_Init
   GPIO already configured in CubeMX — nothing to init here.
   Kept for consistency with other driver patterns.
   ================================================================ */
void BTNS_Init(void)
{
    /* nothing needed — GPIO init done by MX_GPIO_Init in main.c */
}

/* ================================================================
   BTNS_Update — call every loop
   Detects falling edge with debounce, sets event flag.
   ================================================================ */
void BTNS_Update(void)
{
    uint8_t btn = HAL_GPIO_ReadPin(BTN_OLED_PAGE_GPIO_Port, BTN_OLED_PAGE_Pin);

    /* Falling edge = button pressed */
    if(last_btn_oledpage == 1 && btn == 0)
    {
        if(HAL_GetTick() - debounce_tick >= DEBOUNCE_MS)
        {
            oledpage_event = BTN_PRESSED;
            debounce_tick  = HAL_GetTick();
        }
    }

    last_btn_oledpage = btn;
}

/* ================================================================
   BTNS_Get_OLEDPage
   Returns BTN_PRESSED once then clears — like reading a flag.
   ================================================================ */
BTN_State BTNS_Get_OLEDPage(void)
{
    BTN_State state = oledpage_event;
    oledpage_event  = BTN_IDLE;   /* clear after read */
    return state;
}
