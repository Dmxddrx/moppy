#include "oled.h"
#include "i2c_oled.h"   // use an existing I2C_OLED library
#include "fonts.h"

static I2C_HandleTypeDef *oled_i2c;

/* ================================================================
   OLED_RecoverBus
   If I2C gets stuck in BUSY state, force-reset the peripheral.
   Toggles SCL 9 times to unstick any device holding SDA low,
   then reinitialises the HAL I2C peripheral.
   ================================================================ */

static void OLED_RecoverBus(void)
{
    /* Step 1 — abort any stuck HAL transaction */
    HAL_I2C_DeInit(oled_i2c);
    HAL_Delay(10);

    /* Step 2 — toggle SCL 9 times manually (PB10 = OLED_SCL) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = OLED_SCL_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OLED_SCL_GPIO_Port, &gpio);

    for(uint8_t i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* Step 3 — reinit I2C peripheral */
    HAL_I2C_Init(oled_i2c);
    HAL_Delay(10);

    /* Step 4 — reinit display */
    I2C_OLED_Init();
}

void OLED_Init(I2C_HandleTypeDef *hi2c)
{
    oled_i2c = hi2c;
    HAL_Delay(50);         /* let display power rail stabilise */
    I2C_OLED_Init();
}

void OLED_Clear(void) {
    I2C_OLED_Fill(I2C_OLED_COLOR_BLACK);

}

//Draw text into the buffer only (no immediate update)
void OLED_Print(uint8_t x, uint8_t y, const char *str) {
	I2C_OLED_GotoXY(x, y);
	I2C_OLED_Puts((char*)str, &Font_5x7, I2C_OLED_COLOR_WHITE);
}

/* ================================================================
   OLED_Update — flush buffer to display over I2C
   Checks I2C state first — recovers bus if stuck
   ================================================================ */
void OLED_Update(void)
{
	uint32_t start_time = HAL_GetTick();

	    /* 1. Wait patiently for the previous DMA background transfer to finish */
	    while (HAL_I2C_GetState(oled_i2c) != HAL_I2C_STATE_READY)
	    {
	        /* 2. If it takes longer than 50ms, the bus is actually crashed. Recover it! */
	        if ((HAL_GetTick() - start_time) > 50) {
	            OLED_RecoverBus();
	            return;
	        }
	    }

    I2C_OLED_UpdateScreen();
}

// New: clear a small area (erase a line/field without clearing whole screen)
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    I2C_OLED_DrawFilledRectangle(x, y, w, h, I2C_OLED_COLOR_BLACK);
}

void OLED_Rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    I2C_OLED_DrawFilledRectangle(x, y, w, h, I2C_OLED_COLOR_WHITE);
}

// New: Draw an empty white outline
void OLED_DrawOutline(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    I2C_OLED_DrawRectangle(x, y, w, h, I2C_OLED_COLOR_WHITE);
}
/* ═══════════════════════════════════════════════════════════════ */
/* NEW: Advanced Graphical Tools for the Compass                   */
/* ═══════════════════════════════════════════════════════════════ */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r) {
    I2C_OLED_DrawCircle(x, y, r, I2C_OLED_COLOR_WHITE);
}

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    I2C_OLED_DrawLine(x1, y1, x2, y2, I2C_OLED_COLOR_WHITE);
}
/* ─────────────────────────────────────────────────────────────── */
/* OLED_DrawPixel — Bridging to the low-level library              */
/* color: 1 = White, 0 = Black                                     */
/* ─────────────────────────────────────────────────────────────── */
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (color) {
    	I2C_OLED_DrawPixel(x, y, I2C_OLED_COLOR_WHITE);
    } else {
    	I2C_OLED_DrawPixel(x, y, I2C_OLED_COLOR_BLACK);
    }
}
