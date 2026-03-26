#include "oled.h"
#include "ssd1306.h"   // use an existing SSD1306 library
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
    SSD1306_Init();
}

void OLED_Init(I2C_HandleTypeDef *hi2c)
{
    oled_i2c = hi2c;
    HAL_Delay(50);         /* let display power rail stabilise */
    SSD1306_Init();
}

void OLED_Clear(void) {
    SSD1306_Fill(SSD1306_COLOR_BLACK);

}

//Draw text into the buffer only (no immediate update)
void OLED_Print(uint8_t x, uint8_t y, const char *str) {
	SSD1306_GotoXY(x, y);
	SSD1306_Puts((char*)str, &Font_5x7, SSD1306_COLOR_WHITE);
}

/* ================================================================
   OLED_Update — flush buffer to display over I2C
   Checks I2C state first — recovers bus if stuck
   ================================================================ */
void OLED_Update(void)
{
    /* Check if I2C bus is stuck before attempting transfer */
    if(HAL_I2C_GetState(oled_i2c) != HAL_I2C_STATE_READY)
    {
        OLED_RecoverBus();
        return;   /* skip this frame — display will update next tick */
    }

    SSD1306_UpdateScreen();
}

// New: clear a small area (erase a line/field without clearing whole screen)
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    SSD1306_DrawFilledRectangle(x, y, w, h, SSD1306_COLOR_BLACK);
}

void OLED_Rectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    SSD1306_DrawFilledRectangle(x, y, w, h, SSD1306_COLOR_WHITE);
}

// New: Draw an empty white outline
void OLED_DrawOutline(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    SSD1306_DrawRectangle(x, y, w, h, SSD1306_COLOR_WHITE);
}
