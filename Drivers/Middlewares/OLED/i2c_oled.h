
#ifndef I2C_OLED_H
#define I2C_OLED_H 100

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include "stm32f4xx_hal.h"

#include "../OLED/fonts.h"

#include "stdlib.h"
#include "string.h"


/* I2C address */
#ifndef I2C_OLED_I2C_ADDR
#define I2C_OLED_I2C_ADDR         0x78
//#define I2C_OLED_I2C_ADDR       0x7A
#endif

/* I2C_OLED settings */
/* I2C_OLED width in pixels */
#ifndef I2C_OLED_WIDTH
#define I2C_OLED_WIDTH            128
#endif
/* I2C_OLED LCD height in pixels */
#ifndef I2C_OLED_HEIGHT
#define I2C_OLED_HEIGHT           64
#endif


typedef enum {
	I2C_OLED_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	I2C_OLED_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} I2C_OLED_COLOR_t;

void i2c_oled_I2C_SetHandle(I2C_HandleTypeDef *hi2c);
uint8_t I2C_OLED_Init(void);
void I2C_OLED_UpdateScreen(void);
void I2C_OLED_ToggleInvert(void);
void I2C_OLED_Fill(I2C_OLED_COLOR_t Color);
void I2C_OLED_DrawPixel(uint16_t x, uint16_t y, I2C_OLED_COLOR_t color);
void I2C_OLED_GotoXY(uint16_t x, uint16_t y);
char I2C_OLED_Putc(char ch, FontDef_t* Font, I2C_OLED_COLOR_t color);
char I2C_OLED_Puts(char* str, FontDef_t* Font, I2C_OLED_COLOR_t color);
void I2C_OLED_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, I2C_OLED_COLOR_t c);
void I2C_OLED_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, I2C_OLED_COLOR_t c);
void I2C_OLED_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, I2C_OLED_COLOR_t c);
void I2C_OLED_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, I2C_OLED_COLOR_t color);
void I2C_OLED_DrawCircle(int16_t x0, int16_t y0, int16_t r, I2C_OLED_COLOR_t c);
void I2C_OLED_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, I2C_OLED_COLOR_t c);

#ifndef i2c_oled_I2C_TIMEOUT
#define i2c_oled_I2C_TIMEOUT					20000
#endif

void i2c_oled_I2C_Init();
void i2c_oled_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
void i2c_oled_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);
void I2C_OLED_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);


void I2C_OLED_ScrollRight(uint8_t start_row, uint8_t end_row);


void I2C_OLED_ScrollLeft(uint8_t start_row, uint8_t end_row);


void I2C_OLED_Scrolldiagright(uint8_t start_row, uint8_t end_row);


void I2C_OLED_Scrolldiagleft(uint8_t start_row, uint8_t end_row);



void I2C_OLED_Stopscroll(void);


// inverts the display i = 1->inverted, i = 0->normal

void I2C_OLED_InvertDisplay (int i);






// clear the display

void I2C_OLED_Clear (void);


/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
