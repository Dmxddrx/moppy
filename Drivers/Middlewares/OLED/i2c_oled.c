/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>

   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#include "../OLED/i2c_oled.h"

extern I2C_HandleTypeDef hi2c2;
/* Write command */
#define I2C_OLED_WRITECOMMAND(command)      i2c_oled_I2C_Write(I2C_OLED_I2C_ADDR, 0x00, (command))
/* Write data */
#define I2C_OLED_WRITEDATA(data)            i2c_oled_I2C_Write(I2C_OLED_I2C_ADDR, 0x40, (data))
/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))

/* I2C_OLED data buffer */
static uint8_t I2C_OLED_Buffer[I2C_OLED_WIDTH * I2C_OLED_HEIGHT / 8];

/* Private I2C_OLED structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} I2C_OLED_t;

/* Private variable */
static I2C_OLED_t I2C_OLED;


#define I2C_OLED_RIGHT_HORIZONTAL_SCROLL              0x26
#define I2C_OLED_LEFT_HORIZONTAL_SCROLL               0x27
#define I2C_OLED_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define I2C_OLED_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define I2C_OLED_DEACTIVATE_SCROLL                    0x2E // Stop scroll
#define I2C_OLED_ACTIVATE_SCROLL                      0x2F // Start scroll
#define I2C_OLED_SET_VERTICAL_SCROLL_AREA             0xA3 // Set scroll range

#define I2C_OLED_NORMALDISPLAY       0xA6
#define I2C_OLED_INVERTDISPLAY       0xA7


void I2C_OLED_ScrollRight(uint8_t start_row, uint8_t end_row)
{
  I2C_OLED_WRITECOMMAND (I2C_OLED_RIGHT_HORIZONTAL_SCROLL);  // send 0x26
  I2C_OLED_WRITECOMMAND (0x00);  // send dummy
  I2C_OLED_WRITECOMMAND(start_row);  // start page address
  I2C_OLED_WRITECOMMAND(0X00);  // time interval 5 frames
  I2C_OLED_WRITECOMMAND(end_row);  // end page address
  I2C_OLED_WRITECOMMAND(0X00);
  I2C_OLED_WRITECOMMAND(0XFF);
  I2C_OLED_WRITECOMMAND (I2C_OLED_ACTIVATE_SCROLL); // start scroll
}


void I2C_OLED_ScrollLeft(uint8_t start_row, uint8_t end_row)
{
  I2C_OLED_WRITECOMMAND (I2C_OLED_LEFT_HORIZONTAL_SCROLL);  // send 0x26
  I2C_OLED_WRITECOMMAND (0x00);  // send dummy
  I2C_OLED_WRITECOMMAND(start_row);  // start page address
  I2C_OLED_WRITECOMMAND(0X00);  // time interval 5 frames
  I2C_OLED_WRITECOMMAND(end_row);  // end page address
  I2C_OLED_WRITECOMMAND(0X00);
  I2C_OLED_WRITECOMMAND(0XFF);
  I2C_OLED_WRITECOMMAND (I2C_OLED_ACTIVATE_SCROLL); // start scroll
}


void I2C_OLED_Scrolldiagright(uint8_t start_row, uint8_t end_row)
{
  I2C_OLED_WRITECOMMAND(I2C_OLED_SET_VERTICAL_SCROLL_AREA);  // sect the area
  I2C_OLED_WRITECOMMAND (0x00);   // write dummy
  I2C_OLED_WRITECOMMAND(I2C_OLED_HEIGHT);

  I2C_OLED_WRITECOMMAND(I2C_OLED_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  I2C_OLED_WRITECOMMAND (0x00);
  I2C_OLED_WRITECOMMAND(start_row);
  I2C_OLED_WRITECOMMAND(0X00);
  I2C_OLED_WRITECOMMAND(end_row);
  I2C_OLED_WRITECOMMAND (0x01);
  I2C_OLED_WRITECOMMAND (I2C_OLED_ACTIVATE_SCROLL);
}


void I2C_OLED_Scrolldiagleft(uint8_t start_row, uint8_t end_row)
{
  I2C_OLED_WRITECOMMAND(I2C_OLED_SET_VERTICAL_SCROLL_AREA);  // sect the area
  I2C_OLED_WRITECOMMAND (0x00);   // write dummy
  I2C_OLED_WRITECOMMAND(I2C_OLED_HEIGHT);

  I2C_OLED_WRITECOMMAND(I2C_OLED_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  I2C_OLED_WRITECOMMAND (0x00);
  I2C_OLED_WRITECOMMAND(start_row);
  I2C_OLED_WRITECOMMAND(0X00);
  I2C_OLED_WRITECOMMAND(end_row);
  I2C_OLED_WRITECOMMAND (0x01);
  I2C_OLED_WRITECOMMAND (I2C_OLED_ACTIVATE_SCROLL);
}


void I2C_OLED_Stopscroll(void)
{
	I2C_OLED_WRITECOMMAND(I2C_OLED_DEACTIVATE_SCROLL);
}



void I2C_OLED_InvertDisplay (int i)
{
  if (i) I2C_OLED_WRITECOMMAND (I2C_OLED_INVERTDISPLAY);

  else I2C_OLED_WRITECOMMAND (I2C_OLED_NORMALDISPLAY);

}


void I2C_OLED_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color)
{

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++)
    {
        for(int16_t i=0; i<w; i++)
        {
            if(i & 7)
            {
               byte <<= 1;
            }
            else
            {
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }
            if(byte & 0x80) I2C_OLED_DrawPixel(x+i, y, color);
        }
    }
}


static I2C_HandleTypeDef *i2c_oled_hi2c = NULL;

void i2c_oled_I2C_SetHandle(I2C_HandleTypeDef *hi2c)
{
    i2c_oled_hi2c = hi2c;
}


uint8_t I2C_OLED_Init(void) {

	/* Init I2C */
	i2c_oled_I2C_Init();

	/* Check if LCD connected to I2C */
	if (HAL_I2C_IsDeviceReady(&hi2c2, I2C_OLED_I2C_ADDR, 1, 20000) != HAL_OK) {
		/* Return false */
		return 0;
	}

	/* A little delay */
	uint32_t p = 2500;
	while(p>0)
		p--;

	/* Init LCD */
	I2C_OLED_WRITECOMMAND(0xAE); //display off
	I2C_OLED_WRITECOMMAND(0x20); //Set Memory Addressing Mode
	I2C_OLED_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	I2C_OLED_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	I2C_OLED_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	I2C_OLED_WRITECOMMAND(0x00); //---set low column address
	I2C_OLED_WRITECOMMAND(0x10); //---set high column address
	I2C_OLED_WRITECOMMAND(0x40); //--set start line address
	I2C_OLED_WRITECOMMAND(0x81); //--set contrast control register
	I2C_OLED_WRITECOMMAND(0xFF);
	I2C_OLED_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	I2C_OLED_WRITECOMMAND(0xA6); //--set normal display
	I2C_OLED_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	I2C_OLED_WRITECOMMAND(0x3F); //
	I2C_OLED_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	I2C_OLED_WRITECOMMAND(0xD3); //-set display offset
	I2C_OLED_WRITECOMMAND(0x00); //-not offset
	I2C_OLED_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	I2C_OLED_WRITECOMMAND(0xF0); //--set divide ratio
	I2C_OLED_WRITECOMMAND(0xD9); //--set pre-charge period
	I2C_OLED_WRITECOMMAND(0x22); //
	I2C_OLED_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	I2C_OLED_WRITECOMMAND(0x12);
	I2C_OLED_WRITECOMMAND(0xDB); //--set vcomh
	I2C_OLED_WRITECOMMAND(0x20); //0x20,0.77xVcc
	I2C_OLED_WRITECOMMAND(0x8D); //--set DC-DC enable
	I2C_OLED_WRITECOMMAND(0x14); //
	I2C_OLED_WRITECOMMAND(0xAF); //--turn on I2C_OLED panel


	I2C_OLED_WRITECOMMAND(I2C_OLED_DEACTIVATE_SCROLL);

	/* Clear screen */
	I2C_OLED_Fill(I2C_OLED_COLOR_BLACK);

	/* Update screen */
	I2C_OLED_UpdateScreen();

	/* Set default values */
	I2C_OLED.CurrentX = 0;
	I2C_OLED.CurrentY = 0;

	/* Initialized OK */
	I2C_OLED.Initialized = 1;

	/* Return OK */
	return 1;
}

void I2C_OLED_UpdateScreen(void) {
    /* 1. Switch OLED to Horizontal Addressing Mode */
    I2C_OLED_WRITECOMMAND(0x20);
    I2C_OLED_WRITECOMMAND(0x00);

    /* 2. Set the bounding box for the screen */
    I2C_OLED_WRITECOMMAND(0x21); /* Column Address */
    I2C_OLED_WRITECOMMAND(0x00); /* Start: 0 */
    I2C_OLED_WRITECOMMAND(127);  /* End: 127 */

    I2C_OLED_WRITECOMMAND(0x22); /* Page Address */
    I2C_OLED_WRITECOMMAND(0x00); /* Start: Page 0 */
    I2C_OLED_WRITECOMMAND(7);    /* End: Page 7 */

    /* 3. Blast the ENTIRE 1,024-byte buffer in the background using DMA!
       - 0x40 is the OLED control byte meaning "Display Data"
       - 1 is the memory address size (I2C_MEMADD_SIZE_8BIT) */
    HAL_I2C_Mem_Write_DMA(&hi2c2, I2C_OLED_I2C_ADDR, 0x40, 1, I2C_OLED_Buffer, sizeof(I2C_OLED_Buffer));
}

void I2C_OLED_ToggleInvert(void) {
	uint16_t i;

	/* Toggle invert */
	I2C_OLED.Inverted = !I2C_OLED.Inverted;

	/* Do memory toggle */
	for (i = 0; i < sizeof(I2C_OLED_Buffer); i++) {
		I2C_OLED_Buffer[i] = ~I2C_OLED_Buffer[i];
	}
}

void I2C_OLED_Fill(I2C_OLED_COLOR_t color) {
	/* Set memory */
	memset(I2C_OLED_Buffer, (color == I2C_OLED_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(I2C_OLED_Buffer));
}

void I2C_OLED_DrawPixel(uint16_t x, uint16_t y, I2C_OLED_COLOR_t color) {
	if (
		x >= I2C_OLED_WIDTH ||
		y >= I2C_OLED_HEIGHT
	) {
		/* Error */
		return;
	}

	/* Check if pixels are inverted */
	if (I2C_OLED.Inverted) {
		color = (I2C_OLED_COLOR_t)!color;
	}

	/* Set color */
	if (color == I2C_OLED_COLOR_WHITE) {
		I2C_OLED_Buffer[x + (y / 8) * I2C_OLED_WIDTH] |= 1 << (y % 8);
	} else {
		I2C_OLED_Buffer[x + (y / 8) * I2C_OLED_WIDTH] &= ~(1 << (y % 8));
	}
}

void I2C_OLED_GotoXY(uint16_t x, uint16_t y) {
	/* Set write pointers */
	I2C_OLED.CurrentX = x;
	I2C_OLED.CurrentY = y;
}

char I2C_OLED_Putc(char ch, FontDef_t* Font, I2C_OLED_COLOR_t color) {
	uint32_t i, b, j;

	/* Check available space in LCD */
	if (
		I2C_OLED_WIDTH <= (I2C_OLED.CurrentX + Font->FontWidth) ||
		I2C_OLED_HEIGHT <= (I2C_OLED.CurrentY + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}

	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				I2C_OLED_DrawPixel(I2C_OLED.CurrentX + j, (I2C_OLED.CurrentY + i), (I2C_OLED_COLOR_t) color);
			} else {
				I2C_OLED_DrawPixel(I2C_OLED.CurrentX + j, (I2C_OLED.CurrentY + i), (I2C_OLED_COLOR_t)!color);
			}
		}
	}

	/* Increase pointer */
	I2C_OLED.CurrentX += Font->FontWidth;

	/* Return character written */
	return ch;
}

char I2C_OLED_Puts(char* str, FontDef_t* Font, I2C_OLED_COLOR_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (I2C_OLED_Putc(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}

		/* Increase string pointer */
		str++;
	}

	/* Everything OK, zero should be returned */
	return *str;
}


void I2C_OLED_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, I2C_OLED_COLOR_t c) {
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= I2C_OLED_WIDTH) {
		x0 = I2C_OLED_WIDTH - 1;
	}
	if (x1 >= I2C_OLED_WIDTH) {
		x1 = I2C_OLED_WIDTH - 1;
	}
	if (y0 >= I2C_OLED_HEIGHT) {
		y0 = I2C_OLED_HEIGHT - 1;
	}
	if (y1 >= I2C_OLED_HEIGHT) {
		y1 = I2C_OLED_HEIGHT - 1;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			I2C_OLED_DrawPixel(x0, i, c);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			I2C_OLED_DrawPixel(i, y0, c);
		}

		/* Return from function */
		return;
	}

	while (1) {
		I2C_OLED_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}

void I2C_OLED_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, I2C_OLED_COLOR_t c) {
	/* Check input parameters */
	if (
		x >= I2C_OLED_WIDTH ||
		y >= I2C_OLED_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= I2C_OLED_WIDTH) {
		w = I2C_OLED_WIDTH - x;
	}
	if ((y + h) >= I2C_OLED_HEIGHT) {
		h = I2C_OLED_HEIGHT - y;
	}

	/* Draw 4 lines */
	I2C_OLED_DrawLine(x, y, x + w, y, c);         /* Top line */
	I2C_OLED_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	I2C_OLED_DrawLine(x, y, x, y + h, c);         /* Left line */
	I2C_OLED_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void I2C_OLED_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, I2C_OLED_COLOR_t c) {
	uint8_t i;

	/* Check input parameters */
	if (
		x >= I2C_OLED_WIDTH ||
		y >= I2C_OLED_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= I2C_OLED_WIDTH) {
		w = I2C_OLED_WIDTH - x;
	}
	if ((y + h) >= I2C_OLED_HEIGHT) {
		h = I2C_OLED_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		I2C_OLED_DrawLine(x, y + i, x + w, y + i, c);
	}
}

void I2C_OLED_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, I2C_OLED_COLOR_t color) {
	/* Draw lines */
	I2C_OLED_DrawLine(x1, y1, x2, y2, color);
	I2C_OLED_DrawLine(x2, y2, x3, y3, color);
	I2C_OLED_DrawLine(x3, y3, x1, y1, color);
}


void I2C_OLED_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, I2C_OLED_COLOR_t color) {
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		I2C_OLED_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void I2C_OLED_DrawCircle(int16_t x0, int16_t y0, int16_t r, I2C_OLED_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    I2C_OLED_DrawPixel(x0, y0 + r, c);
    I2C_OLED_DrawPixel(x0, y0 - r, c);
    I2C_OLED_DrawPixel(x0 + r, y0, c);
    I2C_OLED_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        I2C_OLED_DrawPixel(x0 + x, y0 + y, c);
        I2C_OLED_DrawPixel(x0 - x, y0 + y, c);
        I2C_OLED_DrawPixel(x0 + x, y0 - y, c);
        I2C_OLED_DrawPixel(x0 - x, y0 - y, c);

        I2C_OLED_DrawPixel(x0 + y, y0 + x, c);
        I2C_OLED_DrawPixel(x0 - y, y0 + x, c);
        I2C_OLED_DrawPixel(x0 + y, y0 - x, c);
        I2C_OLED_DrawPixel(x0 - y, y0 - x, c);
    }
}

void I2C_OLED_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, I2C_OLED_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    I2C_OLED_DrawPixel(x0, y0 + r, c);
    I2C_OLED_DrawPixel(x0, y0 - r, c);
    I2C_OLED_DrawPixel(x0 + r, y0, c);
    I2C_OLED_DrawPixel(x0 - r, y0, c);
    I2C_OLED_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        I2C_OLED_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        I2C_OLED_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        I2C_OLED_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        I2C_OLED_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}



void I2C_OLED_Clear (void)
{
	I2C_OLED_Fill (0);
    I2C_OLED_UpdateScreen();
}
void I2C_OLED_ON(void) {
	I2C_OLED_WRITECOMMAND(0x8D);
	I2C_OLED_WRITECOMMAND(0x14);
	I2C_OLED_WRITECOMMAND(0xAF);
}
void I2C_OLED_OFF(void) {
	I2C_OLED_WRITECOMMAND(0x8D);
	I2C_OLED_WRITECOMMAND(0x10);
	I2C_OLED_WRITECOMMAND(0xAE);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//  _____ ___   _____
// |_   _|__ \ / ____|
//   | |    ) | |
//   | |   / /| |
//  _| |_ / /_| |____
// |_____|____|\_____|
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void i2c_oled_I2C_Init() {
	//MX_I2C1_Init();
	uint32_t p = 250000;
	while(p>0)
		p--;
	//HAL_I2C_DeInit(&hi2c2);
	//p = 250000;
	//while(p>0)
	//	p--;
	//MX_I2C1_Init();
}

void i2c_oled_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
uint8_t dt[256];
dt[0] = reg;
uint8_t i;
for(i = 0; i < count; i++)
dt[i+1] = data[i];
HAL_I2C_Master_Transmit(&hi2c2, address, dt, count+1, 10);
}


void i2c_oled_I2C_Write(uint8_t address, uint8_t reg, uint8_t data) {
	uint8_t dt[2];
	dt[0] = reg;
	dt[1] = data;
	HAL_I2C_Master_Transmit(&hi2c2, address, dt, 2, 10);
}
