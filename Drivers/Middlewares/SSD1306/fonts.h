/**
 * fonts.c / fonts.h
 *
 * Author      : dmxddrx
 * Created     : 2025
 *
 * Description :
 *   Bitmap font library for STM32 TFT displays.
 *   Contains Font_5x7, Font_6x8, Font_7x10, Font_11x18, Font_16x26.
 *
 * Font data origin :
 *   These fonts were built and refined incrementally by dmxddrx
 *   over time, with the assistance of AI tools
 *   Original column-major source data was transposed to row-major
 *   uint16_t format to match the TFT_DrawChar rendering engine.
 *   Any resemblance to prior open-source font tables is incidental —
 *   every glyph has been reviewed, corrected, or redrawn by the author.
 *
 * Usage :
 *   Include fonts.h and pass a FontDef_t pointer to TFT_PrintFont().
 *   Data format: each uint16_t = one row of pixels, MSB = leftmost pixel.
 *
 * No license is attached. Credit appreciated but not required.
 * Do whatever you want with this — just don't claim it was yours alone.
 */
#ifndef FONTS_H
#define FONTS_H 120

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

/**
 *
 * Default fonts library. It is used in all LCD based libraries.
 *
 * \par Supported fonts
 *
 * Currently, these fonts are supported:
 *  - 7 x 10 pixels
 *  - 11 x 18 pixels
 *  - 16 x 26 pixels
 */
#include "stm32f4xx_hal.h"
#include "string.h"

/**
 * @defgroup LIB_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Font structure used on my LCD libraries
 */

typedef struct {
	uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef_t;

/**
 * @brief  String length and height
 */
typedef struct {
	uint16_t Length;      /*!< String length in units of pixels */
	uint16_t Height;      /*!< String height in units of pixels */
} FONTS_SIZE_t;

/**
 * @}
 */

/**
 * @defgroup FONTS_FontVariables
 * @brief    Library font variables
 * @{
 */

// 5 x 7 pixels font size structure
extern FontDef_t Font_5x7;

// 5 x 7 pixels font size structure
extern FontDef_t Font_6x8;

// 7 x 10 pixels font size structure
extern FontDef_t Font_7x10;

// 11 x 18 pixels font size structure
extern FontDef_t Font_11x18;

// 16 x 26 pixels font size structure
extern FontDef_t Font_16x26;

/**
 * @}
 */

/**
 * @defgroup FONTS_Functions
 * @brief    Library functions
 * @{
 */

/**
 * @brief  Calculates string length and height in units of pixels depending on string and font used
 * @param  *str: String to be checked for length and height
 * @param  *SizeStruct: Pointer to empty @ref FONTS_SIZE_t structure where informations will be saved
 * @param  *Font: Pointer to @ref FontDef_t font used for calculations
 * @retval Pointer to string used for length and height
 */
char* FONTS_GetStringSize(char* str, FONTS_SIZE_t* SizeStruct, FontDef_t* Font);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif


#endif
