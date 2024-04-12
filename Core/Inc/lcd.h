/**
 * @file lcd.h
 * @author Tiantian Zhong
 * @brief ST7789 LCD Driver header file.
 * @version 0.1
 * @date 2024-04-12
 *
 */

#ifndef __LCD_H
#define __LCD_H

#include "main.h"
#include "spi.h"
#include "usart.h"
#include "lcd_fonts.h"

/* LCD Screen Parameters */

#define LCD_Width 240  // LCD Width
#define LCD_Height 320 // LCD Height

// LCD Display Direction
#define Direction_H 0      // Horizontal
#define Direction_H_Flip 1 // Horizontal Flip
#define Direction_V 2      // Vertical
#define Direction_V_Flip 3 // Vertical Flip

/** Display Numbers: Fill zeros or spaces at extra digits
 * Used for functions LCD_DisplayNumbers() and LCD_DisplayDecimals()
 * Usage: LCD_ShowNumMode(Fill_Zero), shows 123 as 000123. */
#define Fill_Zero 0
#define Fill_Space 1

/* Commonly Used Colors */
#define LCD_WHITE 0xFFFFFF
#define LCD_BLACK 0x000000
#define LCD_BLUE 0x0000FF
#define LCD_GREEN 0x00FF00
#define LCD_RED 0xFF0000
#define LCD_CYAN 0x00FFFF
#define LCD_MAGENTA 0xFF00FF
#define LCD_YELLOW 0xFFFF00
#define LCD_GREY 0x2C2C2C
#define LIGHT_BLUE 0x8080FF
#define LIGHT_GREEN 0x80FF80
#define LIGHT_RED 0xFF8080
#define LIGHT_CYAN 0x80FFFF
#define LIGHT_MAGENTA 0xFF80FF
#define LIGHT_YELLOW 0xFFFF80
#define LIGHT_GREY 0xA3A3A3
#define DARK_BLUE 0x000080
#define DARK_GREEN 0x008000
#define DARK_RED 0x800000
#define DARK_CYAN 0x008080
#define DARK_MAGENTA 0x800080
#define DARK_YELLOW 0x808000
#define DARK_GREY 0x404040

/* LCD Commands */
void SPI_LCD_Init (void);
void LCD_Clear (void);
void LCD_ClearRect (uint16_t x, uint16_t y, uint16_t width, uint16_t height);
void LCD_SetAddress (uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_SetColor (uint32_t Color);
void LCD_SetBackColor (uint32_t Color);
void LCD_SetDirection (uint8_t direction);

void LCD_SetAsciiFont (pFONT *fonts);
void LCD_DisplayChar (uint16_t x, uint16_t y, uint8_t c);
void LCD_DisplayString (uint16_t x, uint16_t y, char *p);

void LCD_SetTextFont (pFONT *fonts);
void LCD_DisplayChinese (uint16_t x, uint16_t y, char *pText);
void LCD_DisplayText (uint16_t x, uint16_t y, char *pText);

//>>>>>	?????????¨°????
void LCD_ShowNumMode (uint8_t mode);
void LCD_DisplayNumber (uint16_t x, uint16_t y, int32_t number, uint8_t len);
void LCD_DisplayDecimals (uint16_t x, uint16_t y, double number, uint8_t len,
                          uint8_t decs);

//>>>>>	2D????????
void LCD_DrawPoint (uint16_t x, uint16_t y, uint32_t color);

void LCD_DrawLine_V (uint16_t x, uint16_t y, uint16_t height);
void LCD_DrawLine_H (uint16_t x, uint16_t y, uint16_t width);
void LCD_DrawLine (uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void LCD_DrawRect (uint16_t x, uint16_t y, uint16_t width,
                   uint16_t height);
void LCD_DrawCircle (uint16_t x, uint16_t y, uint16_t r);
void LCD_DrawEllipse (int x, int y, int r1, int r2);

//>>>>>	???¨°????????
void LCD_FillRect (uint16_t x, uint16_t y, uint16_t width,
                   uint16_t height);
void LCD_FillCircle (uint16_t x, uint16_t y, uint16_t r);

//>>>>>	????????????
void LCD_DrawImage (uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                    const uint8_t *pImage);

/* Pins Macros */
#define 	LCD_SCK_PIN      		 GPIO_PIN_3									// SCK?????? ?¨¨?????¡§??SPI3??IO??????
#define 	LCD_SCK_PORT     		 GPIOB                 					// SCK??????????????  
#define 	GPIO_LCD_SCK_CLK      __HAL_RCC_GPIOB_CLK_ENABLE()  		// SCK????IO???¡À??

#define 	LCD_SDA_PIN      		 GPIO_PIN_5									// SDA?????? ?¨¨?????¡§??SPI3??IO??????
#define 	LCD_SDA_PORT    		 GPIOB                 					// SDA??????????????  
#define 	GPIO_LCD_SDA_CLK      __HAL_RCC_GPIOB_CLK_ENABLE()  		// SDA????IO???¡À??


#define 	LCD_CS_PIN       				GPIO_PIN_11							// CS???????????????????¡ì
#define 	LCD_CS_PORT      				GPIOD                 			// CS?????????????? 
#define 	GPIO_LCD_CS_CLK     			__HAL_RCC_GPIOD_CLK_ENABLE()	// CS????IO???¡À??

#define  LCD_DC_PIN						GPIO_PIN_12				         // ????????????  ????				
#define	LCD_DC_PORT						GPIOD									// ????????????  GPIO????
#define 	GPIO_LCD_DC_CLK     			__HAL_RCC_GPIOD_CLK_ENABLE()	// ????????????  GPIO?¡À?? 	

#define  LCD_Backlight_PIN				GPIO_PIN_13				         // ¡À???  ????				
#define	LCD_Backlight_PORT			GPIOD									// ¡À??? GPIO????
#define 	GPIO_LCD_Backlight_CLK     __HAL_RCC_GPIOD_CLK_ENABLE()	// ¡À??? GPIO?¡À?? 	

/* Control Macros */
// ?¨°???????????¨¨????¡¤¡À??¡Á¡Â???????????¡Â?¡ì???¨¢????
#define 	LCD_CS_H    		 	LCD_CS_PORT->BSRR = LCD_CS_PIN		// ????????
#define 	LCD_CS_L     			LCD_CS_PORT->BRR  = LCD_CS_PIN		// ????????

#define	LCD_DC_Command		   HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET)	  		// ???????????????? 
#define 	LCD_DC_Data		      HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET)				// ????????????????

#define 	LCD_Backlight_ON      HAL_GPIO_WritePin(LCD_Backlight_PORT, LCD_Backlight_PIN, GPIO_PIN_SET)		// ????????????¡À???
#define 	LCD_Backlight_OFF  	 HAL_GPIO_WritePin(LCD_Backlight_PORT, LCD_Backlight_PIN, GPIO_PIN_RESET)	// ??????????¡À?¡À???


#endif // __LCD_H