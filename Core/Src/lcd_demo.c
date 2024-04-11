/*
 * lcd_demo.c
 *
 *  Created on: Apr 10, 2024
 *      Author: zhong
 */

#include "lcd_demo.h"
#include "lcd.h"

/**
 * @fn void LCD_Demo(void)
 * @brief A demo function testing the LCD display. Can be used for self-testing.
 *
 */
void
LCD_Demo (void)
{
  uint8_t i = 0;      // ¼ÆÊý±äÁ¿

  LCD_SetTextFont (&ASCII_Font24);     // ÉèÖÃ2424ÖÐÎÄ×ÖÌå,ASCII×ÖÌå¶ÔÓ¦Îª2412
  LCD_SetColor (LCD_BLACK);        // ÉèÖÃ»­±ÊÑÕÉ«

  for (i = 0; i < 8; i++)
    {
      switch (i)
        // ÇÐ»»±³¾°É«
        {
        case 0:
          LCD_SetBackColor (LIGHT_RED);
          break;
        case 1:
          LCD_SetBackColor (LIGHT_GREEN);
          break;
        case 2:
          LCD_SetBackColor (LIGHT_BLUE);
          break;
        case 3:
          LCD_SetBackColor (LIGHT_YELLOW);
          break;
        case 4:
          LCD_SetBackColor (LIGHT_CYAN);
          break;
        case 5:
          LCD_SetBackColor (LIGHT_GREY);
          break;
        case 6:
          LCD_SetBackColor (LIGHT_MAGENTA);
          break;
        case 7:
          LCD_SetBackColor (LCD_WHITE);
          break;
        default:
          break;
        }
      LCD_Clear ();    // ÇåÆÁ
      LCD_DisplayText (13, 70, "STM32F1 Flash Scrn");
      LCD_DisplayText (13, 106, "Resolution: 240*320");
      LCD_DisplayText (13, 142, "Ctrller: ST7789");
      HAL_Delay (1000);  // ÑÓÊ±
    }
}
