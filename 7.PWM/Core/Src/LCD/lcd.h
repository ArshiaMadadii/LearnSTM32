
#ifndef _LCD_H
#define _LCD_H

#include "stm32f1xx_hal.h"

#define T  1  

#define D0_PIN_NUMBER  0

#define DATA_PORT  GPIOA

#define CTRL_PORT  GPIOB

#define RS_PIN_NUMBER  5
#define RW_PIN_NUMBER  6
#define E_PIN_NUMBER   7


void lcd_init(void);
void lcd_clear(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_puts(char *str);

#endif
