

#include <stm32f10x.h>
#include "LCD.h"
#include "delay.h"



#define LCD_DPRT GPIOA
#define D4 GPIO_ODR_ODR4
#define D5 GPIO_ODR_ODR5
#define D6 GPIO_ODR_ODR6
#define D7 GPIO_ODR_ODR7

#define LCD_CPRT GPIOB
#define RS GPIO_ODR_ODR10
#define E  GPIO_ODR_ODR11


void LCD_CMD (unsigned char cmd)
{
//	delayMs(1);
	GPIOA->ODR  = (((cmd >> 4)& 0xf) << 4);	
	LCD_CPRT->ODR &=~(RS);	// for command
	LCD_CPRT->ODR |= (E);   // h - to - low
//	delay_us(1);
	delayMs(1);
	LCD_CPRT->ODR &=~(E);	// h - to - low
//	delay_us(100);
	delayMs(1);
	GPIOA->ODR  = (((cmd & 0xF)<< 4));	
	LCD_CPRT->ODR |= (E);   // h - to - low
//	delay_us(1);
	delayMs(1);
	LCD_CPRT->ODR &=~ (E);	// h - to - low
  delayMs(1);
}

void LCD_DATA (unsigned char data)
{
	
//	delay_us(1000);
	GPIOA->ODR  = (((data >> 4)& 0xf) << 4);	
	LCD_CPRT->ODR |= (RS);	// for command
	LCD_CPRT->ODR |= (E);   // h - to - low
//	delay_us(1);
	delayMs(1);
	LCD_CPRT->ODR &=~ (E);	// h - to - low	
	delayMs(1);
	GPIOA->ODR  = (((data & 0xF)<< 4));
	LCD_CPRT->ODR |= (E);   // h - to - low	
//	delay_us(1);
	delayMs(1);
	LCD_CPRT->ODR &=~ (E);	// h - to - low
  delayMs(1);
}

void LCD_INIT (void)
{

	LCD_CPRT->ODR &=~ (E); 
	delayMs(1);	
	LCD_CMD(0X33);
	LCD_CMD(0X32);
	LCD_CMD(0X28);
	LCD_CMD(0X0c);
	LCD_CMD(0X06);
	LCD_CMD(0X01);	
	LCD_CMD(0X02);  
}
void LCD_CLEAR(void)
{
LCD_CMD(0X01);	
}

void LCD_GOTO_XY (unsigned char x,unsigned char y)
{
	unsigned char firstCharADR[]={0X80,0XC0,0X94,0XD4};
	LCD_CMD(firstCharADR[y-1]+x-1);
	delayMs(1);
}

void LCD_PRINT ( char *str )
{
	unsigned char i=0;
	while(str[i]!=0)
	{
		LCD_DATA(str[i]);
		i++;
	}
}

