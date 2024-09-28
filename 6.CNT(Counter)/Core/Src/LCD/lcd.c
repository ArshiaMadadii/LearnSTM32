
#include "lcd.h"

void send_command(unsigned char data)
{
  HAL_Delay(T);
  HAL_GPIO_WritePin(CTRL_PORT,(1<<RS_PIN_NUMBER),GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CTRL_PORT,(1<<RW_PIN_NUMBER),GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT,(0xFF<<D0_PIN_NUMBER),GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT,(data<<D0_PIN_NUMBER),GPIO_PIN_SET);
  HAL_GPIO_WritePin(CTRL_PORT,(1<<E_PIN_NUMBER),GPIO_PIN_SET);
  HAL_GPIO_WritePin(CTRL_PORT,(1<<E_PIN_NUMBER),GPIO_PIN_RESET);
}

void lcd_putchar(unsigned char data)
{
  HAL_Delay(T);
  HAL_GPIO_WritePin(CTRL_PORT,(1<<RS_PIN_NUMBER),GPIO_PIN_SET);
  HAL_GPIO_WritePin(CTRL_PORT,(1<<RW_PIN_NUMBER),GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT,(0xFF<<D0_PIN_NUMBER),GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DATA_PORT,(data<<D0_PIN_NUMBER),GPIO_PIN_SET);
  HAL_GPIO_WritePin(CTRL_PORT,(1<<E_PIN_NUMBER),GPIO_PIN_SET);
  HAL_GPIO_WritePin(CTRL_PORT,(1<<E_PIN_NUMBER),GPIO_PIN_RESET);
}

void lcd_init(void)
{
  HAL_Delay(10*T);
  send_command(0x38);
  send_command(0x06);
  send_command(0x0c);
}

void lcd_puts(char *str)
{
  HAL_Delay(T);
  while(*str != 0)
  {
    lcd_putchar(*str);
    str++;
  }
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
  HAL_Delay(T);
 
    switch(y){
    case 0:
      send_command( 0x80 + x );
    break;
    case 1:
      send_command( 0xC0 + x );
      break;
    case 2:
      send_command( 0x94 + x );
      break;
    case 3:
      send_command( 0xD4 + x );
  }
}

void lcd_clear(void)
{
  HAL_Delay(T);
  send_command(0x01);
  send_command(0x02);
}





