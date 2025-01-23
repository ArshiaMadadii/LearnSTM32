



#ifndef __LCD_h__
#define __LCD_h__

void LCD_CMD (unsigned char );
void LCD_DATA (unsigned char );
void LCD_INIT (void);
void LCD_GOTO_XY (unsigned char ,unsigned char );
void LCD_PRINT ( char *);
void LCD_CLEAR (void );
#endif
