#include <stdio.h>
char name[20]="arshia";

void puts_char (char*p)
{
 while(*p!=0){
    
    putchar(*p);
    *p++;
 }
 
 }

void main(void){
    puts_char(name);
}