#include "stm32f10x.h"
#include "LCD.h"
#include "DELAY.h"
#include <stdio.h>

volatile int counter =0;
volatile int frequency=0;
int duty = 15;

//volatile int off=0;



void SystemClock_Config(void);
void GPIO_Init(void);

int main (void)
{
char x[20];
SystemClock_Config();	
GPIO_Init();
	
delay_intial();	

	
	
	//TIMER3 FOR GENERATE SECOND
	//TIMER4 FOR COUNT EDGE
	//TIMER2 FOR GENERATE WAVEFORM
	
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;	
RCC->APB1ENR |=RCC_APB1ENR_TIM3EN;
RCC->APB1ENR |=RCC_APB1ENR_TIM4EN;
	
TIM2->PSC = 11;
TIM2->ARR = duty;
TIM2->CCR1 = duty;
	
TIM2->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1;	

TIM2->CCER |= TIM_CCER_CC1E; 
TIM2->CCER &=~ TIM_CCER_CC1P;

	

	
	
TIM4->PSC=39;
TIM4->ARR = 65535;
TIM4->CCMR1 |= TIM_CCMR1_CC1S_0; //INPUT SELECT 
TIM4->CCMR1 &=~ TIM_CCMR1_CC1S_1; //INPUT SELECT 
TIM4->CCER |= TIM_CCER_CC1E;
TIM4->DIER |= TIM_DIER_CC1IE;

TIM4->CCMR1 &=~ (TIM_CCMR1_IC1PSC_0 );
TIM4->CCMR1 &=~ TIM_CCMR1_IC1PSC_1;

TIM4->CCER &=~ TIM_CCER_CC1P;

	
TIM3->PSC = 19999;     // set for 5s
TIM3->ARR = 18000; 


TIM3->DIER |= TIM_DIER_UIE;
NVIC_SetPriorityGrouping(0);

NVIC_SetPriority(TIM3_IRQn,3);
NVIC_SetPriority(TIM4_IRQn,4);

NVIC_EnableIRQ(TIM3_IRQn);
NVIC_EnableIRQ(TIM4_IRQn);

TIM4->CNT=0;
TIM3->CNT=0;
TIM2->CNT=0;

TIM3->CR1 |= TIM_CR1_CEN;
TIM4->CR1 |= TIM_CR1_CEN;
TIM2->CR1 |=TIM_CR1_CEN;	


LCD_INIT ();
LCD_CLEAR();
LCD_GOTO_XY(1,1);
LCD_PRINT("Amirhosein.B");
LCD_GOTO_XY(1,2);
LCD_PRINT("Microprocessor");
delayMs(2000);
LCD_CLEAR();


while(1)
{
	
	
	sprintf (x , "Frequency Is: ");	
	LCD_GOTO_XY(1,1);
	LCD_PRINT(x);	
	sprintf (x , "%d Hz ",frequency);
	LCD_GOTO_XY(1,2);
	LCD_PRINT(x);	
	
		if((GPIOB->IDR & GPIO_IDR_IDR5)== 0 )
		{
		while(!(GPIOB->IDR & GPIO_IDR_IDR5));
		duty = duty + 1;	
		if (duty > 65534)duty = 65534;
		TIM2->ARR = duty;
		TIM2->CCR1 = duty;
		}

		if((GPIOB->IDR & GPIO_IDR_IDR4)== 0 )
		{
		while(!(GPIOB->IDR & GPIO_IDR_IDR4));
		duty = duty - 1;	
		if (duty < 1)duty = 1;
		TIM2->ARR = duty;
		TIM2->CCR1 = duty;  			
		}

	
}
}

void TIM3_IRQHandler (void)
{
	if(TIM3->SR & TIM_SR_UIF)
	{	
	frequency=counter/10;	
	counter=0;
	TIM3->SR &=~TIM_SR_UIF;
	}
}

void TIM4_IRQHandler (void)
{
	counter++;
	TIM4->SR &=~ TIM_SR_CC1IF;	
}


void SystemClock_Config(void)
{
// ******************* System clock = 72MHz ******************//

FLASH->ACR |=0X2U;

RCC->CR |= RCC_CR_HSEON;         // HSE ON 
while(!(RCC->CR & RCC_CR_HSERDY));

RCC->CFGR  &=~ RCC_CFGR_PLLXTPRE;
	
RCC->CFGR  |= RCC_CFGR_PLLSRC;	
	
RCC->CFGR |=RCC_CFGR_PLLMULL9;//PLL * 9
	
RCC->CR |= RCC_CR_PLLON;      // PLL ON 
while(!(RCC->CR & RCC_CR_PLLRDY));

RCC->CFGR |=RCC_CFGR_PPRE1_2;

RCC->CFGR |=RCC_CFGR_SW_PLL;
while((RCC->CFGR & RCC_CFGR_SWS_PLL)!=RCC_CFGR_SWS_PLL);
	
}

void GPIO_Init(void)
{
RCC->APB2ENR |=RCC_APB2ENR_IOPAEN;
RCC->APB2ENR |=RCC_APB2ENR_IOPBEN;
RCC->APB2ENR |=RCC_APB2ENR_AFIOEN;
	

	
//*********** CONFIG GPIOA FOR DPRT LCD ***********//
GPIOA->CRL |= GPIO_CRL_MODE4_0|GPIO_CRL_MODE4_1;  
GPIOA->CRL &=~  GPIO_CRL_CNF4_0|GPIO_CRL_CNF4_1;  

GPIOA->CRL |=  GPIO_CRL_MODE5_0|GPIO_CRL_MODE5_1; 
GPIOA->CRL &=~   GPIO_CRL_CNF5_0|GPIO_CRL_CNF5_1; 
	
GPIOA->CRL |= GPIO_CRL_MODE6_0|GPIO_CRL_MODE6_1; 
GPIOA->CRL &=~  GPIO_CRL_CNF6_0|GPIO_CRL_CNF6_1; 

GPIOA->CRL |= GPIO_CRL_MODE7_0|GPIO_CRL_MODE7_1; 
GPIOA->CRL &=~  GPIO_CRL_CNF7_0|GPIO_CRL_CNF7_1; 	
//********** CONFIG GPIOB FOR CPRT LCD ************//
GPIOB->CRH |= GPIO_CRH_MODE10_0|GPIO_CRH_MODE10_1; 
GPIOB->CRH &=~  GPIO_CRH_CNF10_0|GPIO_CRH_CNF10_1; 

GPIOB->CRH |= GPIO_CRH_MODE11_0|GPIO_CRH_MODE11_1; 
GPIOB->CRH &=~  GPIO_CRH_CNF11_0|GPIO_CRH_CNF11_1; 
//*************************************************//

//*************************************************//
////**** CONFIG PINB.6 FOR Inputcapture, CH 1 ******//
GPIOB->CRL &=~  GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1; 
GPIOB->CRL |=   GPIO_CRL_CNF6_0; 
GPIOB->CRL &=~  GPIO_CRL_CNF6_1; 

/////////////////////////
GPIOB->CRL &=~ GPIO_CRL_MODE5_0|GPIO_CRL_MODE5_1; 
GPIOB->CRL &=~ GPIO_CRL_CNF5_0; 
GPIOB->CRL |=  GPIO_CRL_CNF5_1;
	
GPIOB->CRL &=~ GPIO_CRL_MODE4_0|GPIO_CRL_MODE4_1; 
GPIOB->CRL &=~ GPIO_CRL_CNF4_0; 
GPIOB->CRL |=  GPIO_CRL_CNF4_1;

GPIOB->ODR |= GPIO_ODR_ODR4;
GPIOB->ODR |= GPIO_ODR_ODR5;

//**** CONFIG PINA.0 FOR Output PWM, CH 1 ******//
GPIOA->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1; 
GPIOA->CRL &=~  GPIO_CRL_CNF0_0; 
GPIOA->CRL |=   GPIO_CRL_CNF0_1;



}
