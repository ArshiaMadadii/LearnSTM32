#include "stm32f10x.h"

void SystemClock_Config(void);

int main(void)
{
  // Enable clock for A,B,C Ports
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  // Enable AFIO
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  
  GPIOA->CRH = 0;
  GPIOB->CRL = 0;
  
  GPIOA->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1; // PIN A.7 as Output
  GPIOA->CRL &= ~(GPIO_CRL_CNF7_0 | GPIO_CRL_CNF7_1); // PIN A.7 as Push-pull
  
  GPIOA->CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1; // PIN A.9 as Output
  GPIOA->CRH &= ~(GPIO_CRH_CNF9_0 | GPIO_CRH_CNF9_1); // Pin A.9 as Push-pull
  
  GPIOA->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; // PIN A.10 as Output
  GPIOA->CRH &= ~(GPIO_CRH_CNF10_0 | GPIO_CRH_CNF10_1); // Pin A.10 as Push-pull
  
  GPIOB->CRL &= ~(GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1); // Set pin B.0 as input
  GPIOB->CRL &= ~GPIO_CRL_CNF0_0;
  GPIOB->CRL |= GPIO_CRL_CNF0_1;
  
  GPIOA->CRL &= ~(GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1); // Set pin A.3 as input
  GPIOA->CRL &= ~GPIO_CRL_CNF3_0;
  GPIOA->CRL |= GPIO_CRL_CNF3_1;
  
  GPIOA->CRL &= ~(GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1); // Set pin B.4 as input
  GPIOA->CRL &= ~GPIO_CRL_CNF4_0;
  GPIOA->CRL |= GPIO_CRL_CNF4_1;
  
  GPIOA->CRL &= ~(GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1); // Set pin B.5 as input
  GPIOA->CRL &= ~GPIO_CRL_CNF5_0;
  GPIOA->CRL |= GPIO_CRL_CNF5_1;
  
  GPIOB->ODR |= GPIO_ODR_ODR0;
  GPIOA->ODR |= GPIO_ODR_ODR3|GPIO_ODR_ODR4 | GPIO_ODR_ODR5;
  EXTI->IMR |= EXTI_IMR_MR3 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR14;
  
  EXTI->SWIER |= EXTI_SWIER_SWIER14;
  
  EXTI->FTSR |= EXTI_FTSR_TR3 | EXTI_FTSR_TR4 | EXTI_FTSR_TR5;
  AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PA;
  AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PA | AFIO_EXTICR2_EXTI5_PA;
  
  NVIC_SetPriorityGrouping(0);
  
  NVIC_SetPriority(EXTI3_IRQn, 3);
  NVIC_EnableIRQ(EXTI3_IRQn);
  
  NVIC_SetPriority(EXTI4_IRQn, 4);
  NVIC_EnableIRQ(EXTI4_IRQn);
  
  NVIC_SetPriority(EXTI9_5_IRQn, 5);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  NVIC_SetPriority(EXTI15_10_IRQn, 6);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  
  while (1)
  {
    if ((GPIOB->IDR & GPIO_IDR_IDR0) == 0) {
      EXTI->SWIER |= EXTI_SWIER_SWIER14;
    }
  }
}

void EXTI3_IRQHandler(void) {
  GPIOA->ODR |= GPIO_ODR_ODR7;
  EXTI->PR |= EXTI_PR_PR3;
}

void EXTI4_IRQHandler(void) {
  GPIOA->ODR |= GPIO_ODR_ODR9;
  EXTI->PR |= EXTI_PR_PR4;
}

void EXTI9_5_IRQHandler(void) {
  GPIOA->ODR |= GPIO_ODR_ODR10;
  EXTI->PR |= EXTI_PR_PR5;
}

void EXTI15_10_IRQHandler(void) {
  GPIOA->ODR &= ~(GPIO_ODR_ODR7 | GPIO_ODR_ODR9 | GPIO_ODR_ODR10);
  EXTI->PR |= EXTI_PR_PR14;
}

void SystemClock_Config(void) {
}
