#include "stm32f10x.h"

void EXTI0_IRQHandler(void);
int a;
int main(void){
	//Config PC13 OUTPUT
	RCC->APB2ENR |=(1<<4);
	GPIOC->CRH &=0xFF0FFFFF;
	GPIOC->CRH |=0x00300000;
	
	//Config PA0 input
	RCC->APB2ENR |= (1<<2);
	GPIOA->CRL &=0xFFFFFFF0;
	GPIOA->CRL |=0x00000008;
	//Config Interrupt
	RCC->APB2ENR |=(1<<0);
	AFIO->EXTICR[0] &=(0<<0);
	EXTI->IMR |=(1<<0);
	EXTI->RTSR &=(0<<0);
	EXTI->FTSR |=(1<<0);
	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_EnableIRQ(EXTI0_IRQn);
  GPIOC->ODR |=(1<<13);
	while(1){
		if(a==1){
			GPIOC->ODR &=(0<<13);
		}
		else if (a==2){
			GPIOC->ODR |=(1<<13);
		}
	}
}
void EXTI0_IRQHandler(void){
	if(EXTI->PR &(1<<0)){
	a++;
  if(a>2){
		a=1;
	}
	EXTI->PR |=(1<<0);}
}