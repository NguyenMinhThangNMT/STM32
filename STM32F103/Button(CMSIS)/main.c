#include "stm32f10x.h"

int main(void){
	//Enable clock Port A and Port C;
	RCC->APB2ENR |=(1<<4);
	RCC->APB2ENR |=(1<<2);
	//Config PC13 output
	GPIOC->CRH &= 0xFF0FFFFF;
	GPIOC->CRH |= 0x00300000;
	//Config PA0 input
	GPIOA->CRL &= 0xFFFFFFF0;
	GPIOA->CRL |= 0x00000008;
	GPIOA->ODR |=(1<<0);
	
	GPIOA->CRL &= 0xFFFFFF0F;
	GPIOA->CRL |= 0x00000080;
	GPIOA->ODR |=(1<<1);
	while(1){
	
	
	}
}
	
	