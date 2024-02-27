#include "stm32f10x.h"

void Delay(int rep){
	
	for(;rep>0;rep--){
		int i;
		for(i=0;i<100000;i++){}
	}
}

int main(void){
	//Enable Clock Port C
	RCC->APB2ENR |=(1<<4);
	//Config GPIOC output
	GPIOC->CRH &= 0xFF0FFFFF;
	GPIOC->CRH |=0x00300000;
	
	while(1){
		GPIOC->ODR |= (1<<13);
		Delay(10);
		GPIOC->ODR &=(0<<13);
		Delay(10);
	}
}

