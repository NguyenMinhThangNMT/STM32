/*
 * gpio.c
 *
 *  Created on: Feb 24, 2024
 *      Author: NMT
 */


#include "gpio.h"
void GPIO_OUTPUT_Init(GPIO_RegDef_t *pGPIOx,uint8_t Pin_number ,uint8_t style,uint8_t speed){
	if(pGPIOx==GPIOA){
		GPIOA_CLK_EN();
	}
	else if(pGPIOx==GPIOB){
	    GPIOB_CLK_EN();
	}
	else if(pGPIOx==GPIOC){
		GPIOC_CLK_EN();
	}
	pGPIOx->MODER |= (1<<Pin_number*2);
	pGPIOx->OTYPER |=(style<<Pin_number);
	pGPIOx->OSPEEDR |=(speed<<Pin_number*2);
}

void GPIO_INPUT_Init(GPIO_RegDef_t *pGPIOx,uint8_t Pin_number ,uint8_t configure){
	if(pGPIOx==GPIOA){
		GPIOA_CLK_EN();
    }
	else if(pGPIOx==GPIOB){
		GPIOB_CLK_EN();
	}
	else if(pGPIOx==GPIOC){
		GPIOC_CLK_EN();
	}
	pGPIOx->MODER |= (0<<Pin_number);
	pGPIOx->PUPDR |= (configure<<Pin_number*2);
}

uint8_t GPIO_Read_Pin(GPIO_RegDef_t *pGPIOx,uint8_t Pin_number)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> Pin_number) & 0x01);

	return  value;
}
void GPIO_Write_Pin(GPIO_RegDef_t *pGPIOx,uint8_t Pin_number ,uint8_t Pin_value){
	if (Pin_value==1){
		pGPIOx->ODR |=(1<<Pin_number);
	}
	else pGPIOx->ODR &=~ (1<<Pin_number);
}

