/*
 * gpio.c
 *
 *  Created on: Feb 24, 2024
 *      Author: NMT
 */

#ifndef GPIO_H_
#define GPIO_H_
#include "stdint.h"
#include "stm32f411.h"


#define GPIO_PIN_0      0
#define GPIO_PIN_1      1
#define GPIO_PIN_2      2
#define GPIO_PIN_3      3
#define GPIO_PIN_4      4
#define GPIO_PIN_5      5
#define GPIO_PIN_6      6
#define GPIO_PIN_7      7
#define GPIO_PIN_8      8
#define GPIO_PIN_9      9
#define GPIO_PIN_10     10
#define GPIO_PIN_11     11
#define GPIO_PIN_12     12
#define GPIO_PIN_13     13
#define GPIO_PIN_14     14
#define GPIO_PIN_15     15

#define ENABLE   1
#define DISABLE  0
#define HIGH     1
#define LOW      0

//MODER
#define INPUT                 0
#define OUTPUT                1
#define ALTERNATE FUNCTION    2
#define ANALOG                3

//OTYPER
#define OUTPUT_PP             0
#define OUTPUT_OD             1

//OSPEEDR
#define LOW_SPEED             0
#define MEDIUM_SPEED          1
#define FAST_SPEED            2
#define HIGH_SPEED            3

//PUPDR
#define NO_PU_PD              0
#define PULL_UP               1
#define PULL_DOWN             2
#define RESERVED              3

//AFRL,AFRH
#define AF0                   0
#define AF1                   1
#define AF2                   2
#define AF3                   3
#define AF4                   4
#define AF5                   5
#define AF6                   6
#define AF7                   7
#define AF8                   8
#define AF9                   9
#define AF10                  10
#define AF11                  11
#define AF12                  12
#define AF13                  13
#define AF14                  14
#define AF15                  15


#define GPIOA_CLK_EN()   (RCC->AHB1ENR |= (1<<0))
#define GPIOB_CLK_EN()   (RCC->AHB1ENR |= (1<<1))
#define GPIOC_CLK_EN()   (RCC->AHB1ENR |= (1<<2))

#define GPIOA_CLK_DI()   (RCC->APB2ENR &=~ (0<<0))
#define GPIOB_CLK_DI()   (RCC->APB2ENR &=~ (0<<1))
#define GPIOC_CLK_DI()   (RCC->APB2ENR &=~ (0<<2))


void GPIO_OUTPUT_Init(GPIO_RegDef_t *pGPIOx,uint8_t Pin_number ,uint8_t style,uint8_t speed);
void GPIO_INPUT_Init(GPIO_RegDef_t *pGPIOx,uint8_t Pin_number ,uint8_t configure);
uint8_t GPIO_Read_Pin(GPIO_RegDef_t *pGPIOx,uint8_t Pin_number);
void GPIO_Write_Pin(GPIO_RegDef_t *pGPIOx,uint8_t Pin_number ,uint8_t Pin_value);



#endif /* GPIO_H_ */
