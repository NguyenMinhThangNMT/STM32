#ifndef INC_STM32F411_H_
#define INC_STM32F411_H_

#include "stdint.h"


#define FLASH_BASE            0x08000000UL /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define SRAM1_BASE            0x20000000UL /*!< SRAM1(128 KB) base address in the alias region                             */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
#define SRAM1_BB_BASE         0x22000000UL /*!< SRAM1(128 KB) base address in the bit-band region                          */
#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       0x42480000UL /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             0x0807FFFFUL /*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        0x1FFF7800UL /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         0x1FFF7A0FUL /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE


#define AHB1_BASE_ADDR        0x40020000
#define AHB2_BASE_ADDR        0x50000000
#define APB2_BASE_ADDR        0x40010000
#define APB1_BASE_ADDR        0x40000000

#define USB_OTG_BASE_ADDR     0x50000000

#define DMA2_BASE_ADDR        0x40026400
#define DMA1_BASE_ADDR        0x40026000
#define FLASH_BASE_ADDR       0x40023C00
#define RCC_BASE_ADDR         0x40023800
#define CRC_BASE_ADDR         0x40023000
#define GPIOA_BASE_ADDR       0x40020000
#define GPIOB_BASE_ADDR       0x40020400
#define GPIOC_BASE_ADDR       0x40020800


#define TIM1_BASE_ADDR        0x40010000
#define USART1_BASE_ADDR      0x40011000
#define USART6_BASE_ADDR      0x40011400
#define ADC1_BASE_ADDR        0x40012000
#define SPI1_BASE_ADDR        0x40013000
#define SPI4_BASE_ADDR        0x40013400
#define SYSCSFG_BASE_ADDR     0x40013800
#define EXTI_BASE_ADDR        0x40013C00
#define TIM9_BASE_ADDR        0x40014000
#define TIM10_BASE_ADDR       0x40014400
#define TIM11_BASE_ADDR       0x40014800
#define SPI5_BASE_ADDR        0x40015000


#define TIM2_BASE_ADDR         0x40000000
#define TIM3_BASE_ADDR         0x40000400
#define TIM4_BASE_ADDR         0x40000800
#define TIM5_BASE_ADDR         0x40000C00
#define RTC_BASE_ADDR         0x40002800
#define SPI2_BASE_ADDR         0x40003800
#define SPI3_BASE_ADDR         0x40003C00
#define USART2_BASE_ADDR       0x40004400
#define I2C1_BASE_ADDR         0x40005400
#define I2C2_BASE_ADDR         0x40005800
#define I2C3_BASE_ADDR         0x40005C00
#define PWR_BASE_ADDR          0x40007000

typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO_RegDef_t;

typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	uint32_t ResetValue1[2];
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t ResetValue2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	uint32_t ResetValue3[2];
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t ResetValue4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	uint32_t ResetValue5[2];
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t ResetValue6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t ResetValue7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCGR;
	uint32_t ResetValue8;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;

typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}Pin_config_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	Pin_config_t GPIO_PinConfig;
}GPIO_Handle_t;





#define ENABLE   1
#define DISABLE  0
#define HIGH     1
#define LOW      0

#define GPIOA   ((GPIO_RegDef_t *)GPIOA_BASE_ADDR)
#define GPIOB   ((GPIO_RegDef_t *)GPIOB_BASE_ADDR)
#define GPIOC   ((GPIO_RegDef_t *)GPIOC_BASE_ADDR)

#define SPI1  ((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2  ((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3  ((SPI_RegDef_t*)SPI3_BASE_ADDR)

#define RCC   ((RCC_RegDef_t *)RCC_BASE_ADDR)

#endif
