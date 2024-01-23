/*
 * stm32f103xx.h
 *
 *  Created on: Dec 29, 2023
 *      Author: arath
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include<stdint.h>

#define __vo volatile
#define NO_CONFIG 7

/*************************************** START: Processor Specific Details ***************************************/
/*
 * ARM Cortex M3 Processor NVIC ISERx Register Addresses  (Interrupt Set-enable Registers)
*/

#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex M3 Processor NVIC ICERx Register Addresses  (Interrupt Clear-enable Registers)
*/

#define NVIC_ICER0				((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex M3 Processor NVIC IPRx Register Addresses  (Interrupt Priority Registers)
*/

#define NVIC_PR_BASE_ADDR				((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED			4

/*
 * base address of Flash and SRAM memories
*/

#define FLASH_BASEADDR				0x08000000U /* */
#define SRAM_BASEADDR				0x20000000U
#define ROM_BASEADDR				0x1FFFF000U
#define SRAM 						SRAM_BASEADDR

/*
 * AHB and APBx Bus Peripheral base addresses
*/

#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHBPERIPH_BASE				0x40018000U


/*
 * Base address of peripherals which are handing on APB2 bus
 * TODO : Complete for all other peripherals
*/

#define TIM11_BASEADDR				(APB2PERIPH_BASE + 0x5400)
#define TIM10_BASEADDR				(APB2PERIPH_BASE + 0x5000)
#define TIM9_BASEADDR				(APB2PERIPH_BASE + 0x4C00)
#define ADC3_BASEADDR				(APB2PERIPH_BASE + 0x3C00)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x3800)
#define TIM8_BASEADDR				(APB2PERIPH_BASE + 0x3400)
#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000)
#define TIM1_BASEADDR				(APB2PERIPH_BASE + 0x2C00)
#define ADC2_BASEADDR				(APB2PERIPH_BASE + 0x2800)
#define ADC1_BASEADDR				(APB2PERIPH_BASE + 0x2400)
#define GPIOG_BASEADDR				(APB2PERIPH_BASE + 0x2000)
#define GPIOF_BASEADDR				(APB2PERIPH_BASE + 0x1C00)
#define GPIOE_BASEADDR				(APB2PERIPH_BASE + 0x1800)
#define GPIOD_BASEADDR				(APB2PERIPH_BASE + 0x1400)
#define GPIOC_BASEADDR				(APB2PERIPH_BASE + 0x1000)
#define GPIOB_BASEADDR				(APB2PERIPH_BASE + 0x0C00)
#define GPIOA_BASEADDR				(APB2PERIPH_BASE + 0x0800)
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x0400)
#define AFIO_BASEADDR				(APB2PERIPH_BASE + 0x0000)

/*
 * Base address of peripherals which are handing on APB1 bus
 * TODO : Complete for all other peripherals
*/

#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800)
#define SPI2_I2S_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0x5000)
#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0x4800)


/*
 * Base address of peripherals which are handing on AHB bus
 * TODO : Complete for all other peripherals
*/

//#define RCC_BASEADDR 				(AHBPERIPH_BASE + 0x1000)
#define RCC_BASEADDR 				(0x40021000)
#define CRC_BASEADDR 				(AHBPERIPH_BASE + 0xB000)


/***************************** peripheral register definition structures *****************************/

typedef struct{
	__vo uint32_t CRL;					// Port configuration register low			Address offset: 0x00
	__vo uint32_t CRH;					// Port configuration register high			Address offset: 0x04
	__vo uint32_t IDR;					// Port input data register					Address offset: 0x08
	__vo uint32_t ODR;					// Port output data register				Address offset: 0x0C
	__vo uint32_t BSRR;
	__vo uint32_t BRR;
	__vo uint32_t LCKR;
}GPIO_RegDef_t;

//GPIO_RegDef_t *pGPIOA = GPIOA;

typedef struct{
	__vo uint32_t CR;					// Clock control register					Address offset: 0x00
	__vo uint32_t CFGR;					// Clock configuration register				Address offset: 0x04
	__vo uint32_t CIR;					// Clock interrupt register					Address offset: 0x08
	__vo uint32_t APB2RSTR;				// APB2 peripheral reset register			Address offset: 0x0C
	__vo uint32_t APB1RSTR;				// APB1 peripheral reset register			Address offset: 0x10
	__vo uint32_t AHBENR;				// AHB peripheral clock enable register		Address offset: 0x14
	__vo uint32_t APB2ENR;				// APB2 peripheral clock enable register	Address offset: 0x18
	__vo uint32_t APB1ENR;				// APB1 peripheral clock enable register	Address offset: 0x1C
	__vo uint32_t BDCR;					// Backup domain control register			Address offset: 0x20
	__vo uint32_t CSR;					// Control/status register					Address offset: 0x24
}RCC_RegDef_t;

/*
 *	Peripheral register definition structure for EXTI
 */

typedef struct{
	__vo uint32_t IMR;					// 											Address offset: 0x00
	__vo uint32_t EMR;					// 											Address offset: 0x04
	__vo uint32_t RTSR;					// 				TODO						Address offset: 0x08
	__vo uint32_t FTSR;					//											Address offset: 0x0C
	__vo uint32_t SWIER;				// 											Address offset: 0x10
	__vo uint32_t PR;					// 											Address offset: 0x14
}EXTI_RegDef_t;


/*
 *	Peripheral register definition structure for EXTI
 */

typedef struct{
	__vo uint32_t EVCR;					// Address offset: 0x00
	__vo uint32_t MAPR;					// Address offset: 0x04
	__vo uint32_t EXTICR[4];			// Address offset: 0x08-0x14
	uint32_t RESERVERD;					// Reserved: 0x18
	__vo uint32_t MAPR2;				// Address offset: 0x00
}AFIO_RegDef_t;

/*
 *	Peripheral definitions (Peripherals base address typecasted to xxx_RegGef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define AFIO			((AFIO_RegDef_t*)AFIO_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()		(RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()		(RCC->APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN()		(RCC->APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN()		(RCC->APB2ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))


/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))


/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()		(RCC->APB2ENR |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB2ENR |= (1 << 18))

/*
 * Clock Enable Macros for UARTx peripherals
 */
#define UART4_PCLK_EN()		(RCC->APB2ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))

/*
 * Clock Enable Macros for generic peripherals
 */
#define AFIO_PCLK_EN()		(RCC->APB2ENR |= (1 << 0))

/***************************** disable peripherals CLK *****************************/

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 18))

/*
 * Clock Disable Macros for UARTx peripherals
 */
#define UART4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 20))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 2));	(RCC->APB2RSTR |= (1 << 2));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 3));	(RCC->APB2RSTR |= (1 << 2));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 4));	(RCC->APB2RSTR |= (1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 5));	(RCC->APB2RSTR |= (1 << 2));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 6));	(RCC->APB2RSTR |= (1 << 2));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 7));	(RCC->APB2RSTR |= (1 << 2));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 8));	(RCC->APB2RSTR |= (1 << 2));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
									 (x == GPIOD) ? 3 :\
									 (x == GPIOE) ? 4 :\
									 (x == GPIOF) ? 5 :\
									 (x == GPIOG) ? 6 :0)

/*
 * IRQ(Interrupt Request)
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * Macros for all the possible priority levels IRQ
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


// some generic macros

#define ENABLE 				1
#define DISABLE				0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define HIGH 1
#define LOW  0






#include"stm32f103xx_gpio_driver.h"



#endif /* INC_STM32F103XX_H_ */
