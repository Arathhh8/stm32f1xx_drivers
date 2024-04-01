/*
 * stm32f103xx.h
 *
 *  Created on: Dec 29, 2023
 *      Author: arath
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))
#define NO_CONFIG 7


/*********************************************************************************************************/
/********************************* START: Processor Specific Details *************************************/
/*********************************************************************************************************/

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

#define RESERVED4_APB1_BASEADDR		(APB1PERIPH_BASE + 0x7800)
#define DAC_BASEADDR				(APB1PERIPH_BASE + 0x7400)
#define PWR_BASEADDR				(APB1PERIPH_BASE + 0x7000)
#define BACKUP_REG_BASEADDR			(APB1PERIPH_BASE + 0x6C00)
#define BXCAN2_BASEADDR				(APB1PERIPH_BASE + 0x6800)
#define BXCAN1_BASEADDR				(APB1PERIPH_BASE + 0x6400)
//Shared USB/CAN SRAM 512 bytes		(0x4000 6000(1) - 0x4000 63FF)
#define USB_FS_REG_BASEADDR			(APB1PERIPH_BASE + 0x5C00)
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800)
#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400)
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0x5000)
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0x4C00)
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0x4800)
#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400)
#define RESERVED3_APB1_BASEADDR		(APB1PERIPH_BASE + 0x4000)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00)
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800)
#define RESERVED2_APB1_BASEADDR		(APB1PERIPH_BASE + 0x3400)
#define IWDG_BASEADDR				(APB1PERIPH_BASE + 0x3000)
#define WWDG_BASEADDR				(APB1PERIPH_BASE + 0x2C00)
#define RTC_BASEADDR				(APB1PERIPH_BASE + 0x2800)
#define RESERVED1_APB1_BASEADDR		(APB1PERIPH_BASE + 0x2400)
#define TIM14_BASEADDR				(APB1PERIPH_BASE + 0x2000)
#define TIM13_BASEADDR				(APB1PERIPH_BASE + 0x1C00)
#define TIM12_BASEADDR				(APB1PERIPH_BASE + 0x1800)
#define TIM7_BASEADDR				(APB1PERIPH_BASE + 0x1400)
#define TIM6_BASEADDR				(APB1PERIPH_BASE + 0x1000)
#define TIM5_BASEADDR				(APB1PERIPH_BASE + 0x0C00)
#define TIM4_BASEADDR				(APB1PERIPH_BASE + 0x0800)
#define TIM3_BASEADDR				(APB1PERIPH_BASE + 0x0400)
#define TIM2_BASEADDR				(APB1PERIPH_BASE + 0x0000)


/*
 * Base address of peripherals which are handing on AHB bus
 * TODO : Complete for all other peripherals
*/

//#define RCC_BASEADDR 				(AHBPERIPH_BASE + 0x1000)
#define FSMC_BASEADDR 				(0xA0000000)
#define USBOTG_FS_BASEADDR 			(0x50000000)
#define RESERVED1_BASEADDR 			(0x40030000)
#define ETHERNET_BASEADDR 			(0x40028000)
#define RESERVED2_BASEADDR 			(0x40023400)
#define CRC_BASEADDR 				(0x40023000)
#define FLASH_AHB_BASEADDR 			(0x40022000)
#define RESERVED3_BASEADDR 			(0x40021400)
#define RCC_BASEADDR 				(0x40021000)
#define RESERVED4_BASEADDR 			(0x40020800)
#define DMA2_BASEADDR 				(0x40020400)
#define DMA1_BASEADDR 				(0x40020000)
#define RESERVED5_BASEADDR 			(0x40018400)
#define SDIO_BASEADDR 				(0x40018000)


/*********************************************************************************************************/
/***************************** PERIPHERAL REGIOSTER DEFINITION STRUCTURES ********************************/
/*********************************************************************************************************/

typedef struct{
	__vo uint32_t CRL;					// Port configuration register low			Address offset: 0x00
	__vo uint32_t CRH;					// Port configuration register high			Address offset: 0x04
	__vo uint32_t IDR;					// Port input data register					Address offset: 0x08
	__vo uint32_t ODR;					// Port output data register				Address offset: 0x0C
	__vo uint32_t BSRR;
	__vo uint32_t BRR;
	__vo uint32_t LCKR;
}GPIO_RegDef_t;

/*
 *	Peripheral register definition structure for RCC
 */


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

	__vo uint32_t IMR;					// Interrupt mask register					Address offset: 0x00
	__vo uint32_t EMR;					// Event mask register						Address offset: 0x04
	__vo uint32_t RTSR;					// Rising trigger selection register		Address offset: 0x08
	__vo uint32_t FTSR;					// Falling trigger selection register		Address offset: 0x0C
	__vo uint32_t SWIER;				// Software interrupt event register		Address offset: 0x10
	__vo uint32_t PR;					// Pending register							Address offset: 0x14

}EXTI_RegDef_t;


/*
 *	Peripheral register definition structure for AFIO
 */

typedef struct{

	__vo uint32_t EVCR;					// Event control register 					Address offset: 0x00
	__vo uint32_t MAPR;					// Remap and debug IO configuration reg		Address offset: 0x04
	__vo uint32_t EXTICR[4];			// External interrupt configuration reg		Address offset: 0x08-0x14
	uint32_t RESERVERD;					// Reserved									Address offset: 0x18
	__vo uint32_t MAPR2;				// Remap2 and debug IO configuration reg	Address offset: 0x1C

}AFIO_RegDef_t;


/*
 *	Peripheral register definition structure for SPI
 */

typedef struct{

	__vo uint32_t CR1;					// Control register	1						Address offset: 0x00
	__vo uint32_t CR2;					// Control register 2						Address offset: 0x04
	__vo uint32_t SR;					// Status register							Address offset: 0x08
	__vo uint32_t DR;					// Data register							Address offset: 0x0C
	__vo uint32_t CRCPR;				// CRC polynomial register					Address offset: 0x10
	__vo uint32_t RXCRCR;				// Rx CRC register 							Address offset: 0x14
	__vo uint32_t TXCRCR;				// Tx CRC register							Address offset: 0x18
	__vo uint32_t I2SCFGR;				// I2S configuration register 				Address offset: 0x1C
	__vo uint32_t I2SPR;				// I2S prescaler register					Address offset: 0x20

}SPI_RegDef_t;

/*
 *	Peripheral register definition structure for I2C
 */

typedef struct{

	__vo uint32_t CR1;					// Control register	1						Address offset: 0x00
	__vo uint32_t CR2;					// Control register 2						Address offset: 0x04
	__vo uint32_t OAR1;					// Own address register	1					Address offset: 0x08
	__vo uint32_t OAR2;					// Own address register	1					Address offset: 0x0C
	__vo uint32_t DR;					// Data register							Address offset: 0x10
	__vo uint32_t SR1;					// Status register 1						Address offset: 0x14
	__vo uint32_t SR2;					// Status register 2						Address offset: 0x18
	__vo uint32_t CCR;					// Clock control register	 				Address offset: 0x1C
	__vo uint32_t TRISE;				// TRISE register							Address offset: 0x20

}I2C_RegDef_t;

/*
 *	Peripheral register definition structure for USART
 */

typedef struct{

	__vo uint32_t SR;					// Status register						Address offset: 0x00
	__vo uint32_t DR;					// Data register						Address offset: 0x04
	__vo uint32_t BRR;					// Baud rate register					Address offset: 0x08
	__vo uint32_t CR1;					// Control register 1					Address offset: 0x0C
	__vo uint32_t CR2;					// Control register 2					Address offset: 0x10
	__vo uint32_t CR3;					// Control register 3					Address offset: 0x14
	__vo uint32_t GTPR;					// Guard time and prescaler register	Address offset: 0x18


}USART_RegDef_t;

/*********************************************************************************************************/
/**************** Peripheral definitions (Peripherals base address typecasted to xxx_RegGef_t)************/
/*********************************************************************************************************/

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define AFIO			((AFIO_RegDef_t*)AFIO_BASEADDR)

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)

#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4			((USART_RegDef_t*)UART4_BASEADDR)
#define UART5			((USART_RegDef_t*)UART5_BASEADDR)


/*********************************************************************************************************/
/**************************************** ENABLE PERIPHERALS CLK *****************************************/
/*********************************************************************************************************/

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
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))

/*
 * Clock Enable Macros for UARTx peripherals
 */
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))

/*
 * Clock Enable Macros for generic peripherals
 */
#define AFIO_PCLK_EN()		(RCC->APB2ENR |= (1 << 0))


/*********************************************************************************************************/
/**************************************** DISABLE PERIPHERALS CLK ****************************************/
/*********************************************************************************************************/

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
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 18))

/*
 * Clock Disable Macros for UARTx peripherals
 */
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 2));	(RCC->APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 3));	(RCC->APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 4));	(RCC->APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 5));	(RCC->APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 6));	(RCC->APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 7));	(RCC->APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 8));	(RCC->APB2RSTR &= ~(1 << 2));}while(0)

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									 (x == GPIOB) ? 1 :\
									 (x == GPIOC) ? 2 :\
									 (x == GPIOD) ? 3 :\
									 (x == GPIOE) ? 4 :\
									 (x == GPIOF) ? 5 :\
									 (x == GPIOG) ? 6 :0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)


#define SPI_BASEADDR_TO_CODE(x)	((x == SPI1) ? 0 :\
									 (x == SPI2) ? 1 :\
									 (x == SPI3) ? 2 :0)

/*
 * Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14));}while(0)
#define USART2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));}while(0)

#define USART_BASEADDR_TO_CODE(x)	((x == USART1) ? 0 :\
									 (x == USART2) ? 1 :\
									 (x == USART3) ? 2 :\
									 (x == UART4)  ? 3 :\
									 (x == UART5)  ? 4 :0)

/*********************************************************************************************************/
/********************************** IRQ (INTERRUPT REQUEST) STM32F103xx **********************************/
/*********************************************************************************************************/

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
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

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


// Some generic macros

#define ENABLE 				1
#define DISABLE				0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define HIGH 1
#define LOW  0
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/*********************************************************************************************************/
/*							Bit position definitions of SPI peripheral 									 */
/*********************************************************************************************************/

/*
 * Macros for all bit position in SPI_CR1
 */

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF 		11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN   	13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Macros for all bit position in SPI_CR2
 */

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_RES			3
#define SPI_CR2_RES2		4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_RESERVED	8

/*
 * Macros for all bit position in SPI_SR
 */

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR		    3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_RESERVED		8


/*********************************************************************************************************/
/*							Bit position definitions of I2C peripheral 									 */
/*********************************************************************************************************/

/*
 * Macros for all bit position in I2C_CR1
 */

#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_RES1		2
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_RES2		14
#define I2C_CR1_SWRST		15

/*
 * Macros for all bit position in I2C_CR2
 */

#define I2C_CR2_FREQ		0
#define I2C_CR2_RES1		6
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12
#define I2C_CR2_RES2		13


/*
 * Macros for all bit position in I2C_OAR1
 */

#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD71		1
#define I2C_OAR1_ADD98		8
#define I2C_OAR1_RES		10
#define I2C_OAR1_ADDMODE	15

/*
 * Macros for all bit position in I2C_OAR2
 */

#define I2C_OAR2_ENDUAL		0
#define I2C_OAR2_ADD2_71	1
#define I2C_OAR2_RES1		8

/*
 * Macros for all bit position in I2C_DR
 */

#define I2C_DR_DR70			0
#define I2C_DR_RES			8

/*
 * Macros for all bit position in I2C_SR1
 */

#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RES1		5
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PEC_ERR		12
#define I2C_SR1_RES2		13
#define I2C_SR1_TIME_OUT	14
#define I2C_SR1_SMB_ALERT	15

/*
 * Macros for all bit position in I2C_OAR2
 */

#define I2C_OAR2_ENDUAL		0
#define I2C_OAR2_ADD2_71	1
#define I2C_OAR2_RES1		8

/*
 * Macros for all bit position in I2C_TRISE
 */

#define I2C_TRISE_50		0
#define I2C_TRISE_RES1		6

/*
 * Macros for all bit position in I2C_CCR
 */

#define I2C_CCR_CCR			0
#define I2C_CCR_RES1		1
#define I2C_CCR_DUTY		2
#define I2C_CCR_FS			3



/*********************************************************************************************************/
/*							Bit position definitions of USART peripheral 									 */
/*********************************************************************************************************/


/*
 * Macros for all bit position in USART_SR
 */

#define USART_SR_FLAG_PE		0
#define USART_SR_FLAG_FE		1
#define USART_SR_FLAG_NE		2
#define USART_SR_FLAG_ORE		3
#define USART_SR_FLAG_IDLE		4
#define USART_SR_FLAG_RXNE		5
#define USART_SR_FLAG_TC		6
#define USART_SR_FLAG_TXE		7
#define USART_SR_FLAG_LBD		8
#define USART_SR_FLAG_CTS		9

/*
 * Macros for all bit position in USART_DR
 */

#define USART_DR_DR			0

/*
 * Macros for all bit position in USART_BRR
 */

#define USART_BRR_DIV_FRAC	0
#define USART_BRR_DIV_MANT	4

/*
 * Macros for all bit position in USART_CR1
 */

#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/*
 * Macros for all bit position in USART_CR2
 */

#define USART_CR2_ADD		0
#define USART_CR2_RES1		4
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_RES2		7
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLK_EN	11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

/*
 * Macros for all bit position in USART_CR3
 */

#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10

/*
 * Macros for all bit position in USART_GTPR
 */

#define USART_GTPR_PSC		0
#define USART_GTPR_GT		8


















#include"stm32f103xx_gpio_driver.h"
#include"stm32f1xx_spi_driver.h"
#include"stm32f103xx_i2c_driver.h"
#include<stm32f103xx_usart_driver.h>
#include"stm32f103xx_rcc_driver.h"



#endif /* INC_STM32F103XX_H_ */
