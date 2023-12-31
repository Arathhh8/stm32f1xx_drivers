/*
 * stm32f103xx.h
 *
 *  Created on: Dec 29, 2023
 *      Author: arath
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

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

#define GPIOA_BASEADDR				(APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR				(APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR				(APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR				(APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR				(APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR				(APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR				(APB2PERIPH_BASE + 0x2000)
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x0400)
#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x3800)
#define TIM1_BASEADDR				(APB2PERIPH_BASE + 0x2C00)

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

#define RCC_BASEADDR 				(AHBPERIPH_BASE + 0x1000)
#define CRC_BASEADDR 				(AHBPERIPH_BASE + 0xB000)






#endif /* INC_STM32F103XX_H_ */
