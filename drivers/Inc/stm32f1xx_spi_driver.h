/*
 * stm32f1xx_spi_driver.h
 *
 *  Created on: Jan 26, 2024
 *      Author: arath
 */

#ifndef INC_STM32F1XX_SPI_DRIVER_H_
#define INC_STM32F1XX_SPI_DRIVER_H_

#include"stm32f103xx.h"


 /*
  * Configuration structure for SPIx peripheral
  */

typedef struct {

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */

typedef struct {

	SPI_Config_t *pSPIx;			/*!< This holds the base address of the SPIx(X:0,1,2) peripheral >*/
	SPI_Config_t SPIConfig;			/*!< This holds SPI pin configuration settings >*/
}SPI_Handle_t;




/**************************************************************************************************************
 *								APIs supported by this driver
 *				For more information about the APIs check the function definitions
 *********************************************************************************************************** */


/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t  EnorDi);

/*
 * Init and De-Init
 */
void SPI_Init(GPIO_Handle_t *pGPIOHandle);
void SPI_DeInit(GPIO_Handle_t *pGPIOx);

/*
 * Data send and data receive
 */


/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(uint8_t PinNumber);





#endif /* INC_STM32F1XX_SPI_DRIVER_H_ */
