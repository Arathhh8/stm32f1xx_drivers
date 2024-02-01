/*
 * stm32f1xx_spi_driver.c
 *
 *  Created on: Jan 26, 2024
 *      Author: arath
 */

#include"stm32f1xx_spi_driver.h"

/*
 * Peripheral Clock setup
 */

/**************************************************************************************************
 * @fn 					- SPI_PeriClockControl
 *
 * @brief				- This function enable or disables peripheral clock for the given SPI
 *
 * @param[in]			- base address of the SPI peripheral
 * @param[in]			- ENABLE or DISABLE macros
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t  EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}

/**************************************************************************************************
 * @fn 					- SPI_Init
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note				-
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){

	// Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// First lets configure the SPI_CR1 register
	uint32_t tempreg = 0;  // temporal register

	// 1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;
	pSPIHandle->pSPIx->CR1 |= tempreg;

	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDIMODE should be cleared
		tempreg &= ~(1 << 15);
		pSPIHandle->pSPIx->CR1 |= tempreg;
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// BIDIMODE should be set
		tempreg |= (1 << 15);
		pSPIHandle->pSPIx->CR1 |= tempreg;
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDIMODE should be cleared
		tempreg &= ~(1 << 15);
		pSPIHandle->pSPIx->CR1 |= tempreg;
		// RXONLY bit must be set
		tempreg &= ~(1 << 10);
		pSPIHandle->pSPIx->CR1 |= tempreg;
	}

	// 3. Configure the Sclk Speed SPI_SclkSpeed;
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;
	pSPIHandle->pSPIx->CR1 |= tempreg;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;
	pSPIHandle->pSPIx->CR1 |= tempreg;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;
	pSPIHandle->pSPIx->CR1 |= tempreg;

	//6. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;
	pSPIHandle->pSPIx->CR1 |= tempreg;

	//7. Configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << 9;
	pSPIHandle->pSPIx->CR1 |= tempreg;
}

/**************************************************************************************************
 * @fn 					- SPI_DeInit
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**************************************************************************************************
 * @fn 					- SPI_SendData
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- This is blocking call
 */



void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){
		// 1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16 BIT DFF
			// 1. Load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			// 8 BIT
			// 1. Load the data in to the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/**************************************************************************************************
 * @fn 					- SPI_ReceiveData
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */
void SPI_ReceiveData(SPI_Handle_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

}
void SPI_IRQHandling(SPI_Handle_t *pHandle){

}

/**************************************************************************************************
 * @fn 					- SPI_PeripheralControl
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/**************************************************************************************************
 * @fn 					- SPI_SSOEConfig
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}




