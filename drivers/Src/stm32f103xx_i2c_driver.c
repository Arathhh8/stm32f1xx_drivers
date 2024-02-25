/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Feb 24, 2024
 *      Author: arath
 */

#include"stm32f103xx_i2c_driver.h"


/**************************************************************************************************
 * @fn 					- I2C_PeriClockControl
 *
 * @brief				- This function enable or disables peripheral clock for the given I2C
 *
 * @param[in]			- base address of the I2C peripheral
 * @param[in]			- ENABLE or DISABLE macros
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t  EnorDi){

	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
	}else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
	}
}

	/**************************************************************************************************
	 * @fn 					- I2C_PeripheralControl
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

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/**************************************************************************************************
 * @fn 					- I2C_Init
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

void I2C_Init(I2C_Handle_t *pI2CHandle){

	// Enable peripheral clock
	//SPI_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// First lets configure the SPI_CR1 register
	uint32_t tempreg = 0;  // temporal register

	pI2CHandle->pI2Cx->CR1 = tempreg;
	// 1. Configure the SCLSpeed
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM){
		tempreg |= (0 << 15);
		pI2CHandle->pI2Cx->CCR = tempreg;
	}else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM4K){
		tempreg |= (1 << 15);
		pI2CHandle->pI2Cx->CCR = tempreg;
	}
}


