/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Feb 24, 2024
 *      Author: arath
 */

#include"stm32f103xx_i2c_driver.h"
uint16_t AHB_PreScaler[8] = {2,4,8,16,32,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

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

uint32_t RCC_GetPLLOutputClock(){

	return 0;
}

uint32_t RCC_GetPCLK1Value(void){

	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahpb, apb1p;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0){
		SystemClk = 8000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	//ahbp
	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp < 8){
		ahbp = 1;
	}else{
		ahpb = AHB_PreScaler[temp - 8];
	}

	//apb1
	temp = (RCC->CFGR >> 8) & 0x7;

	if(temp < 4){
		apb1p = 1;
	}else{
		ahpb = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahpb) / apb1p;

	return pclk1;
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

	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;

}


