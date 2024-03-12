/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Feb 24, 2024
 *      Author: arath
 */

#include"stm32f103xx_i2c_driver.h"
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADRRFlag(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // SlaveAdrr is Slave address + r/nw
	pI2Cx->DR = SlaveAddr;

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){

	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

static void I2C_ClearADRRFlag(I2C_RegDef_t *pI2Cx){

	uint32_t dummyRead = pI2Cx->SR1;
    dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}




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

	uint8_t clksrc, temp, ahbp, apb1p;

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
		ahbp = AHB_PreScaler[temp - 8];
	}

	//apb1
	temp = (RCC->CFGR >> 8) & 0x7;

	if(temp < 4){
		apb1p = 1;
	}else{
		ahbp = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

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


	uint32_t tempreg = 0;  // temporal register

	// Enable the clock for the I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Enable the I2C peripheral
	I2C_PeripheralControl(I2C2, ENABLE);
	// ACK control bit
	//pI2CHandle->pI2Cx->CR1 = tempreg;
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	// Configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	//pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);
	pI2CHandle->pI2Cx->CR2 = 0x8;

	// Program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR Calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}else{
		// mode is fast mode
		tempreg |= (1 << 15); // configuring bit 15 of I2C_CCR
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}else{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/**************************************************************************************************
 * @fn 					- I2C_DeInit
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

void I2C_DeInit(I2C_RegDef_t *pI2Cx){

}

/**************************************************************************************************
 * @fn 					- I2C_MasterSendData
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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr){
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	// NOTE: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that adress phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear the ADDR Flag according to its software sequence
	// NOTE: until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADRRFlag(pI2CHandle->pI2Cx);

	// 6. Send data until Len becomes 0
	while(Len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE)); // Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. When Len becomes 0 wait for TXE = 1 and BTF = 1 before generating the STOP condition
	// NOTE: TXE = 1, BF1 = 1, means that both SR and DR are empty and next transmission should begin
	// when BTF = 1 SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. Generate STOP condition and master need not to wait for the completion of stop condition
	// NOTE: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}












