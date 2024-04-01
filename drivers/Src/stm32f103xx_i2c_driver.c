/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: Feb 24, 2024
 *      Author: arath
 */

#include"stm32f103xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADRRFlag(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // SlaveAdrr is Slave address + r/nw bit = 0
	pI2Cx->DR = SlaveAddr;

}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; // SlaveAdrr is Slave address + r/nw bit = 1
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
	I2C_PeripheralControl(I2C1, ENABLE);
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

	//I2C_PeripheralControl(I2C1, ENABLE);
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	// NOTE: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	//I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
	pI2CHandle->pI2Cx->DR |= ((SlaveAddr<<1)&0xFE);	//Shift Slave Address to insert at 0th position the r/w bit (write)
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

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr){

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADRRFlag(pI2CHandle->pI2Cx);
		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADRRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == I2C_ACK_ENABLE){
		// enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else{
		// disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}











