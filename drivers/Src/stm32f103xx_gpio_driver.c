/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Jan 3, 2024
 *      Author: arath
 */

#include"stm32f103xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/**************************************************************************************************
 * @fn 					- GPIO_PeriClockControl
 *
 * @brief				- This function enable or disables peripheral clock for the given GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macros
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t  EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();}
		}
	}
}

/**************************************************************************************************
 * @fn 					- GPIO_Init and GPIO_DeInit
 *
 * @brief				- This function
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 */

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0; // temporal variable

	// 1. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->CRL |= temp; //setting
	}else{
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->CRH |= temp;
	}

	// 2. Configure the mode of gpio pin
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_AF_OD){
		// then non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
			// then the GPIO select is between Pin0 and Pin7 -> CRL
			pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->CRL |= temp;
		}else{
			// then the GPIO select is between Pin8 and Pin16 -> CRH
			pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->CRH |= temp;
		}

	}else{
		// this part will code later . (interrupt mode)
	}

	// 3. Configure pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->ODR &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->ODR |= temp;

	// 4. Configure the optype   PUSH-PULL or OPEN DRAIN
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->ODR &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->ODR |= temp;

	// 5. Configure the alt functionality
	temp = 0;
	if(temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode >= GPIO_MODE_AF_PP )){
		// configure the alternate function registers
	}
}
void GPIO_DeInit(GPIO_Handle_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
}

/**************************************************************************************************
 * @fn 					- GPIO_ReadFromInputPin
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- 0 or 1
 *
 * @Note				- none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = ((uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**************************************************************************************************
 * @fn 					- GPIO_ReadFromInputPort
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- 0 or 1
 *
 * @Note				- none
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**************************************************************************************************
 * @fn 					- GPIO_WriteToOutputPin
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- 0 or 1
 *
 * @Note				- none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		// write 1 to the output data register at the bit field corresponding pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**************************************************************************************************
 * @fn 					- GPIO_WriteToOutputPort
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- 0 or 1
 *
 * @Note				- none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;
}

/**************************************************************************************************
 * @fn 					- GPIO_ToggleOutputPin
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- 0 or 1
 *
 * @Note				- none
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){

}
void GPIO_IRQHandling(uint8_t PinNumber){

}


