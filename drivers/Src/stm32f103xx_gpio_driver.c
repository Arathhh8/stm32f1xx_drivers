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

	// Enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the speed
	//temp = 2;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->CRL |= temp; //setting
	}else{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)));
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->CRH |= temp;
	}

	// 2. Configure the mode of gpio pin
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_AF_OD){
		// then non interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
			// then the GPIO select is between Pin0 and Pin7 -> CRL
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->CRL |= temp;
		}else{
			// then the GPIO select is between Pin8 and Pin16 -> CRH
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)));
			pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (2 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8))); //clearing
			pGPIOHandle->pGPIOx->CRH |= temp;
		}

	}else{
		// this part will code later . (interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT){
			// 1. Configure the FTSR (Falling trigger detection register)
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
				// then the GPIO select is between Pin0 and Pin7 -> CRL
				temp = (2 << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // conf pupd
				pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
				pGPIOHandle->pGPIOx->CRL |= temp;
			}else{
				// then the GPIO select is between Pin8 and Pin16 -> CRH
				temp = (2 << (2 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8))); // conf pupd
				pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (2 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8))); //clearing
				pGPIOHandle->pGPIOx->CRH |= temp;
			}
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT){
			// 1. Configure the RTSR (Rising trigger detection register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT){
			// 1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Config the GPIO port selection in SYSCFG_EXTICR
			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			AFIO_PCLK_EN();
			AFIO->EXTICR[temp1] = portCode << (temp2 * 4);
		// 3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 3. Configure pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->ODR &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->ODR |= temp;

	// 4. Configure the optype   PUSH-PULL or OPEN DRAIN
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType == NO_CONFIG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->ODR &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->ODR |= temp;
	}

//	// 5. Configure the alt functionality
	temp = 0;
	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_AF_OD )){
		// configure the alternate function registers
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
			// then the GPIO select is between Pin0 and Pin7 -> CRL
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->CRL &= ~(0x3 << (2 + 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->CRL |= temp;
		}else{
			// then the GPIO select is between Pin8 and Pin16 -> CRH
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (2 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)));
			pGPIOHandle->pGPIOx->CRH &= ~(0x3 << (2 + 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8))); //clearing
			pGPIOHandle->pGPIOx->CRH |= temp;
		}
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){ //32 to 63
			// program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// program ISER2 register
			*NVIC_ISER1 |= (1 << IRQNumber % 64);
		}
	}else{
		if(IRQNumber <= 31){
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){ //32 to 63
			// program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

/**************************************************************************************************
 * @fn 					- GPIO_IRQPriorityConfig
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	// 1. First lets find out the IPR register  (iprx = interrupt priority register)
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber){
	// Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
}


