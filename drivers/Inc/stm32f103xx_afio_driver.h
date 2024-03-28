/*
 * stm32f103xx_afio_driver.h
 *
 *  Created on: Mar 26, 2024
 *      Author: arath
 */

#ifndef INC_STM32F103XX_AFIO_DRIVER_H_
#define INC_STM32F103XX_AFIO_DRIVER_H_

#include"stm32f103xx.h"

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct{
	uint8_t GPIO_PinNumber;				/*!< possible values from @GPIO_PIN_NUMBER >*/
	uint8_t GPIO_PinMode;				/*!< possible values from @GPIO_PIN_MODE >*/
	uint8_t GPIO_PinSpeed;				/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;		/*!< possible values from @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOPType;				/*!< possible values from @GPIO_PIN_PPOD >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;			/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;


#endif /* INC_STM32F103XX_AFIO_DRIVER_H_ */
