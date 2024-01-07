/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Jan 3, 2024
 *      Author: arath
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

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

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_MODE_IN_RESET_STATE 	0
#define GPIO_SPEED_OUT_MHZ_10 		1
#define GPIO_SPEED_OUT_MHZ_2 		2
#define GPIO_SPEED_OUT_MHZ_50 		3

/*
 * @GPIO_PIN_MODE
 * GPIO pin possible modes
 */

// in input mode
#define GPIO_MODE_ANALOG 			0
#define GPIO_MODE_INPUT_FLOATING 	1
#define GPIO_MODE_IN_PUPD 			2
// in output mode
#define GPIO_MODE_OUT_PP 			0
#define GPIO_MODE_OUT_OD 			1
#define GPIO_MODE_AF_PP 			2
#define GPIO_MODE_AF_OD 			3
// in interrupt mode
#define GPIO_MODE_IT_FT				0
#define GPIO_MODE_IT_RT				1
#define GPIO_MODE_IT_RFT			2

/*
 * @GPIO_PIN_PUPD
 * GPIO pin possible pull-up or pull-down config
 */
#define GPIO_MODE_PD				0
#define GPIO_MODE_PU				1

/*
 * @GPIO_PIN_PPOD
 * GPIO pin possible push-pull or open-drain config
 */
#define GPIO_MODE_PD				0
#define GPIO_MODE_PU				1

/**************************************************************************************************************
 *								APIs supported by this driver
 *				For more information about the APIs check the function definitions
 *********************************************************************************************************** */


/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t  EnorDi);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Handle_t *pGPIOx);

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);











#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
