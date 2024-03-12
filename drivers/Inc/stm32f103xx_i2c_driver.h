/*
 * stm32f103xx_i2c_driver.h
 *
 *  Created on: Feb 24, 2024
 *      Author: arath
 */

#ifndef INC_STM32F103XX_I2C_DRIVER_H_
#define INC_STM32F103XX_I2C_DRIVER_H_

#include"stm32f103xx.h"

 /*
  * Configuration structure for I2Cx peripheral
  */

typedef struct {

	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */

typedef struct {

	I2C_RegDef_t *pI2Cx;			/*!< This holds the base address of the I2Cx(X:0,1,2) peripheral >*/
	I2C_Config_t I2C_Config;			/*!< This holds I2C pin configuration settings >*/

}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000		// Standard mode
#define I2C_SCL_SPEED_FM4K		400000		// Fast mode
#define I2C_SCL_SPEED_FM2K		200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE			1		// Standard mode
#define I2C_ACK_DISABLE			0		// Fast mode

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0		// Standard mode
#define I2C_FM_DUTY_16_9		1		// 16 / 9

/*
 * @I2C related status flags definitions
 */
#define I2C_FLAG_SB						(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR					(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF					(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10					(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF					(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RES1					(1 << I2C_SR1_RES1)
#define I2C_FLAG_RxNE					(1 << I2C_SR1_RxNE)
#define I2C_FLAG_TxE					(1 << I2C_SR1_TxE)
#define I2C_FLAG_BERR					(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO					(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF					    (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR					(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR					(1 << I2C_SR1_PEC_ERR)
#define I2C_FLAG_RES2					(1 << I2C_SR1_RES2)
#define I2C_FLAG_TIME_OUT				(1 << I2C_SR1_TIME_OUT)
#define I2C_FLAG_SMB_ALERT				(1 << I2C_SR1_SMB_ALERT)



/**************************************************************************************************************
 *								APIs supported by this driver
 *				For more information about the APIs check the function definitions
 *********************************************************************************************************** */


/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t  EnorDi);

/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);


/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Other peripheral control APIs
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Application CallBack
 */

void I2C_ApplicationEventCallBack(I2C_Handle_t *pHandle, uint8_t AppEv);










#endif /* INC_STM32F103XX_I2C_DRIVER_H_ */
