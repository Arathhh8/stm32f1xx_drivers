/*
 * 011i2c_master_rx_testing2.c
 *
 *  Created on: Mar 23, 2024
 *      Author: arath
 */


#include<stdio.h>
#include<string.h>
#include "stm32f103xx.h"

extern void initialise_monitor_handles();

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68


// rcv bffr
uint8_t rcv_bffr[32];

void delay(void){
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


I2C_Handle_t I2C1Handle;
/*
 * PB6 -->  I2C1_SCL
 * PB7 --> I2C1_SDA
 */

void I2C1_GPIOInits(void){

	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_AF_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_OUT_MHZ_10;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void){

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void){

	GPIO_Handle_t GpioBtn;
	// This is Btn gpio configuration
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_MODE_IN_RESET_STATE;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PU;

	GPIO_Init(&GpioBtn);
}


int main(void){
	/* initialise_monitor_handles();
	printf("Hello STM32\n"); */

	uint8_t commandCode;
	uint8_t len;

	GPIO_ButtonInit();

	// I2C Pin inits
	I2C1_GPIOInits();

	// I2C Peripheral configuration
	I2C1_Inits();

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	// Wait for button press
	while(1){
		// Wait till button is pressed
			while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

			// To avoid button de-bouncing related issues 200ms of delay
			delay();

			commandCode = 0x51;

			I2C_MasterSendData(&I2C1Handle, &commandCode , 1, SLAVE_ADDR);

			I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR);

			commandCode = 0x52;
			I2C_MasterSendData(&I2C1Handle, &commandCode , 1, SLAVE_ADDR);

			I2C_MasterReceiveData(&I2C1Handle, rcv_bffr, len, SLAVE_ADDR);

	}
}




