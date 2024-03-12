/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Mar 9, 2024
 *      Author: arath
 */


#include<stdio.h>
#include<string.h>
#include "stm32f103xx.h"

extern void initialise_monitor_handles();

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68


// some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

void delay(void){
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


I2C_Handle_t I2C2Handle;
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
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void){

	I2C2Handle.pI2Cx = I2C2;
	I2C2Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C2Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C2Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C2Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C2Handle);
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
	initialise_monitor_handles();
	printf("Hello STM32\n");

	GPIO_ButtonInit();

	// I2C Pin inits
	I2C1_GPIOInits();


	// I2C Peripheral configuration
	I2C1_Inits();

	// Wait for button press
	while(1){
		// Wait till button is pressed
			while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

			// To avoid button de-bouncing related issues 200ms of delay
			delay();

			// Send some data to the slave
			I2C_MasterSendData(&I2C2Handle,some_data, strlen((char*)some_data), SLAVE_ADDR);
	}
}










