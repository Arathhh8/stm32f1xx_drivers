/*
 * 015uart_tx.c
 *
 *  Created on: Mar 31, 2024
 *      Author: arath
 */


/*
 	PB10 -> TX
 	PB11 -> RX
*/

#include<string.h>
#include"stm32f103xx.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart3_handle;


void delay(void){
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


void USART3_Init(void){

	usart3_handle.pUSARTx = USART3;
	usart3_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart3_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart3_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart3_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart3_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart3_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart3_handle);

}


void USART2_GPIOInit(void){
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOB;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_OUT_MHZ_50;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_AF_PP;

	// USAT2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&usart_gpios);

	// USAT2 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&usart_gpios);
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

	GPIO_ButtonInit();

	USART2_GPIOInit();

	USART3_Init();

	USART_PeripheralControl(USART3, ENABLE);

	while(1){

		// wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart3_handle, (uint8_t*)msg, strlen(msg));

	}

	return 0;
}




