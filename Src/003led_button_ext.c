/*
 * 003led_button_ext.c
 *
 *  Created on: Jan 16, 2024
 *      Author: arath
 */

#include"stm32f103xx.h"

void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(){

	GPIO_Handle_t GpioLed, GpioBtn;

	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_OUT_MHZ_50;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;
	//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PP;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_MODE_IN_RESET_STATE;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == LOW){
			delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
		}

		}

	return 0;
}
