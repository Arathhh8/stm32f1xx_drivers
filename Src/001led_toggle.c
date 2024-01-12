/*
 * 001led_toggle.c
 *
 *  Created on: Jan 8, 2024
 *      Author: arath
 */

#include"stm32f103xx.h"

void delay(void){
	for(uint32_t i = 0; i < 500000; i++);
}

int main(){

	GPIO_Handle_t GpioLed;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_OUT_MHZ_50;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PD;

	GPIO_Init(&GpioLed);

	while(1){
		//GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		delay();
	}

	return 0;
}
