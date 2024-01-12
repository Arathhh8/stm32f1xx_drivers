/*
 * 002led_button.c
 *
 *  Created on: Jan 11, 2024
 *      Author: arath
 */


#include"stm32f103xx.h"

#define HIGH 1
#define LOW  0

void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(){

	GPIO_Handle_t GpioLed, GpioBtn;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_OUT_MHZ_50;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;
	//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PP;

	GPIO_PeriClockControl(GPIOC, ENABLE);
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
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		}

		}

	return 0;
}
