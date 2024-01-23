/*
 * 005button_interrupt.c
 *
 *  Created on: Jan 22, 2024
 *      Author: arath
 */

#include"stm32f103xx.h"
#include"string.h"

#define HIGH 1
#define LOW  0
#define BTN_PRESSED LOW

void delay(void){

	// this will introduce ~200ms delay when system clock is 16 MHz
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(){

	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));				/*  set all structure to 0   */
	memset(&GpioBtn,0,sizeof(GpioBtn));

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_OUT_MHZ_2;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;
	//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PP;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_MODE_IN_RESET_STATE;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = NO_CONFIG;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_MODE_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);

	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_13, GPIO_PIN_SET);
	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_13, GPIO_PIN_RESET);
	// IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1){
		/*if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == LOW){
			delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		}*/
	}

	return 0;
}


void EXTI9_5_IRQHandler(void){

	delay(); // 200ms

	GPIO_IRQHandling(GPIO_PIN_NO_9); // clear the pending event from EXTI line

	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
}

