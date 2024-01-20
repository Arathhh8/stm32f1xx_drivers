/*
 * main.c
 *
 *  Created on: Jan 20, 2024
 *      Author: arath
 */

#include"stm32f103xx.h"

int main(){

	return 0;
}


void EXTI0_IRQHandler(void){

	// handle the interrupt
	GPIO_IRQHandling(0);
}
