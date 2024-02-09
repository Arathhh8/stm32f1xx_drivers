/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Feb 2, 2024
 *      Author: arath
 */

#include"stm32f103xx.h"
#include"stdio.h"
#include<string.h>

extern void initialise_monitor_handles();



// Comand codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0
#define LED_PIN					9

// Arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

/*
 * 	 PB15 --> SPI2_MOSI
 * 	 PB14 --> SPI2_MISO
 *	 PB13 --> SPI2_SCK
 *	 PB12 --> SPI2_NSS
 */

void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++);
}

void SPI2_GPIOInits(void){

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_OUT_MHZ_50;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // generates SLCK of 8 MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave managment enabled for NSS pin

	SPI_Init(&SPI2handle);
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

void GPIO_LedInit(void){

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_OUT_MHZ_50;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);
}

uint8_t SPI_VeryfyResponse(uint8_t ackByte){

	if(ackByte == 0xF5){
		// ack
		return 1;
	}else{
		return 0;
	}
}

int main(void){

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();
	GPIO_LedInit();

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	printf("SPI Init. Done\n");

	/*
	 *  Making SSOE 1 does NSS output enable
	 *  The NSS pin is automatically managed by hardware
	 *  i.e when SPE = 1, NSS will be pulled to LOW
	 *  and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
	// Wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		// Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// 1. CMD_LED_CTRL <pin no(1)>		<value(1)>
		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackByte;
		uint8_t args[2];

		// Send command
		SPI_SendData(SPI2, &commandCode, 1);

		// Do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// Read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(SPI_VeryfyResponse(ackByte)){
			// Send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// send args
			SPI_SendData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL Executed\n");
		}
		// End of COMMAND_LED_CTRL

		// 2. CMD_SENSOR_READ	<analog pin number(1)>

		// Wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_SENSOR_READ;

		// Send command
		SPI_SendData(SPI2, &commandCode, 1);

		// Do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// Read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(SPI_VeryfyResponse(ackByte)){
			// Send arguments
			args[0] = ANALOG_PIN0;
			// send args
			SPI_SendData(SPI2, args, 1); // sending one byte of

			// Do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// Insert some delay so that slave can ready with the data
			delay();

			// Send some dummy bits (1 byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ %d\n", analog_read);
		}
		// End of COMMAND_SENSOR_READ

		// 3. COMMAND_LED_READ	<analog pin number(1)>

		// Wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// To avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_LED_READ;

		// Send command
		SPI_SendData(SPI2, &commandCode, 1);

		// Do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send some dummy bits (1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// Read the ack byte received
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(SPI_VeryfyResponse(ackByte)){
			// Send arguments
			args[0] = LED_PIN;
			// send args
			SPI_SendData(SPI2, args, 1); // sending one byte of

			// Do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// Insert some delay so that slave can ready with the data
			delay();

			// Send some dummy bits (1 byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("COMMAND_LED_READ %d\n", led_status);
		}
		// End of COMMAND_LED_READ


		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2,&commandCode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackByte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VeryfyResponse(ackByte)){
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,args,1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI2,&message[i],1);
				SPI_ReceiveData(SPI2,&dummy_read,1);
			}

			printf("COMMAND_PRINT Executed \n");

		} // End of COMMAND_PRINT

		//5. CMD_ID_READ

		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2,&commandCode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackByte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VeryfyResponse(ackByte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		} // End of COMMAND_ID_READ


		// Lets confirm SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI Communication closed\n");
	}

	return 0;
}