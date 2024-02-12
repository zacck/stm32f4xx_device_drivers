/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Feb 12, 2024
 *      Author: zaccko
 */

//#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

//command codes
#define COMMAND_LED_CTRL 		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ 		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ 		0x54

#define LED_ON		1
#define LED_OFF 	0


//F103XX analog PINS
#define ANALOG_PA0 		0
#define ANALOG_PA1 		1
#define ANALOG_PA2 		2
#define ANALOG_PA3 		3
#define ANALOG_PA4 		4
#define ANALOG_PA5 		5
#define ANALOG_PA6 		6
#define ANALOG_PA7 		7
#define ANALOG_PB1 		8

//Bluepill Onboard LED
#define LED_PIN 		13


void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}


//pb15 spi2_MOSI
//pb14 spi2_MISO
//pb13 spi2_SCLK
//pb12 spi2_NSS
//alt function mode 5
void SPI2_GPIOInits() {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5) {
		return 1;
	}
	return 0;
}

void SPI2_Inits() {
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BUSConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //hardware slave management

	SPI_Init(&SPI2Handle);

}

int main(void) {

	// SPI COMMAND BUTTON
	GPIO_Handle_t GpioBtn;

	// lets make a Button handler
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	//Enable the clock for port D & A
	GPIO_PCLK_CTRL(GPIOA, ENABLE);

	//init the pins with the above handler
	GPIO_Init(&GpioBtn);

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read, analog_read;

	//init gpio pins for SPI
	SPI2_GPIOInits();

	//Init Peripheral
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {

		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		SPI_PeripheralControl(SPI2, ENABLE);



		//1 CMD_LED_CTRL <pin_no> <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		SPI_SendData(SPI2, &commandcode, 1);

		// dummy read so we clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send a dummy bit to fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// now receive data
		SPI_ReceiveData(SPI2, &ackbyte, 1);


		if(SPI_VerifyResponse(ackbyte)){
			// connection established, now we can send commands
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//Send Message
			SPI_SendData(SPI2, args, 2);
		}

		// end LED command
		//2. CMD_SENSOR_READ <analog pin number>
		commandcode = COMMAND_SENSOR_READ;
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();
		SPI_SendData(SPI2, &commandcode, 1);

		// dummy read so we clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send a dummy bit to fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// now receive data
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {
			// connection established, now we can send commands
			args[0] = ANALOG_PA0;

			//Send Message
			SPI_SendData(SPI2, args, 1);
			// clear data
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			// send a dummy bit to fetch the response from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			//delay to give slave time to process
			delay();

			// read actual data
			SPI_ReceiveData(SPI2, &analog_read, 1);

		}

		//3 CMD_LED_READ pin no
		commandcode = COMMAND_LED_READ;
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();
		SPI_SendData(SPI2, &commandcode, 1);

		// dummy read so we clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send a dummy bit to fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// now receive data
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {

			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2, args, 1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
		}

		//4 CMD_PRINT message
		commandcode = COMMAND_PRINT;
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			;

		delay();
		SPI_SendData(SPI2, &commandcode, 1);

		// dummy read so we clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send a dummy bit to fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// now receive data
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t message[] = "Hello! Spooky SPI??";

		if (SPI_VerifyResponse(ackbyte)) {

			args[0] = strlen((char*) message);

			//send arguments
			SPI_SendData(SPI2, args, 1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			delay();

			//send message
			for (int i = 0; i < args[0]; i++) {
				SPI_SendData(SPI2, &message[i], 1);
				SPI_ReceiveData(SPI2, &dummy_read, 1);
			}

			//printf("COMMAND_PRINT Executed \n");
		}

		//5 CMD_ID+_READ
		commandcode = COMMAND_ID_READ;
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			;

		delay();
		SPI_SendData(SPI2, &commandcode, 1);

		// dummy read so we clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send a dummy bit to fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// now receive data
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		uint8_t id[11];
		uint32_t i=0;

		if (SPI_VerifyResponse(ackbyte)) {
			//read 10 bytes id from the slave
			for (i = 0; i < 10; i++) {
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[10] = '\0';
			//printf("COMMAND_ID : %s \n", id);

		}





		//Confirm that SPI2 not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}



