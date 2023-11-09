/*
 * 007_spi_txonly_ard.c
 *
 *  Created on: Nov 8, 2023
 *      Author: zaccko
 */

#include<string.h>
#include "stm32f407xx.h"

//pb15 spi2_MOSI
//pb14 spi2_MISO
//pb13 spi2_SCLK
//pb12 spi2_NSS
//alt function mode 5

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

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

	/*MISO
	 SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	 GPIO_Init(&SPIPins);
	 */

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
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

	char user_data[] = "Zacck from Stm32";

	//init gpio pins for SPI
	SPI2_GPIOInits();

	//Init Peripheral
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {

		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		SPI_PeripheralControl(SPI2, ENABLE);

		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		//uint16_t value = (uint16_t)SPI2->DR;

		//Send Data
		SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

		//Confirm that SPI2 not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

