/*
 * 001led_toggle.c
 *
 *  Created on: Oct 26, 2023
 *      Author: zaccko
 */


#include "stm32f407xx.h"


void delay(void){
	for(uint32_t i = 0; i < 500000/2; i ++);
}

int main(void){
	GPIO_Handle_t  GpioLed;

	// lets make a handler
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	//Enable the clock for port D
	GPIO_PCLK_CTRL(GPIOD, ENABLE);
	//init the pin with the above handler
	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
