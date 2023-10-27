/*
 * 002led_button.c
 *
 *  Created on: Oct 27, 2023
 *      Author: zaccko
 */
#include "stm32f407xx.h"


void delay(void){
	for(uint32_t i = 0; i < 500000/2; i ++);
}

int main(void){
	GPIO_Handle_t  GpioLed, GpioBtn;

	// lets make a LED handler
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	// lets make a Button handler
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	//Enable the clock for port D & A
	GPIO_PCLK_CTRL(GPIOD, ENABLE);
	GPIO_PCLK_CTRL(GPIOA, ENABLE);

	//init the pins with the above handler
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);

	while(1){
		//read button status
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)){
			//debounce button press
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

	}

	return 0;
}


