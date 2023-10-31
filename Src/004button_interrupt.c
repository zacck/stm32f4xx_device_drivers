/*
 * 004button_interrupt.c
 *
 *  Created on: Oct 30, 2023
 *      Author: zaccko
 */

#include "stm32f407xx.h"

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

int main(void) {
	GPIO_Handle_t GpioLed, GpioBtn;



	// lets make a handler
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	// lets make a Button handler
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;

	//Enable the clock for port D
	GPIO_PCLK_CTRL(GPIOD, ENABLE);


	//init the button and led with the above handlers
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);


	//IRQ Coonfiguration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI3, NVIC_IRQ_PRIO15);
	GPIO_IRQConfig(IRQ_NO_EXTI3 , ENABLE);
	while(1){
//		//read button status
//				if(!GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_3)){
//					//debounce button press
//					delay();
//					GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
//				}
	}

}

void EXTI3_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_3);
	delay();
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

}

