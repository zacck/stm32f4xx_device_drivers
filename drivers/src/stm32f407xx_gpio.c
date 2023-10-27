/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Oct 25, 2023
 *      Author: zaccko
 */

#include "stm32f407xx_gpio.h"

/******
 * @fn GPIO_INITL
 *
 * @brief  Enables or Disables clock for a GPIO Port
 *
 * @params[pGPIOx] port handle structure
 *
 * @return void
 * @note
 *  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp = 0;
	uint32_t temp1, temp2;
	// configure mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;

	} else {
		// this is an interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//set FTSR and reset RTSR
			EXTI ->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI ->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//set RTSR and reset FTSR
			EXTI ->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI ->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			//set RTSR and FTSR
			EXTI ->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI ->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// port selection in sysconfig
		//select the CR register
		uint8_t cr_reg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/ 4;
		//select the offset on the cr  reg offset
		uint8_t cr_offset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode =  GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		//enable sysconfig clk
		SYSCFG_EN();

		// configure sys cfg
		SYSCFG->EXTICR[cr_reg] = portcode << (cr_offset * 4);

		//enable exti using imr
		EXTI -> IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		//get index for AFR to use
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		// get register to use
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFR[temp1] &= (0XF << (4 * temp2));

		pGPIOHandle->pGPIOx->AFR[temp1] |=
				(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}

	temp = 0;

	//configure speed

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}

}

/******
 * @fn GPIO_PCLK_CTRL
 *
 * @brief  Enables or Disables clock for a GPIO Port
 *
 * @params[pGPIOx] port base address
 * @params[EnorDi] Enable or Disable MAcros from header
 *
 * @return void
 * @note
 *  */
void GPIO_PCLK_CTRL(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();

		}

	}

}
//Data Read and Write
/******
 * @fn GPIO_ReadFromInputPin
 *
 * @brief  Reads from an input pin and returns the 8 bit value
 *
 * @params[pGPIOx] port base address
 * @params[PinNumber] Pin Number to read from
 *
 * @return 8 bit integer 0 or 1
 * @note
 *  */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;

	// we right shift the IDR by the pin number that way
	// the value of the pin becomes the LSB
	// then we use a mask to read from this bit and return the
	// read value
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/******
 * @fn GPIO_ReadFromInputPort
 *
 * @brief  Reads from an input port and returns the read value
 *
 * @params[pGPIOx] port base address
 *
 *
 * @return
 * @note
 *  */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;

	value = (uint16_t) pGPIOx->IDR;
	return value;
}

/******
 * @fn GPIO_WriteToOutputPin
 *
 * @brief Writes a value to an output pin
 *
 * @params[pGPIOx] port base address
 * @params[PinNumber] Pin Number to write to
 * @params[value] value to write to the Pin
 * @return none
 * @note
 *  */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value)
{

	//determine what value we want to write
	//based on set and reset macros
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);

	}

}

/******
 * @fn GPIO_WriteToOutputPort
 *
 * @brief Writes a value to an output port
 *
 * @params[pGPIOx] port base address
 * @params[value] value to write to the Port
 * @return none
 * @note
 *  */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;

}
/******
 * @fn GPIO_ToggleOutputPin
 *
 * @brief Flips the value at a given output pin
 *
 * @params[pGPIOx] port base address
 * @params[PinNumber] Pin Number to Flip
 *
 * @return none
 * @note
 *  */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	//use XOR to toggle bitfield
	pGPIOx->ODR = pGPIOx ->ODR ^ (1 << PinNumber);
}

//IRQ config and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}
void GPIO_IRQHandling(uint8_t PinNumber) {

}
