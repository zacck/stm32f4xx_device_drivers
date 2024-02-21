/*
 * stm32f4xx_i2c.c
 *
 *  Created on: Feb 20, 2024
 *      Author: zaccko
 */

#include "stm32f407xx_spi.h"
//array of ahb facors
uint16_t AHB_PreScaler[9] =  {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] =  {2,4,8,16};


uint32_t RCC_GetPCLK1Value(void){

	uint32_t pClock1, temp, SystemClock;

	//find clock source
	uint8_t clksrc, ahbp, apb1p;
	clksrc = ((RCC->CFGR >> 2) & 0x3); //2 is the clock source bit field

	// which clock
	if(clksrc == 0){
		SystemClock = 16000000;
	} else if(clksrc == 1){
		SystemClock = 8000000;
	} else if(clksrc == 2){
		//pll source calculation
	}

	// AHB prescaler value
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else {
		ahbp = AHB_PreScaler[temp - 8];
	}


	// APB1 Prescaler value

	temp = 0;

	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScaler[temp - 4];
	}

	// pclock1 value

	pClock1 = (SystemClock/ahbp) / apb1p;

	return pClock1;

}

/******
 * @fn I2C_Init
 *
 * @brief Initializes a I2C Perpheral
 *
 * @params[pI2Cx] port handle structure
 *
 * @return void
 * @note
 *  */

void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t temp_reg = 0;


	// ack control
	temp_reg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;

	pI2CHandle->pI2Cx->CR1 = temp_reg;

	//CR2 items

	temp_reg = 0;

	temp_reg |= RCC_GetPCLK1Value() / 1000000;

	pI2CHandle->pI2Cx->CR2 = temp_reg & 0x3F;


	/// Only applies if we are in Slave Mode
	temp_reg = 0;
	temp_reg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;

	// TODO consider addressing mode

	temp_reg |= (1 << 14); // 14th bit needs to be kept 1 by software

	pI2CHandle->pI2Cx->OAR1 = temp_reg;

	// CCR Register
	uint16_t ccr_value =  0;
	temp_reg = 0;


	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2  * pI2CHandle->I2C_Config.I2C_SCLSpeed));

		temp_reg |= (ccr_value & 0xFFF);
	} else {
		// fast mode
		//set mode
		temp_reg |=  (1 << 15); // Mode bit

		// duty cycle
		temp_reg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14); // duty cycle bit

		//ccr setting
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value() / ( 3  * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			ccr_value = (RCC_GetPCLK1Value() / ( 25  * pI2CHandle->I2C_Config.I2C_SCLSpeed));

		}

		temp_reg |= (ccr_value & 0xFFF);

	}

	pI2CHandle->pI2Cx->CCR = temp_reg;

}


/******
 * @fn I2C_DeInit
 *
 * @brief  Resets a I2C Perpheral
 *
 * @params[pI2Cx] port handle structure
 *
 * @return void
 * @note
 *  */

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/******
 * @fn I2C_PeripheralControl
 *
 * @brief  Enables or Disables a I2C peripheral
 *
 * @params[pI2Cx] port handle structure
 * @params[EnorDi] Enable or Disable

 *
 * @return void
 * @note
 *  */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

//IRQ config and ISR handling
/******
 * @fn I2C_IRQConfig
 *
 * @brief Flips the value at a given output pin
 *
 * @params[IRQNumber] IRQ number to configure
 * @params[EnorDi] Enable or disable
 *
 * @return none
 * @note
 *  */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		// we only handle the first 3 registers as our MCU
		// Can only handle 88 interrupts
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}

	} else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}

	}

}

/******
 * @fn I2C_IRQConfig
 *
 * @brief Flips the value at a given output pin
 *
 * @params[IRQNumber] IRQ number to configure
 * @params[IRQPriority] Priority of the given IRQ
 *
 * @return none
 * @note
 *  */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

	//find out the IPR register to use
	uint8_t iprx = IRQNumber / 4;

	//section of register
	uint8_t iprx_section = IRQNumber % 4;


	//this is due to the lower nibble not being implemented in iPR regusters
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);


	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);


}



