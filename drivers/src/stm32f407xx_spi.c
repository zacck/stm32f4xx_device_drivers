/*
 * stm32f407xx_spi.c
 *
 *  Created on: Nov 1, 2023
 *      Author: zaccko
 */

#include "stm32f407xx_spi.h"

/******
 * @fn SPI_PCLK_CTRL
 *
 * @brief  Enables or Disables clock for a GPIO Port
 *
 * @params[pSPIx] port handle structure
 * @params[EnorDi] Enable or Disable
 *
 * @return void
 * @note
 *  */
void SPI_PCLK_CTRL(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}

	}

}

/******
 * @fn SPI_Init
 *
 * @brief  Initilizes a SPI peripheriral
 *
 * @params[pSPIx] port handle structure
 *
 * @return void
 * @note
 *  */

void SPI_Init(SPI_Handle_t *pSPIHandle) {
	//Enable Peripheral clock for SPI peripheral
	SPI_PCLK_CTRL(pSPIHandle->pSPIx, ENABLE);
	//configure CR1
	uint32_t tempreg = 0;

	//device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//bus config
	if (pSPIHandle->SPIConfig.SPI_BUSConfig == SPI_BUS_CONFIG_FD) {
		//clear bidimode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BUSConfig == SPI_BUS_CONFIG_HD) {
		//set bidi mode
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BUSConfig
			== SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//clear bidi
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//set rxonly
		tempreg |= (1 << SPI_CR1_RXONLY);

	}

	//configure clock
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// cofigure frame size
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//cpol
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/******
 * @fn SPI_DeInit
 *
 * @brief  Resets a SPI Perpheral
 *
 * @params[pSPIx] port handle structure
 *
 * @return void
 * @note
 *  */

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}

/******
 * @fn SPI_SendData
 *
 * @brief  Blocking API for sending Data over SPI, function will wait until all bytes are transmitted
 *
 * @params[pSPIx] port handle structure
 * @params[pTxBuffer] Outbound Data Buffer
 * @params[Len] Lenght of Data we are sending
 *
 * @return void
 * @note
 *  */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		//wait for TXE
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == (uint8_t)FLAG_RESET);

		// check DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			//16bit DFF
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			//decrease length
			Len--;
			Len--;
			//inc data pointer location
			(uint16_t*) pTxBuffer++;
		} else {
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}
/******
 * @fn SPI_ReceiveData
 *
 * @brief  Blocking API for Receiving Data over SPI, function will wait until all bytes are transmitted
 *
 * @params[pSPIx] port handle structure
 * @params[pTxBuffer] Outbound Data Buffer
 * @params[Len] Lenght of Data we are sending
 *
 * @return void
 * @note
 *  */
void SPI_ReceiveData(SPI_RegDef_t*pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0) {
		//wait for RXNE
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

		// check DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			//16bit DFF
			//load data from DR to RXBuffer
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			//decrease length
			Len--;
			Len--;
			//inc data pointer location
			(uint16_t*) pRxBuffer++;
		} else {
			// 8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}
}


/******
 * @fn SPI_GetFagStatus
 *
 * @brief  Gets a status from from t SR register
 *
 * @params[pSPIx] port handle structure
 * @params[FlagName] Outbound Data Buffer

 *
 * @return void
 * @note
 *  */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/******
 * @fn SPI_PeripheralControl
 *
 * @brief  Enables or Disables a SPI peripheral
 *
 * @params[pSPIx] port handle structure
 * @params[EnorDi] Enable or Disable

 *
 * @return void
 * @note
 *  */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/******
 * @fn SPI_SSICOnfig
 *
 * @brief  Enables or Disables SSI for a SPI
 *
 * @params[pSPIx] port handle structure
 * @params[EnorDi] Enable or Disable

 *
 * @return void
 * @note
 *  */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/******
 * @fn SPI_SSOEConfig
 *
 * @brief  Enables or Disables a SSOE for a SPI peripheral
 *
 * @params[pSPIx] port handle structure
 * @params[EnorDi] Enable or Disable

 *
 * @return void
 * @note
 *  */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

//IRQ config and ISR handling
/******
 * @fn SPI_IRQConfig
 *
 * @brief Flips the value at a given output pin
 *
 * @params[IRQNumber] IRQ number to configure
 * @params[EnorDi] Enable or disable
 *
 * @return none
 * @note
 *  */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi) {
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
 * @fn SPI_IRQConfig
 *
 * @brief Flips the value at a given output pin
 *
 * @params[IRQNumber] IRQ number to configure
 * @params[IRQPriority] Priority of the given IRQ
 *
 * @return none
 * @note
 *  */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

	//find out the IPR register to use
	uint8_t iprx = IRQNumber / 4;

	//section of register
	uint8_t iprx_section = IRQNumber % 4;


	//this is due to the lower nibble not being implemented in iPR regusters
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);


	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);


}





