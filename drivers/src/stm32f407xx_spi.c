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
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

