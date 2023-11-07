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
void SPI_PCLK_CTRL(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
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
 * @brief  Enables or Disables clock for a GPIO Port
 *
 * @params[pSPIx] port handle structure
 *
 * @return void
 * @note
 *  */

void SPI_Init(SPI_Handle_t *pSPIHandle){
	//configure CR1
	uint32_t  tempreg  = 0;

	//device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//bus config
	if(pSPIHandle->SPIConfig.SPI_BUSConfig == SPI_BUS_CONFIG_FD){
		//clear bidimode
		tempreg &= ~(1<< SPI_CR1_MSTR);
	} else if(pSPIHandle->SPIConfig.SPI_BUSConfig == SPI_BUS_CONFIG_HD){
		//set bidi mode
		tempreg |= (1<< SPI_CR1_MSTR);

	} else if(pSPIHandle->SPIConfig.SPI_BUSConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//clear bidi
		tempreg &= ~(1<< SPI_CR1_MSTR);
		//set rxonly
		tempreg |= (1<< SPI_CR1_RXONLY);

	}

	//configure clock
	tempreg  |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// cofigure frame size
	tempreg  |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//cpol
	tempreg  |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//CPHA
	tempreg  |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;






}

