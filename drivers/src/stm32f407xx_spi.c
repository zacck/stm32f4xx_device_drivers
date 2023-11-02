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

void SPI_Init(SPI_Handle_t *pSPIHandle);

