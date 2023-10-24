/*
 * stm32f407xx.h
 *
 *  Created on: Oct 24, 2023
 *      Author: zaccko
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

// base addresses of flash and RAM memories

#define FLASH_BASEADDR  		0x08000000U
#define SRAM1_BASEADDR  		0x20000000U
#define SRAM2_BASEADDR 			0x20001C00U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR


// bus domains of the controllers  AHB and APB

#define PERIPH_BASE 			0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U




#endif /* INC_STM32F407XX_H_ */
