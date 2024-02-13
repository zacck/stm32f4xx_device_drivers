/*
 * stm32f407xx_spi.h
 *
 *  Created on: Nov 1, 2023
 *      Author: zaccko
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"

// Configuration struct for SPIx Peripheral
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BUSConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;


//Handle struct for SPIx Peripheral
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	// Globals to store the data we need to send
	uint8_t 	*pTxBuffer;
	uint8_t 	*pRxBuffer;
	uint32_t 	TxLen;
	uint32_t 	RxLen;
	uint8_t 	TxState;
	uint8_t 	RxState;
}SPI_Handle_t;

// SPI Application states
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

// SPI APP EVENTS
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3



// Device Modes
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE 	0


// SPI Bus Modes
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3


// SPI CLock SPeed
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

//SPI DFF
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

//SPI CPOL
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

//SPI CPHA
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

//SPI SSM
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0


//APIs

// Init and DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//ClockSetup
void SPI_PCLK_CTRL(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Data TX and RX
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t*pSPIx, uint8_t *pRxBuffer, uint32_t Len);

//Interrupt Based
uint8_t SPI_SendDataIT(SPI_Handle_t  *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t  *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


//IRQ config and ISR handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


// FLags
#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1 << SPI_SR_BSY)
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);


// Other Peipheral control APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//application callback
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv);


#endif /* INC_STM32F407XX_SPI_H_ */
