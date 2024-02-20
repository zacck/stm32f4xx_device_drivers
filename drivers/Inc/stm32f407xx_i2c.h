/*
 * stm32f4xx_i2c.h
 *
 *  Created on: Feb 20, 2024
 *      Author: zaccko
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_

#include "stm32f407xx.h"


//TYPE Struct

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;


//Handle Struct

typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

//SPEEDS
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM 400000

//ACK Control
#define I2C_ACK_EN		1
#define I2C_ACK_DI		0

//FM DUTY CYCLE
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

//APIs

// Init and DeInit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

//ClockSetup
void I2C_PCLK_CTRL(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Data TX and RX


//IRQ config and ISR handling
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

// Other Peipheral control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2cx, uint8_t EnorDi);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


//Application callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);




#endif /* INC_STM32F407XX_I2C_H_ */
