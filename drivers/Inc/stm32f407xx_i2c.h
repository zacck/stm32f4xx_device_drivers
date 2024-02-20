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








#endif /* INC_STM32F407XX_I2C_H_ */
