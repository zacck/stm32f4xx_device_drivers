/*
 * ds1307_i2c_clock.c
 *
 *  Created on: Mar 28, 2024
 *      Author: zaccko
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

//extern void initialise_monitor_handles();



#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];


/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_MODE_OUT_OD;
	I2CPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl =
			ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);



}



int main(void)
{

	uint8_t commandcode;



	//printf("Application is running\n");

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, ENABLE);


	commandcode = 0x00;

	uint8_t rxes[2] = {0x00, 0x7F};

	I2C_MasterSendData(&I2C1Handle, rxes, 2, SLAVE_ADDR);

	delay();

	uint8_t read_time[3];

	I2C_MasterReceiveData(&I2C1Handle, read_time, 3, SLAVE_ADDR);

	while(1)
	{



		//uint8_t time[4] = {0, 3, 9, 0x60};

		//int time_size = (sizeof(time) / sizeof(time[0]));
		//commandcode = 0x80;

		//I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR);

		//uint8_t read_time[3] = {0};



		for (int i = 0; i < 3; i++) {

			//printf("Result = %d\n", read_time[i]);
		}

	}

}
