/*
 * tps2482.h
 *
 *  Created on: 25.05.2022
 *      Author: johan
 */

#include "stm32g4xx_hal.h"

/* Slave adresses */
#define TPS2482_SLAVE0	0x80
#define TPS2482_SLAVE1	0x82
#define TPS2482_SLAVE2	0x84
#define TPS2482_SLAVE3	0x86
#define TPS2482_SLAVE15	0x4F

/* TPS2482 Registers */
#define TPS2482_REGISTER_CONFIG		0x00
#define TPS2482_REGISTER_VOLTAGE	0x01
#define TPS2482_REGISTER_BVOLTAGE	0x02
#define TPS2482_REGISTER_POWER		0x03
#define TPS2482_REGISTER_CURRENT	0x04
#define TPS2482_REGISTER_CALIB		0x05

typedef struct {
	I2C_HandleTypeDef*	hi2c;			//I2C Handler
	uint16_t			addr;			//Slave Adress
	uint8_t				data[2];		//Last pulled data
	uint8_t				plugged;		//Binary value wether device is plugged in
} TPS2482_HandleTypeDef;

void tps2482_init(TPS2482_HandleTypeDef *htps2482, I2C_HandleTypeDef *hi2c, uint16_t addr);
HAL_StatusTypeDef InitTPS2482(TPS2482_HandleTypeDef* htps2482);

#ifndef INC_TPS2482_H_
#define INC_TPS2482_H_





#endif /* INC_TPS2482_H_ */
