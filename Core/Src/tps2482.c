/*
 * tps2482.c
 *
 *  Created on: 25.05.2022
 *      Author: johan
 */
#include "tps2482.h"

#define I2C_TIMEOUT		10

void tps2482_init(TPS2482_HandleTypeDef *htps2482, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
//	htps2482->hi2c = hi2c;
	htps2482->addr = addr << 1;
	htps2482->plugged = 0;
}

HAL_StatusTypeDef InitTPS2482(TPS2482_HandleTypeDef* htps2482)
{
	uint8_t initdata = 0x45;
	uint8_t* p_data = &initdata;
	//size 1 in byte
	return HAL_I2C_Mem_Write(htps2482->hi2c, htps2482->addr, TPS2482_REGISTER_CONFIG, 2, p_data, 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef ReadTPS2482(TPS2482_HandleTypeDef* htps2482)
{
	uint8_t* p_data = &htps2482->data[0];
	return HAL_I2C_Mem_Read(htps2482->hi2c, htps2482->addr, TPS2482_REGISTER_CURRENT, 2, p_data, 2, I2C_TIMEOUT);
}
