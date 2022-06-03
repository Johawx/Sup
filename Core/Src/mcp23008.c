/**
  ******************************************************************************
  * @file    mcp23008.c
  * @brief   This file provides code for the control of
  *          the mcp23008 i2c port expander.
  ******************************************************************************
  * @attention
  * CURE 2022
  ******************************************************************************
  */
#include "mcp23008.h"

// Registers
#define REGISTER_IODIR		0x00
#define REGISTER_IPOL		0x01
#define REGISTER_GPINTEN	0x02
#define REGISTER_DEFVAL		0x03
#define REGISTER_INTCON		0x04
//	IOCON			0x05
#define REGISTER_GPPU		0x06
#define REGISTER_INTF		0x07
#define REGISTER_INTCAP		0x08
#define REGISTER_GPIO		0x09
#define REGISTER_OLAT		0x0A

#define I2C_TIMEOUT		10

void mcp23008_init(MCP23008_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
	hdev->hi2c = hi2c;
	hdev->addr = addr << 1;
}

HAL_StatusTypeDef mcp23008_read(MCP23008_HandleTypeDef *hdev, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, reg, 1, data, 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef mcp23008_write(MCP23008_HandleTypeDef *hdev, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, reg, 1, data, 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef mcp23008_iodir(MCP23008_HandleTypeDef *hdev, uint8_t iodir)
{
	uint8_t data[1] = {iodir};
	return mcp23008_write(hdev, REGISTER_IODIR, data);
}

HAL_StatusTypeDef mcp23008_ipol(MCP23008_HandleTypeDef *hdev, uint8_t ipol)
{
	uint8_t data[1] = {ipol};
	return mcp23008_write(hdev, REGISTER_IPOL, data);
}

HAL_StatusTypeDef mcp23008_ggpu(MCP23008_HandleTypeDef *hdev, uint8_t pu)
{
	uint8_t data[1] = {pu};
	return mcp23008_write(hdev, REGISTER_GPPU, data);
}

HAL_StatusTypeDef mcp23008_read_gpio(MCP23008_HandleTypeDef *hdev)
{
	uint8_t data[1];
	data[0] = 0x00;
	HAL_StatusTypeDef status;
	status = mcp23008_read(hdev, REGISTER_GPIO, data);
	if (status == HAL_OK)
		hdev->gpio[0] = data[0];
	return status;
}

uint8_t mcp23008_read_gpio_pin(MCP23008_HandleTypeDef *hdev, int pin)
{
	if(pin > 7)
	{
		pin = 7;
	}else if(pin < 0)
	{
		pin = 0;
	}

	mcp23008_read_gpio(hdev);
	return (hdev->gpio[0] >> pin) & 1U;
}

HAL_StatusTypeDef mcp23008_write_gpio(MCP23008_HandleTypeDef *hdev)
{
	uint8_t data[1] = {hdev->gpio[0]};
	return mcp23008_write(hdev, REGISTER_GPIO, data);
}

HAL_StatusTypeDef mcp23008_write_gpio_pin(MCP23008_HandleTypeDef *hdev,int pin, int value)
{
	HAL_StatusTypeDef status;

	uint8_t mask;
	mask = 1 << pin;
	hdev->gpio[0] = ((hdev->gpio[0] & ~mask) | (value << pin));

	status = mcp23008_write_gpio(hdev);

	return status;
}
