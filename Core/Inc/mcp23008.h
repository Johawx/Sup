/**
  ******************************************************************************
  * @file    mcp23008.h
  * @brief   This file contains all the function prototypes for
  *          the mcp23008.c file
  ******************************************************************************
  * @attention
  * CURE 2022
  ******************************************************************************
  */
#include "stm32g4xx_hal.h"

// Ports
#define MCP23008_PORTA			0x00
#define MCP23008_PORTB			0x01

// Address (A0-A2)
#define MCP23008_ADDRESS_20		0x20
#define MCP23008_ADDRESS_21		0x21
#define MCP23008_ADDRESS_22		0x22
#define MCP23008_ADDRESS_23		0x23
#define MCP23008_ADDRESS_24		0x24
#define MCP23008_ADDRESS_25		0x25
#define MCP23008_ADDRESS_26		0x26
#define MCP23008_ADDRESS_27		0x27

// I/O Direction
// Default state: MCP23008_IODIR_ALL_INPUT
#define MCP23008_IODIR_ALL_OUTPUT	0x00
#define MCP23008_IODIR_ALL_INPUT	0xFF
#define MCP23008_IODIR_IO0_INPUT	0x01
#define MCP23008_IODIR_IO1_INPUT	0x02
#define MCP23008_IODIR_IO2_INPUT	0x04
#define MCP23008_IODIR_IO3_INPUT	0x08
#define MCP23008_IODIR_IO4_INPUT	0x10
#define MCP23008_IODIR_IO5_INPUT	0x20
#define MCP23008_IODIR_IO6_INPUT	0x40
#define MCP23008_IODIR_IO7_INPUT	0x80

// Input Polarity
// Default state: MCP23008_IPOL_ALL_NORMAL
#define MCP23008_IPOL_ALL_NORMAL	0x00
#define MCP23008_IPOL_ALL_INVERTED	0xFF
#define MCP23008_IPOL_IO0_INVERTED	0x01
#define MCP23008_IPOL_IO1_INVERTED	0x02
#define MCP23008_IPOL_IO2_INVERTED	0x04
#define MCP23008_IPOL_IO3_INVERTED	0x08
#define MCP23008_IPOL_IO4_INVERTED	0x10
#define MCP23008_IPOL_IO5_INVERTED	0x20
#define MCP23008_IPOL_IO6_INVERTED	0x40
#define MCP23008_IPOL_IO7_INVERTED	0x80

// Pull-Up Resistor
// Default state: MCP23008_GPPU_ALL_DISABLED
#define MCP23008_GPPU_ALL_DISABLED	0x00
#define MCP23008_GPPU_ALL_ENABLED	0xFF
#define MCP23008_GPPU_IO0_ENABLED	0x01
#define MCP23008_GPPU_IO1_ENABLED	0x02
#define MCP23008_GPPU_IO2_ENABLED	0x04
#define MCP23008_GPPU_IO3_ENABLED	0x08
#define MCP23008_GPPU_IO4_ENABLED	0x10
#define MCP23008_GPPU_IO5_ENABLED	0x20
#define MCP23008_GPPU_IO6_ENABLED	0x40
#define MCP23008_GPPU_IO7_ENABLED	0x80

typedef struct {
	I2C_HandleTypeDef*	hi2c;
	uint16_t		addr;
	uint8_t			gpio[1];
} MCP23008_HandleTypeDef;



void    			mcp23008_init(MCP23008_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr);
HAL_StatusTypeDef	mcp23008_read(MCP23008_HandleTypeDef *hdev, uint16_t reg, uint8_t *data);
HAL_StatusTypeDef	mcp23008_write(MCP23008_HandleTypeDef *hdev, uint16_t reg, uint8_t *data);
HAL_StatusTypeDef	mcp23008_iodir(MCP23008_HandleTypeDef *hdev, uint8_t iodir);
HAL_StatusTypeDef	mcp23008_ipol(MCP23008_HandleTypeDef *hdev, uint8_t ipol);
HAL_StatusTypeDef	mcp23008_ggpu(MCP23008_HandleTypeDef *hdev, uint8_t pu);
HAL_StatusTypeDef	mcp23008_read_gpio(MCP23008_HandleTypeDef *hdev);
HAL_StatusTypeDef	mcp23008_write_gpio(MCP23008_HandleTypeDef *hdev);
uint8_t				mcp23008_read_gpio_pin(MCP23008_HandleTypeDef *hdev, int pin);
HAL_StatusTypeDef	mcp23008_write_gpio_pin(MCP23008_HandleTypeDef *hdev, int pin, int value);
