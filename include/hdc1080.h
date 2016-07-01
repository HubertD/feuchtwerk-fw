#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stm32f0xx_hal.h"

enum {
	HDC1080_CFG_HRES_8BIT = 0x0200,
	HDC1080_CFG_HRES_11BIT = 0x0100,
	HDC1080_CFG_HRES_14BIT = 0x0000,
	HDC1080_CFG_TRES_11BIT = 0x0400,
	HDC1080_CFG_TRES_14BIT = 0x0000,
	HDC1080_CFG_BAT_STATUS = 0x0800,
	HDC1080_CFG_MODE_ONE   = 0x0000,
	HDC1080_CFG_MODE_BOTH  = 0x1000,
	HDC1080_CFG_HEATER     = 0x2000,
	HDC1080_CFG_RESET      = 0x8000,
};

enum {
	HDC1080_REG_TEMPERATURE  = 0x00,
	HDC1080_REG_HUMIDITY     = 0x01,
	HDC1080_REG_CONFIG       = 0x02,
	HDC1080_REG_SERIAL_1     = 0xFB,
	HDC1080_REG_SERIAL_2     = 0xFC,
	HDC1080_REG_SERIAL_3     = 0xFD,
	HDC1080_REG_MANUFACTURER = 0xFE,
	HDC1080_REG_DEVICE_ID    = 0xFF,
};

bool hdc1080_read_register(I2C_HandleTypeDef *hi2c, uint8_t register_id, uint16_t *value);
bool hdc1080_write_register(I2C_HandleTypeDef *hi2c, uint8_t register_id, uint16_t value);
bool hdc1080_soft_reset(I2C_HandleTypeDef *hi2c);
bool hdc1080_make_measurement(I2C_HandleTypeDef *hi2c, uint8_t reg_id, uint16_t *value);
