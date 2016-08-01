#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stm32f0xx_hal.h"

bool ssd1306_cmd(I2C_HandleTypeDef *hi2c, uint8_t cmd);
bool ssd1306_init(I2C_HandleTypeDef *hi2c);
bool ssd1306_clear(I2C_HandleTypeDef *hi2c);
bool ssd1306_set_pixel(I2C_HandleTypeDef *hi2c, uint8_t x, uint8_t y, bool value);
bool ssd1306_update(I2C_HandleTypeDef *hi2c);
bool ssd1306_demo(I2C_HandleTypeDef *hi2c);
