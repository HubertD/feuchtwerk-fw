#include "hdc1080.h"

bool hdc1080_read_register(I2C_HandleTypeDef *hi2c, uint8_t register_id, uint16_t *value)
{
	uint8_t i2caddr = (0x40<<1);
	HAL_StatusTypeDef result;

	result = HAL_I2C_Master_Transmit(hi2c, i2caddr, &register_id, 1, 100);
	if (result != HAL_OK) {
		return false;
	}

	result = HAL_I2C_Master_Receive(hi2c, i2caddr, (uint8_t*)value, 2, 100);

	return result == HAL_OK;
}

bool hdc1080_write_register(I2C_HandleTypeDef *hi2c, uint8_t register_id, uint16_t value)
{
	HAL_StatusTypeDef result;
	uint8_t i2caddr = (0x40<<1);
	uint8_t txdata[] = {register_id, (value>>0) & 0xFF, (value>>8) & 0xFF };

	result = HAL_I2C_Master_Transmit(hi2c, i2caddr, txdata, sizeof(txdata), 100);
	return result == HAL_OK;
}

bool hdc1080_soft_reset(I2C_HandleTypeDef *hi2c)
{
	return hdc1080_write_register(hi2c, 0x02, 0x8000);
}

bool hdc1080_make_measurement(I2C_HandleTypeDef *hi2c, uint8_t reg_id, uint16_t *value) {
	HAL_StatusTypeDef result;
	uint8_t i2caddr = (0x40<<1);

	result = HAL_I2C_Master_Transmit(hi2c, i2caddr, &reg_id, 1, 5);
	if (result != HAL_OK) {
		return false;
	}
	HAL_Delay(10);

	result = HAL_I2C_Master_Receive(hi2c, i2caddr, value, 2, 100);
	return result == HAL_OK;
}

