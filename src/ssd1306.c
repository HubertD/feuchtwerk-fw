#include "ssd1306.h"

static uint8_t ssd1306_buf[1024];

bool ssd1306_cmd(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
	uint8_t i2caddr = 0x78; //(0x78<<1);
	uint8_t buf[] = { 0x00, cmd };
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, i2caddr, buf, sizeof(buf), 100);
	return result == HAL_OK;
}

bool ssd1306_cmd_data(I2C_HandleTypeDef *hi2c, uint8_t cmd, uint8_t data)
{
	uint8_t i2caddr = 0x78;
	uint8_t buf[] = { 0x80, cmd, data };
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, i2caddr, buf, sizeof(buf), 100);
	return result == HAL_OK;
}

bool ssd1306_init(I2C_HandleTypeDef *hi2c)
{
	return ssd1306_cmd(hi2c, 0xAE) //display off
	    && ssd1306_cmd(hi2c, 0x20) //Set Memory Addressing Mode
	    && ssd1306_cmd(hi2c, 0x00) //00,Horizontal Addressing Mode01,Vertical Addressing Mode10,Page Addressing Mode (RESET)11,Invalid
	    && ssd1306_cmd(hi2c, 0xB0) //Set Page Start Address for Page Addressing Mode,0-7
	    && ssd1306_cmd(hi2c, 0xC8) //Set COM Output Scan Direction
	    && ssd1306_cmd(hi2c, 0x00) //---set low column address
	    && ssd1306_cmd(hi2c, 0x10) //---set high column address
	    && ssd1306_cmd(hi2c, 0x40) //--set start line address
	    && ssd1306_cmd(hi2c, 0x81) //--set contrast control register
	    && ssd1306_cmd(hi2c, 0xFF)
	    && ssd1306_cmd(hi2c, 0xA1) //--set segment re-map 0 to 127
	    && ssd1306_cmd(hi2c, 0xA6) //--set normal display
	    && ssd1306_cmd(hi2c, 0xA8) //--set multiplex ratio(1 to 64)
	    && ssd1306_cmd(hi2c, 0x3F) //
	    && ssd1306_cmd(hi2c, 0xA4) //0xa4,Output follows RAM content0xa5,Output ignores RAM content
	    && ssd1306_cmd(hi2c, 0xD3) //-set display offset
	    && ssd1306_cmd(hi2c, 0x00) //-not offset
	    && ssd1306_cmd(hi2c, 0xD5) //--set display clock divide ratio/oscillator frequency
	    && ssd1306_cmd(hi2c, 0xF0) //--set divide ratio
	    && ssd1306_cmd(hi2c, 0xD9) //--set pre-charge period
	    && ssd1306_cmd(hi2c, 0x22) //
	    && ssd1306_cmd(hi2c, 0xDA) //--set com pins hardware configuration
	    && ssd1306_cmd(hi2c, 0x12)
	    && ssd1306_cmd(hi2c, 0xDB) //--set vcomh
	    && ssd1306_cmd(hi2c, 0x20) //0x20,0.77xVcc
	    && ssd1306_cmd(hi2c, 0x8D) //--set DC-DC enable
	    && ssd1306_cmd(hi2c, 0x14) //
	    && ssd1306_cmd(hi2c, 0xAF); //--turn on SSD1306 panel
}

bool ssd1306_clear(I2C_HandleTypeDef *hi2c)
{
	memset(ssd1306_buf, 0x00, sizeof(ssd1306_buf));
}

bool ssd1306_set_pixel(I2C_HandleTypeDef *hi2c, uint8_t x, uint8_t y, bool value)
{
	int p = ((y/8)*128+x);
	if (value) {
		ssd1306_buf[p] |= (1<<(y%8));
	} else {
		ssd1306_buf[p] &= ~(1<<(y%8));
	}
}

bool ssd1306_update(I2C_HandleTypeDef *hi2c)
{
	uint8_t buf[65];
	bool result = true;

	ssd1306_cmd(hi2c, 0x20);
	ssd1306_cmd(hi2c, 0x00);

	for (int i=0; i<sizeof(ssd1306_buf); i+=sizeof(buf)-1) {
		buf[0] = 0x40;
		memcpy(&buf[1], &ssd1306_buf[i], sizeof(buf)-1);
		result &= HAL_I2C_Master_Transmit(hi2c, 0x78, buf, sizeof(buf), 200) == HAL_OK;
	}

	return result;
}

bool ssd1306_demo(I2C_HandleTypeDef *hi2c)
{
	while (42) {
		ssd1306_clear(hi2c);
		for (int y=0; y<64; y++) {
			for (int x=0; x<128; x++) {
				ssd1306_set_pixel(hi2c, x, y, true);
			}
			ssd1306_update(hi2c);
		}
	}
}
