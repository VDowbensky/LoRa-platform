#include "ssd1306_interface.h"
uint8_t buf[2];
char strbuffer[64];

#include "i2c.h"

void SSD1306_interface_init(void)
{
	i2c1_init();
}

//void SSD1306_Select(void)
//{
//	//nothing to do in I2C mode
//}

//void SSD1306_Deselect(void)
//{
//	//nothing to do in I2C mode
//}

//void SSD1306_WriteByte(uint8_t b)
//{
//	//not used in I2C mode
//}

//void SSD1306_Command(bool cmd)
//{
//	//nothing to do in I2C mode
//}

//void SSD1306_Reset(void)
//{
//	//nothing to do in I2C mode
//}

//void SSD1306_WR_Byte(uint8_t dat,uint8_t cmd)
//{
//	//uint8_t buf[2];
//	if(cmd) buf[0] = 0;
//	else buf[0] = 0x40;
//	buf[1] = dat;
//	i2c1_write(SSD1306_ADDR, buf, 2);
//}

void SSD1306_WriteCommand(uint8_t cmd)
{
	i2c1_writessd1306(0x00, &cmd, 1);
}

void SSD1306_WriteData(uint8_t *data,uint16_t len)
{
	i2c1_writessd1306(0x40, data, len);
}

