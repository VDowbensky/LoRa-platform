#include "ssd1306_interface.h"

char strbuffer[64];

#if SSD1306_INTERFACE_SOFT_SPI

void SSD1306_interface_init(void)
{
	gpio_init(SSD1306_CS_PORT, SSD1306_CS_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
	gpio_init(SSD1306_SCK_PORT, SSD1306_SCK_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	gpio_init(SSD1306_MOSI_PORT, SSD1306_MOSI_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	gpio_init(SSD1306_RST_PORT, SSD1306_RST_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
	gpio_init(SSD1306_DC_PORT, SSD1306_DC_PIN, GPIO_MODE_OUTPUT_PP_LOW);
}

void SSD1306_Select(void)
{
	gpio_write(SSD1306_CS_PORT, SSD1306_CS_PIN,GPIO_LEVEL_LOW);
}

void SSD1306_Deselect(void)
{
	gpio_write(SSD1306_CS_PORT, SSD1306_CS_PIN,GPIO_LEVEL_HIGH);
}

void SSD1306_WriteByte(uint8_t b)
{
	uint8_t i;
	
	for(i = 8;i > 0;i--)
	{
		if(b & 0x80) gpio_write(SSD1306_MOSI_PORT, SSD1306_MOSI_PIN,GPIO_LEVEL_HIGH);
		else gpio_write(SSD1306_MOSI_PORT, SSD1306_MOSI_PIN,GPIO_LEVEL_LOW);
		gpio_write(SSD1306_SCK_PORT, SSD1306_SCK_PIN,GPIO_LEVEL_LOW);
		gpio_write(SSD1306_SCK_PORT, SSD1306_SCK_PIN,GPIO_LEVEL_HIGH);
    b <<= 1; 
	}
}

void SSD1306_Command(bool cmd)
{
	if(cmd) gpio_write(SSD1306_DC_PORT, SSD1306_DC_PIN,GPIO_LEVEL_HIGH);
	else gpio_write(SSD1306_DC_PORT, SSD1306_DC_PIN,GPIO_LEVEL_LOW);
}

void SSD1306_Reset(void)
{
	gpio_write(SSD1306_RST_PORT, SSD1306_RST_PIN,GPIO_LEVEL_HIGH);
	delay_ms(100);
	gpio_write(SSD1306_RST_PORT, SSD1306_RST_PIN,GPIO_LEVEL_LOW);
	delay_ms(100);
	gpio_write(SSD1306_RST_PORT, SSD1306_RST_PIN,GPIO_LEVEL_HIGH);	
}

void SSD1306_WR_Byte(uint8_t dat,uint8_t cmd)
{
	if(cmd) SSD1306_Command(true);
	else SSD1306_Command(false);
	SSD1306_Select();
	SSD1306_WriteByte(dat);
	SSD1306_Deselect();
}

#elif SSD1306_INTERFACE_HARD_SPI

#include "wk_spi.h"

void SSD1306_interface_init(void)
{
	//spi initialized before, initialize GPIO only
  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
  /* gpio input config */
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = SSD1306_CS_PIN;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MAXIMUM;
  gpio_init(SSD1306_CS_PORT, &gpio_init_struct);
	gpio_bits_set(SSD1306_CS_PORT,SSD1306_CS_PIN);
	gpio_init_struct.gpio_pins = SSD1306_RST_PIN;
	gpio_init(SSD1306_RST_PORT, &gpio_init_struct);
	gpio_bits_set(SSD1306_RST_PORT,SSD1306_RST_PIN);
	gpio_init_struct.gpio_pins = SSD1306_DC_PIN;
	gpio_init(SSD1306_DC_PORT, &gpio_init_struct);
	gpio_bits_reset(SSD1306_DC_PORT,SSD1306_DC_PIN);
}

void SSD1306_Select(void)
{
	gpio_bits_reset(SSD1306_CS_PORT,SSD1306_CS_PIN);
}

void SSD1306_Deselect(void)
{
	gpio_bits_set(SSD1306_CS_PORT,SSD1306_CS_PIN);
}

void SSD1306_WriteByte(uint8_t b)
{
	spi1_transfer(b);
}

void SSD1306_Command(bool cmd)
{
	if(cmd) gpio_bits_set(SSD1306_DC_PORT,SSD1306_DC_PIN);
	else gpio_bits_reset(SSD1306_DC_PORT,SSD1306_DC_PIN);
}

void SSD1306_Reset(void)
{
	gpio_bits_set(SSD1306_RST_PORT,SSD1306_RST_PIN);
	delay_ms(100);
	gpio_bits_reset(SSD1306_RST_PORT,SSD1306_RST_PIN);
	delay_ms(100);
	gpio_bits_set(SSD1306_RST_PORT,SSD1306_RST_PIN);
}

void SSD1306_WR_Byte(uint8_t dat,uint8_t cmd)
{
	if(cmd) SSD1306_Command(true);
	else SSD1306_Command(false);
	SSD1306_Select();
	SSD1306_WriteByte(dat);
	SSD1306_Deselect();
}
#else //I2c

#include "i2c.h"

void SSD1306_interface_init(void)
{
	i2c0_init();
}

void SSD1306_Select(void)
{
	//nothing to do in I2C mode
}

void SSD1306_Deselect(void)
{
	//nothing to do in I2C mode
}

void SSD1306_WriteByte(uint8_t b)
{
	//not used in I2C mode
}

void SSD1306_Command(bool cmd)
{
	//nothing to do in I2C mode
}

void SSD1306_Reset(void)
{
	//nothing to do in I2C mode
}

void SSD1306_WR_Byte(uint8_t dat,uint8_t cmd)
{
	uint8_t buf[2];
	if(cmd) buf[0] = 0;
	else buf[0] = 0x40;
	buf[1] = dat;
	i2c0_write(SSD1306_ADDR, buf, 2);
}

#endif
