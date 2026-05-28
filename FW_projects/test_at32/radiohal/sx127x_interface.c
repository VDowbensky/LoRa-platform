#include "sx127x_interface.h"

void SX127x_select(void);
void SX127x_deselect(void);
uint8_t SX127x_spi_transfer(uint8_t b);

void SX127x_reset(void)
{
	gpio_bits_reset(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_us(10);
	gpio_bits_set(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_us(20);
}


void SX127x_select(void)
{
	gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void SX127x_deselect(void)
{
	gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

uint8_t SX127x_spi_transfer(uint8_t b)
{
	return spi1_transfer(b);
}


void SX127x_write_regs(uint8_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i;
	
	SX127x_deselect();
	SX127x_spi_transfer(reg & ~0x80);
	for(i = 0; i < len; i++) buf[i] = SX127x_spi_transfer(0);
	SX127x_deselect();
}

void SX127x_write_reg(uint8_t reg,uint8_t val)
{
	SX127x_write_regs(reg,&val,1);
}

void SX127x_read_regs(uint8_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i;
	
	SX127x_select();
	SX127x_spi_transfer(reg | 0x80);
	for(i = 0; i < len; i++) SX127x_spi_transfer(buf[i]);
	SX127x_deselect();
}

uint8_t SX127x_read_reg(uint8_t reg)
{
	uint8_t b;
	SX127x_read_regs(reg,&b,1);
	return b;
}


