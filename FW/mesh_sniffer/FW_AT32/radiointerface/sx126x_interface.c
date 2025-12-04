#include "sx126x_interface.h"

void SX126X_reset(void)
{
	gpio_bits_reset(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_us(50);
	gpio_bits_set(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_us(50);
}

void SX126X_Wakeup(void)
{
	SX126X_select();
	delay_us(20);
	SX126X_spi_transfer(SX126X_GET_STATUS);
	SX126X_spi_transfer(0x00);
	SX126X_deselect();
	// Wait for chip to be ready.
	SX126X_checkBusy();
}

bool SX126X_checkBusy(void)
{ 
  uint32_t attempts = SX126X_BUSY_TIMEOUT;
	while(attempts--)
	{
		if(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == RESET) return true;
	}
	return false;
}

void SX126X_select(void)
{
	gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void SX126X_deselect(void)
{
	gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void SX126X_writeCmd(uint8_t cmd, uint8_t *buffer, uint16_t size)
{
  uint8_t i;
  SX126X_checkBusy();
  SX126X_select();
  SX126X_spi_transfer(cmd);
  for(i = 0;i < size;i++) SX126X_spi_transfer(buffer[i]);
  SX126X_deselect();
  if(cmd != SX126X_SET_SLEEP) SX126X_checkBusy();
}

void SX126X_readCmd(uint8_t cmd, uint8_t *buffer, uint16_t size)
{
  uint8_t i;
  
  SX126X_checkBusy();
  SX126X_select();
  SX126X_spi_transfer(cmd);
  for(i = 0;i < size;i++) buffer[i] = SX126X_spi_transfer(0xff);
  SX126X_deselect();
  SX126X_checkBusy();
}

void SX126X_writeRegs(uint16_t reg, uint8_t *buffer, uint16_t size)
{
  uint8_t addr_l,addr_h;
  uint8_t i;
  
  addr_l = reg & 0xff;
  addr_h = reg >> 8;
  SX126X_checkBusy();
  SX126X_select();
  SX126X_spi_transfer(SX126X_WRITE_REGISTER);
  SX126X_spi_transfer(addr_h);//MSB
  SX126X_spi_transfer(addr_l);//LSB
  for(i = 0;i < size; i++) SX126X_spi_transfer(buffer[i]);
  SX126X_deselect();
  SX126X_checkBusy();
}

void SX126X_readRegs(uint16_t reg, uint8_t *buffer, uint16_t size)
{
  uint16_t i;
  uint8_t addr_l,addr_h;
  
  addr_h = reg >> 8;
  addr_l = reg & 0x00FF;
  
  SX126X_checkBusy();
  SX126X_select();
  SX126X_spi_transfer(SX126X_READ_REGISTER);
  SX126X_spi_transfer(addr_h);//MSB
  SX126X_spi_transfer(addr_l);//LSB
  SX126X_spi_transfer(0xff);
  for(i = 0; i < size; i++) buffer[i] = SX126X_spi_transfer(0xff);
  SX126X_deselect();
  SX126X_checkBusy();
}

void SX126X_writeReg(uint16_t reg, uint8_t value)
{
  SX126X_writeRegs(reg,&value,1);
}

uint8_t SX126X_readReg(uint16_t reg)
{
  uint8_t value;
  
  SX126X_readRegs(reg,&value,1);
  return value;
}

void SX126X_writeBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
  uint16_t i;
  
  SX126X_checkBusy();
  SX126X_select();
  SX126X_spi_transfer(SX126X_WRITE_BUFFER);
  SX126X_spi_transfer(offset);
  for(i = 0; i < length;i++) SX126X_spi_transfer(data[i]);
  SX126X_deselect();
  SX126X_checkBusy();
}

void SX126X_readBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
  uint8_t i;
  
  SX126X_checkBusy();
  SX126X_select();
  SX126X_spi_transfer(SX126X_READ_BUFFER);
  SX126X_spi_transfer(offset);
  SX126X_spi_transfer(0xff);
  for(i = 0;i < length; i++) data[i] = SX126X_spi_transfer(0xff);
  SX126X_deselect();
  SX126X_checkBusy();
}

void SX126X_rfsw_tx(void)
{

}

void SX126X_rfsw_rx(void)
{

}

uint8_t SX126X_spi_transfer(uint8_t b)
{
	return spi1_transfer(b);
}




