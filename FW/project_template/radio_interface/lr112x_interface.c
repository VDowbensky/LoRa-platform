#include "lr112x_interface.h"

#define LR112X_NOP    0

void LR112X_reset(void)
{
	gpio_bits_reset(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_ms(2);
	gpio_bits_set(RF_RST_GPIO_PORT,RF_RST_PIN);
	//delay_ms(10);
}

void LR112X_Wakeup(void)
{
	LR112X_select();
  delay_ms(1);
  // Wait for chip to be ready.
	LR112X_deselect();
  LR112X_checkBusy();
}

bool LR112X_checkBusy(void)
{ 
  uint32_t attempts = LR112X_BUSY_TIMEOUT;
	while(attempts--)
	{
		if(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == RESET) return true;
	}
	return false;
}

void LR112X_select(void)
{
	gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void LR112X_deselect(void)
{
	gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void LR112X_writeCmd(uint16_t cmd, uint8_t *buffer, uint16_t size)
{
  uint8_t i;
  
  LR112X_checkBusy();
  LR112X_select();
  LR112X_spi_transfer(cmd >> 8);
  LR112X_spi_transfer(cmd & 0xff); //big endian
  for(i = 0;i < size;i++) LR112X_spi_transfer(buffer[i]);
  LR112X_deselect();
  if(cmd != LR112X_SET_SLEEP) LR112X_checkBusy();
}

void LR112X_readCmd(uint16_t cmd, uint8_t *args, uint8_t argslen, uint8_t *buffer, uint16_t size)
{
  uint8_t i;
  
  LR112X_checkBusy();
  LR112X_select();
  LR112X_spi_transfer(cmd >> 8);
  LR112X_spi_transfer(cmd & 0xff); //big endian
  for(i = 0; i < argslen; i++) LR112X_spi_transfer(args[i]);
  LR112X_deselect();
  LR112X_checkBusy();
  LR112X_select();
  for(i = 0;i < size;i++) buffer[i] = LR112X_spi_transfer(LR112X_NOP);
  LR112X_deselect();
  LR112X_checkBusy();
}

void LR112X_WriteRegMem32(uint32_t addr, uint8_t *buffer, uint16_t size)
{
  uint8_t buf[4];
  uint16_t i;
  
  buf[0] = (addr >> 24) & 0xff;
  buf[1] = (addr >> 16) & 0xff;
  buf[2] = (addr >> 8) & 0xff;
  buf[3] = addr & 0xff;
  LR112X_checkBusy();
  LR112X_select();
  LR112X_spi_transfer(LR112X_WRITE_REG_MEM32 >> 8);
  LR112X_spi_transfer(LR112X_WRITE_REG_MEM32 & 0xff);
  for(i = 0; i < 4; i++) LR112X_spi_transfer(buf[i]);
  for(i = 0; i < size*4; i++) LR112X_spi_transfer(buffer[i]);
  LR112X_deselect();
  LR112X_checkBusy();
}

void LR112X_ReadRegMem32(uint32_t addr, uint8_t *buffer, uint16_t size)
{
  uint16_t i;
  uint8_t buf[5];
  
  buf[0] = (addr >> 24) & 0xff;
  buf[1] = (addr >> 16) & 0xff;
  buf[2] = (addr >> 8) & 0xff;
  buf[3] = addr & 0xff;
  buf[4] = size;
  LR112X_checkBusy();
  LR112X_select();
  LR112X_spi_transfer(LR112X_READ_REG_MEM32 >> 8);
  LR112X_spi_transfer(LR112X_READ_REG_MEM32 & 0xff);
  for(i = 0; i < 5; i++) LR112X_spi_transfer(buf[i]);
	LR112X_deselect();
  LR112X_checkBusy();
	LR112X_select();
  LR112X_spi_transfer(LR112X_NOP); //first byte (Stat1) discarded
  for(i = 0; i < size*4; i++) buffer[i] = LR112X_spi_transfer(LR112X_NOP);
  LR112X_deselect();
  LR112X_checkBusy();
}

void LR112X_writeRegMemMask32(uint32_t addr, uint32_t mask, uint32_t regdata)
{
  uint8_t buf[12];
  
  buf[0] = (addr >> 24) & 0xff;
  buf[1] = (addr >> 16) & 0xff;
  buf[2] = (addr >> 8) & 0xff;
  buf[3] = addr & 0xff;
  buf[4] = (mask >> 24) & 0xff;
  buf[5] = (mask >> 16) & 0xff;
  buf[6] = (mask >> 8) & 0xff;
  buf[7] = mask & 0xff;
  buf[8] = (regdata >> 24) & 0xff;
  buf[9] = (regdata >> 16) & 0xff;
  buf[10] = (regdata >> 8) & 0xff;
  buf[11] = regdata & 0xff;
  LR112X_writeCmd(LR112X_WRITE_REG_MEM_MASK32,buf,12);
}

void LR112X_writeBuffer8(uint8_t *data, uint8_t len)
{
  uint8_t i;
  
  LR112X_checkBusy();
  LR112X_select();
  LR112X_spi_transfer(LR112X_WRITE_BUFFER8 >> 8);
  LR112X_spi_transfer(LR112X_WRITE_BUFFER8 & 0xff);
  for(i = 0; i < len;i++) LR112X_spi_transfer(data[i]);
  LR112X_deselect();
  LR112X_checkBusy();
}

void LR112X_readBuffer8(uint8_t offset, uint8_t *data, uint8_t len)
{
  uint8_t i;
  
  LR112X_checkBusy();
  LR112X_select();
  LR112X_spi_transfer(LR112X_READ_BUFFER8 >> 8);
  LR112X_spi_transfer(LR112X_READ_BUFFER8 & 0xff);
  LR112X_spi_transfer(offset);
  LR112X_spi_transfer(len);
  LR112X_checkBusy();
  LR112X_spi_transfer(LR112X_NOP); //first byte (Stat1) discarded
  for(i = 0;i < len; i++) data[i] = LR112X_spi_transfer(LR112X_NOP);
  LR112X_deselect();
  LR112X_checkBusy();
}

uint8_t LR112X_spi_transfer(uint8_t b)
{
	return spi1_transfer(b);
}

void LR112X_rfsw_tx(void)
{

}

void LR112X_rfsw_rx(void)
{

}
