#include "pan_interface.h"

void pan_select(void); 
uint8_t pan_spi_transfer(uint8_t b);
void pan_deselect(void);

/**
 * @brief Get the number of trailing zeros in a byte
 * @param Value The value to check
 * @return The number of trailing zeros
 */
uint8_t __ctz(uint8_t Value)
{
  int i;

  for (i = 0; i < 8; ++i)
  {
    if ((Value >> i) & 1) return (uint8_t)i;
  }
  return 0;
}

/**
* @brief Read a single byte from the specified register
* @param Addr Register address to be read
* @return uint8_t Value read from the register
* @note The pan_select();, pan_deselect();, pan_spi_transfer()
* and pan_spi_transfer(0) functions of this function need to be modified according to the actual hardware implementation.
*/
uint8_t PAN_ReadReg(uint8_t Addr)
{
  uint8_t Temp;

	pan_select(); /* Pull the chip select signal low to start SPI transmission */
	pan_spi_transfer((Addr << 1) & 0xFE); /* Bit7:0 is the address, Bit0 is the read/write bit, Bit0=0 indicates a read operation */
	Temp = pan_spi_transfer(0); /* Read the register value */
	pan_deselect(); /* Pull the chip select signal high to end SPI transmission */
  return Temp;
}

/**
* @brief Write a single byte to the specified register
* @param Addr Register address to be written
* @param Value Single byte data to be written to the register
* @return int8_t Returns the operation result
* - PAN_OK: Operation successful
* - PAN_FAIL: Operation failed
* @note The pan_select();, pan_deselect();, pan_spi_transfer()
* and pan_spi_transfer(0) functions of this function need to be modified according to the actual hardware implementation.
*/
int8_t PAN_WriteReg(uint8_t Addr, uint8_t Value)
{
	pan_select(); /* Pull the chip select signal low to start SPI transfer */
	pan_spi_transfer((Addr << 1) | 0x01); /* Bit 7:1 is the address, Bit 0 is the read/write bit, Bit 0 = 1 indicates a write operation */
	pan_spi_transfer(Value); /* Write the register value */
	pan_deselect(); /* Pull the chip select signal high to end SPI transfer */
	return PAN_OK;
}

/**
* @brief Reads multiple bytes continuously from the specified register.
* @param Addr Register address to be read
* @param Buffer Buffer pointer to the buffer storing the read data
* @param Size Number of bytes to be read
* @note The pan_select();, pan_deselect();, pan_spi_transfer()
*, and pan_spi_transfer(0) functions in this function need to be modified based on the actual hardware implementation.
*/
void PAN_ReadRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
	unsigned char i;
	pan_select(); /* Pull the chip select signal low to start SPI transmission */
	pan_spi_transfer((Addr << 1) & 0xFE); /* Bits 7:0 are the address, Bit 0 is the read/write bit, Bit 0 = 0 indicates a read operation */
	for (i = 0; i < Size; i++) Buffer[i] = pan_spi_transfer(0); /* Read the register value */
	pan_deselect(); /* Pull the chip select signal high to end SPI transmission */
}

/**
* @brief Writes multiple bytes continuously to a specified register area.
* @param Addr: Starting address of the register area to be written.
* @param Buffer: Pointer to the buffer to be written to the register.
* @param Size: Number of bytes to be written.
* @note: The pan_select();, pan_deselect();, and pan_spi_transfer() functions in this function need to be modified based on the actual hardware implementation. */
void PAN_WriteRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size)
{
	uint8_t i;
	pan_select(); /** Pull the chip select signal low to start SPI transmission */
	pan_spi_transfer((Addr << 1) | 0x01); /** Bit7:1 is the address, Bit0 is the read/write bit, Bit0=1 indicates a write operation */
	for (i = 0; i < Size; i++) pan_spi_transfer(Buffer[i]); /** Write register value */
	pan_deselect(); /** Pull the chip select signal high to end SPI transmission */
}

void PAN_Reset(void) //to be implemented
{
	gpio_bits_reset(RF_RST_GPIO_PORT,RF_RST_PIN);
	PAN_DelayMs(1);     /* Ensure that the actual delay is above 100us */
	gpio_bits_set(RF_RST_GPIO_PORT,RF_RST_PIN);
	PAN_DelayMs(1);     /* Ensure that the actual delay is above 100us */
}

void PAN_DelayUs(uint32_t us) //to be implemented
{
  
}

void PAN_DelayMs(uint32_t ms) //to be implemented
{
  delay_ms(ms);
}

void pan_select(void)
{
	gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
}

uint8_t pan_spi_transfer(uint8_t b)
{
	return spi1_transfer(b);
}

void pan_deselect(void)
{
	gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

