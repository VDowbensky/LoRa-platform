#include "llcc68_hal.h"
#include "bsp.h"

define SX126X_BUSY_TIMEOUT  1000000UL

llcc68_hal_status_t llcc68_hal_reset(const void* context)
{
	gpio_bits_reset(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_us(50);
	gpio_bits_set(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_us(50);
  return LLCC68_HAL_STATUS_OK; 
}

llcc68_hal_status_t llcc68_hal_wait_on_busy(const void* context)
{
  //while(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == SET);
  uint32_t attempts = LLCC68_BUSY_TIMEOUT;
	while(attempts--)
	{
		if(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == RESET) return LLCC68_HAL_STATUS_OK;
	}
	return LLCC68_HAL_STATUS_ERROR;
}

void llcc68_select(const void* context)
{
	gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void llcc68_deselect(const void* context)
{
	gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

uint8_t llcc68_spi_transfer(const void* context,uint8_t b)
{
	return spi1_transfer(b);
}

/* --- EOF ------------------------------------------------------------------ */