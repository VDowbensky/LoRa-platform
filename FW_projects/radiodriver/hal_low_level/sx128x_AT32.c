#include "sx128x_hal.h"
#include "bsp.h"

#define SX128X_BUSY_TIMEOUT  1000000UL

sx128x_hal_status_t sx128x_hal_reset(const void* context)
{
	delay_ms(20); 
	gpio_bits_reset(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_ms(50);   
	gpio_bits_set(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_ms(20);
  return SX128X_HAL_STATUS_OK;
}

sx128x_hal_status_t sx128x_hal_wait_on_busy(const void* context)
{
  //while(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == SET);
  uint32_t attempts = SX128X_BUSY_TIMEOUT;
	while(attempts--)
	{
		if(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == RESET) return SX128X_HAL_STATUS_OK;
	}
	return SX128X_HAL_STATUS_ERROR;
}

void sx128x_select(const void* context)
{
	gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void sx128x_deselect(const void* context)
{
	gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

uint8_t sx128x_spi_transfer(const void* context,uint8_t b)
{
	return spi1_transfer(b);
}

/* --- EOF ------------------------------------------------------------------ */