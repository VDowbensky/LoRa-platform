#include "lr20xx_hal.h"
#include "bsp.h"

#define LR20XX_BUSY_TIMEOUT  1000000UL

lr20xx_hal_status_t lr20xx_hal_reset(const void* context)
{	
  gpio_bits_reset(RF_RST_GPIO_PORT,RF_RST_PIN);
  delay_ms(2);
  gpio_bits_set(RF_RST_GPIO_PORT,RF_RST_PIN);
  return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_wait_on_busy(const void* context)
{
  //while(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == SET);
  uint32_t attempts = LR20XX_BUSY_TIMEOUT;
	while(attempts--)
	{
		if(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == RESET) return LR20XX_HAL_STATUS_OK;
	}
	return LR20XX_HAL_STATUS_ERROR;
}

void lr20xx_select(const void* context)
{
	gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void lr20xx_deselect(const void* context)
{
	gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

uint8_t lr20xx_spi_transfer(const void* context,uint8_t b)
{
	return spi1_transfer(b);
}

/* --- EOF ------------------------------------------------------------------ */