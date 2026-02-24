#include "lr1110_hal.h"
#include "bsp.h"

#define LR1110_BUSY_TIMEOUT  1000000UL

lr1110_hal_status_t lr1110_hal_reset(const void* context)
{
	gpio_bits_reset(RF_RST_GPIO_PORT,RF_RST_PIN);
	delay_ms(2);
	gpio_bits_set(RF_RST_GPIO_PORT,RF_RST_PIN);
  return LR1110_HAL_STATUS_OK;                                   
}


lr1110_hal_status_t lr11xx_hal_wait_on_busy(const void* context)
{
  //while(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == SET);
  uint32_t attempts = LR1110_BUSY_TIMEOUT;
	while(attempts--)
	{
		if(gpio_input_data_bit_read(RF_BUSY_GPIO_PORT,RF_BUSY_PIN) == RESET) return LR1110_HAL_STATUS_OK;
	}
	return LR1110_HAL_STATUS_ERROR;
}

void lr1110_select(const void* context)
{
	gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
}

void lr1110_deselect(const void* context)
{
	gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

uint8_t lr1110_spi_transfer(const void* context,uint8_t b)
{
	return spi1_transfer(b);
}

/* --- EOF ------------------------------------------------------------------ */