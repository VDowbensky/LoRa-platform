#include "sx126x_hal.h"
#include "bsp.h"

#define SX126X_BUSY_TIMEOUT  1000000UL

sx126x_hal_status_t sx126x_hal_reset(const void* context)
{
	subghz_reset();
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wait_on_busy(const void* context)
{
  uint32_t attempts = SX126X_BUSY_TIMEOUT;
	while(attempts--)
	{
		if(!(LL_PWR_IsActiveFlag_RFBUSYS())) return SX126X_HAL_STATUS_OK;
	}
	return SX126X_HAL_STATUS_ERROR;
}

void sx126x_select(const void* context)
{
	subghz_select();
}

void sx126x_deselect(const void* context)
{
	subghz_deselect();
}

uint8_t sx126x_spi_transfer(const void* context,uint8_t b)
{
	return subghz_spi_transfer(b);
}

/* void SX126X_rfsw_tx(void)
{
  subghz_rfsw_tx();
}

void SX126X_rfsw_rx(void)
{
  subghz_rfsw_rx();
} */

/* --- EOF ------------------------------------------------------------------ */