#include "sx128x_hal.h"
#include "bsp.h"

sx128x_hal_status_t sx128x_hal_write(const void* context,const uint8_t* command,const uint16_t command_length,const uint8_t* data, const uint16_t data_length)
{
  uint16_t i;

  sx128x_hal_status_t retval = sx128x_hal_wait_on_busy(context);
  if(retval != SX128X_HAL_STATUS_OK) return retval;
  sx128x_select(context);
  for(i = 0; i < command_length; i++) sx128x_spi_transfer(context,command[i]);
  if (data != NULL && data_length > 0) 
  {
    for(i = 0; i < data_length; i++) sx128x_spi_transfer(context,data[i]);
  }
  sx128x_deselect(context);
  return SX128X_HAL_STATUS_OK;    
}

sx128x_hal_status_t sx128x_hal_read(const void* context,const uint8_t* command,const uint16_t command_length,uint8_t* data,const uint16_t data_length)
{
  uint16_t i;
    
  sx128x_hal_status_t retval = sx128x_hal_wait_on_busy(context);
  if(retval != SX128X_HAL_STATUS_OK) return retval;
  sx128x_select(context);
  for(i = 0; i < command_length; i++) sx128x_spi_transfer(context,command[i]);
  if(data != NULL && data_length > 0)
  {
    for(i = 0; i < data_length; i++) data[i] = sx128x_spi_transfer(context,0);	
  }
  sx128x_deselect(context);
  return SX128X_HAL_STATUS_OK;
}

sx128x_hal_status_t sx128x_hal_wakeup(const void* context)
{
	sx128x_select(context);
  delay_ms(1); //125 us; 2 us minimum 
	sx128x_deselect(context);
  return sx128x_hal_wait_on_busy(context);
  //return SX128X_HAL_STATUS_OK;
}


/* --- EOF ------------------------------------------------------------------ */
