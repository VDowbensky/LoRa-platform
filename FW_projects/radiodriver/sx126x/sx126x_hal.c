#include "sx126x_hal.h"
#include "bsp.h"

sx126x_hal_status_t sx126x_hal_write(const void* context,const uint8_t* command,const uint16_t command_length,const uint8_t* data, const uint16_t data_length)
{
  uint16_t i;

  sx126x_hal_status_t retval = sx126x_hal_wait_on_busy(context);
  if(retval != SX126X_HAL_STATUS_OK) return retval;
  sx126x_select(context);
  for(i = 0; i < command_length; i++) sx126x_spi_transfer(context,command[i]);
  if (data != NULL && data_length > 0) 
  {
    for(i = 0; i < data_length; i++) sx126x_spi_transfer(context,data[i]);
  }
  sx126x_deselect(context);
  return SX126X_HAL_STATUS_OK;                                
}
                                      
sx126x_hal_status_t sx126x_hal_read(const void* context,const uint8_t* command,const uint16_t command_length,uint8_t* data, const uint16_t data_length)
{
  uint16_t i;
    
  sx126x_hal_status_t retval = sx126x_hal_wait_on_busy(context);
  if(retval != SX126X_HAL_STATUS_OK) return retval;
  sx126x_select(context);
  for(i = 0; i < command_length; i++) sx126x_spi_transfer(context,command[i]);
  if(data != NULL && data_length > 0)
  {
    for(i = 0; i < data_length; i++) data[i] = sx126x_spi_transfer(context,0);	
  }
  sx126x_deselect(context);
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context)
{
	sx126x_select(context);
  delay_ms(1); //125 us; 2 us minimum 
	sx126x_deselect(context);
  return sx126x_hal_wait_on_busy(context);
  //return SX126X_HAL_STATUS_OK;
}

/* --- EOF ------------------------------------------------------------------ */
