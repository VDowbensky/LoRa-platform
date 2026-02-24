#include "llcc68_hal.h"
#include "bsp.h"

llcc68_hal_status_t llcc68_hal_write(const void* context,const uint8_t* command,const uint16_t command_length,const uint8_t* data,const uint16_t data_length)
{
  uint16_t i;

  llcc68_hal_status_t retval = sx128x_hal_wait_on_busy(context);
  if(retval != LLCC68_HAL_STATUS_OK) return retval;
  llcc68_select(context);
  for(i = 0; i < command_length; i++) llcc68_spi_transfer(context,command[i]);
  if (data != NULL && data_length > 0) 
  {
    for(i = 0; i < data_length; i++) llcc68_spi_transfer(context,data[i]);
  }
  llcc68_deselect(context);
  return LLCC68_HAL_STATUS_OK;  
}

llcc68_hal_status_t llcc68_hal_read(const void* context,const uint8_t* command,const uint16_t command_length,uint8_t* data,const uint16_t data_length)
{
  uint16_t i;
    
  llcc68_hal_status_t retval = sx128x_hal_wait_on_busy(context);
  if(retval != LLCC68_HAL_STATUS_OK) return retval;
  llcc68_select(context);
  for(i = 0; i < command_length; i++) llcc68_spi_transfer(context,command[i]);
  if(data != NULL && data_length > 0)
  {
    for(i = 0; i < data_length; i++) data[i] = llcc68_spi_transfer(context,0);	
  }
  llcc68_deselect(context);
  return LLCC68_HAL_STATUS_OK;
}

llcc68_hal_status_t llcc68_hal_wakeup(const void* context)
{
	llcc68_select(context);
  delay_ms(1); //125 us; 2 us minimum 
	llcc68_deselect(context);
  llcc68_hal_wait_on_busy(context);
  return LLCC68_HAL_STATUS_OK;
}


/* --- EOF ------------------------------------------------------------------ */
