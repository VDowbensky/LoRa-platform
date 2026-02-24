#include "lr1110_hal.h"
#include "bsp.h"

lr1110_hal_status_t lr1110_hal_write(const void* context,const uint8_t* command,const uint16_t command_length,const uint8_t* data,const uint16_t data_length)
{
  uint16_t i;
//#if defined(USE_LR11XX_CRC_OVER_SPI)
//  uint8_t cmd_crc = lr11xx_hal_compute_crc(0xFF, command, command_length);
//  cmd_crc = lr11xx_hal_compute_crc(cmd_crc, data, data_length);
//#endif
  lr1110_hal_status_t retval = lr1110_hal_wait_on_busy(context);
  if(retval != LR1110_HAL_STATUS_OK) return retval;
  lr1110_select(context);
  for(i = 0; i < command_length; i++) lr1110_spi_transfer(context,command[i]);
  if (data != NULL && data_length > 0) 
  {
    for(i = 0; i < data_length; i++) lr1110_spi_transfer(context,data[i]);
  }
  lr1110_deselect(context);
  return LR1110_HAL_STATUS_OK;                                  
}

lr1110_hal_status_t lr1110_hal_read(const void* context,const uint8_t* command,const uint16_t command_length,uint8_t* data, const uint16_t data_length)
{
  uint16_t i;
    
  lr1110_hal_status_t retval = lr1110_hal_wait_on_busy(context);
  if(retval != LR1110_HAL_STATUS_OK) return retval;
  lr1110_select(context);
  for(i = 0; i < command_length; i++) lr1110_spi_transfer(context,command[i]);
  lr1110_deselect(context);
  retval = lr1110_hal_wait_on_busy(context);
  if(retval != LR1110_HAL_STATUS_OK) return retval;
  lr1110_select(context);
  for(i = 0; i < 2; i++) lr1110_spi_transfer(context,0); //drop status
  if(data != NULL && data_length > 0)
  {
    for(i = 0; i < data_length; i++) data[i] = lr1110_spi_transfer(context,0);	
  }
  lr1110_deselect(context);
  return LR1110_HAL_STATUS_OK;                                    
}

lr1110_hal_status_t lr1110_hal_direct_read(const void* context,uint8_t* data,const uint16_t data_length)
{
  uint16_t i; 
  
  lr1110_hal_status_t retval = lr1110_hal_wait_on_busy(context);
  if(retval != LR1110_HAL_STATUS_OK) return retval;
  lr1110_select(context);
  for(i = 0; i < data_length; i++) data[i] = lr1110_spi_transfer(context,0);
  lr1110_deselect(context);
  return LR1110_HAL_STATUS_OK;                                
}

lr1110_hal_status_t lr1110_hal_wakeup(const void* context)
{
	lr1110_select(context);
  delay_ms(1);
	lr1110_deselect(context);
  return lr1110_hal_wait_on_busy(context);                                  
}

/* --- EOF ------------------------------------------------------------------ */