#include "lr11xx_hal.h"
#include "bsp.h"

lr11xx_hal_status_t lr11xx_hal_write(const void* context,const uint8_t* command,const uint16_t command_length,const uint8_t* data,const uint16_t data_length)
{
  uint16_t i;
//#if defined(USE_LR11XX_CRC_OVER_SPI)
//  uint8_t cmd_crc = lr11xx_hal_compute_crc(0xFF, command, command_length);
//  cmd_crc = lr11xx_hal_compute_crc(cmd_crc, data, data_length);
//#endif
  lr11xx_hal_status_t retval = lr11xx_hal_wait_on_busy(context);
  if(retval != LR11XX_HAL_STATUS_OK) return retval;
  lr11xx_select(context);
  for(i = 0; i < command_length; i++) lr11xx_spi_transfer(context,command[i]);
  if (data != NULL && data_length > 0) 
  {
    for(i = 0; i < data_length; i++) lr11xx_spi_transfer(context,data[i]);
  }
  lr11xx_deselect(context);
  return LR11XX_HAL_STATUS_OK;                                
}

lr11xx_hal_status_t lr11xx_hal_read(const void* context,const uint8_t* command,const uint16_t command_length,uint8_t* data,const uint16_t data_length)
{
  uint16_t i;
    
  lr11xx_hal_status_t retval = lr11xx_hal_wait_on_busy(context);
  if(retval != LR11XX_HAL_STATUS_OK) return retval;
  lr11xx_select(context);
  for(i = 0; i < command_length; i++) lr11xx_spi_transfer(context,command[i]);
  lr11xx_deselect(context);
  retval = lr11xx_hal_wait_on_busy(context);
  if(retval != LR11XX_HAL_STATUS_OK) return retval;
  lr11xx_select(context);
	lr11xx_spi_transfer(context,0);
  if(data != NULL && data_length > 0)
  {
    for(i = 0; i < data_length; i++) data[i] = lr11xx_spi_transfer(context,0);	
  }
  lr11xx_deselect(context);
  return LR11XX_HAL_STATUS_OK;                             
}

lr11xx_hal_status_t lr11xx_hal_direct_read(const void* context,uint8_t* data,const uint16_t data_length)
{
  uint16_t i; 
  
  lr11xx_hal_status_t retval = lr11xx_hal_wait_on_busy(context);
  if(retval != LR11XX_HAL_STATUS_OK) return retval;
  lr11xx_select(context);
  for(i = 0; i < data_length; i++) data[i] = lr11xx_spi_transfer(context,0);
  lr11xx_deselect(context);
  return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void* context)
{
	lr11xx_select(context);
  delay_ms(1);
	lr11xx_deselect(context);
  return lr11xx_hal_wait_on_busy(context);
  //return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd(const void* context)
{
  lr11xx_select(context);
  for(uint16_t i = 0; i < 4; i++) lr11xx_spi_transfer(context,0);
  lr11xx_deselect(context);
  return lr11xx_hal_wait_on_busy(context);
  //return LR11XX_HAL_STATUS_OK;
}


/* --- EOF ------------------------------------------------------------------ */
