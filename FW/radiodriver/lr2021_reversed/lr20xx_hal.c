
lr20xx_hal_status_t lr20xx_hal_reset(void *context)
{
  //set RST low
  //delay 1 ms
  //set RST high
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t lr20xx_hal_wakeup(void *context)
{
  //NSS low
  //delay 1 ms
  //NSS gigh
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t lr20xx_hal_write(void *context,uint8_t *command,uint16_t command_length,uint8_t *data,uint16_t data_length)
{
  lr20xx_hal_wait_on_busy(context);
  //NSS low
  //send 2 command bytes by SPI
  //smtc_hal_mcu_spi_rw_buffer(*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),command,NULL,command_length);
  //send data_length parameters bytes by SPI
  //smtc_hal_mcu_spi_rw_buffer(*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),data,NULL,data_length);
  //NSS high
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t lr20xx_hal_read(void *context,uint8_t *command,uint16_t command_length,uint8_t *data,uint16_t data_length)
{
  uint8_t dummy_byte_rx [2];
  uint8_t dummy_byte [2];
  
  dummy_byte[0] = 0;
  dummy_byte[1] = 0;
  dummy_byte_rx[0] = 0;
  dummy_byte_rx[1] = 0;
  lr20xx_hal_wait_on_busy(context);
  //NSS low
  smtc_hal_mcu_spi_rw_buffer(*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),command,NULL,command_length);
  //NSS high
  lr20xx_hal_wait_on_busy(context);
  //NSS low
  smtc_hal_mcu_spi_rw_buffer(*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),dummy_byte,dummy_byte_rx,2);
  smtc_hal_mcu_spi_rw_buffer(*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),NULL,data,data_length);
  //NSS high
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t lr20xx_hal_direct_read_fifo(void *context,uint8_t *command,uint16_t command_length,uint8_t *data,uint16_t data_length)
{
  lr20xx_hal_wait_on_busy(context);
  //NSS low
  smtc_hal_mcu_spi_rw_buffer(*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),command,NULL,command_length);
  smtc_hal_mcu_spi_rw_buffer(*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),NULL,data,data_length);
  //NSS high
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t lr20xx_hal_direct_read(void *radio,uint8_t *data,uint16_t data_length)
{
  uint16_t data_length_local;
  uint8_t *data_local;
  void *radio_local;
  lr20xx_hal_context_t *lr20xx_context;
  
  lr20xx_hal_wait_on_busy(radio);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)radio + 0x10),SMTC_HAL_MCU_GPIO_STATE_LOW);
  smtc_hal_mcu_spi_rw_buffer
            (*(smtc_hal_mcu_spi_inst_t *)((int)radio + 4),(uint8_t *)0x0,data,data_length);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)radio + 0x10),SMTC_HAL_MCU_GPIO_STATE_HIGH);
  return LR20XX_HAL_STATUS_OK;
}



void lr20xx_hal_wait_on_busy(void *radio)
{
  void *radio_local;
  smtc_hal_mcu_gpio_state_t gpio_state;
  lr20xx_hal_context_t *lr20xx_context;
  
  lr20xx_context = (lr20xx_hal_context_t *)radio;
  do
  {
    smtc_hal_mcu_gpio_get_state((lr20xx_context->busy).inst,&gpio_state);
  } while (gpio_state == SMTC_HAL_MCU_GPIO_STATE_HIGH);
  return;
}

