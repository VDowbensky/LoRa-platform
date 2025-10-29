
lr20xx_hal_status_t lr20xx_hal_reset(void *context)
{
  void *context_local;
  lr20xx_hal_context_t *lr20xx_context;
  
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x1c),SMTC_HAL_MCU_GPIO_STATE_LOW);
  smtc_hal_mcu_wait_ms(1);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x1c),SMTC_HAL_MCU_GPIO_STATE_HIGH);
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t lr20xx_hal_wakeup(void *context)
{
  void *context_local;
  lr20xx_hal_context_t *lr20xx_context;
  
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x10),SMTC_HAL_MCU_GPIO_STATE_LOW);
  smtc_hal_mcu_wait_ms(1);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x10),SMTC_HAL_MCU_GPIO_STATE_HIGH);
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t
lr20xx_hal_write(void *context,uint8_t *command,uint16_t command_length,uint8_t *data,
                uint16_t data_length)
{
  uint8_t *data_local;
  uint16_t command_length_local;
  uint8_t *command_local;
  void *context_local;
  lr20xx_hal_context_t *lr20xx_context;
  
  lr20xx_hal_wait_on_busy(context);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x10),SMTC_HAL_MCU_GPIO_STATE_LOW);
  smtc_hal_mcu_spi_rw_buffer
            (*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),command,(uint8_t *)0x0,command_length);
  smtc_hal_mcu_spi_rw_buffer
            (*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),data,(uint8_t *)0x0,data_length);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x10),SMTC_HAL_MCU_GPIO_STATE_HIGH);
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t
lr20xx_hal_read(void *context,uint8_t *command,uint16_t command_length,uint8_t *data,
               uint16_t data_length)
{
  uint8_t *data_local;
  uint16_t command_length_local;
  uint8_t *command_local;
  void *context_local;
  uint8_t dummy_byte_rx [2];
  uint8_t dummy_byte [2];
  lr20xx_hal_context_t *lr20xx_context;
  
  dummy_byte[0] = '\0';
  dummy_byte[1] = '\0';
  dummy_byte_rx[0] = '\0';
  dummy_byte_rx[1] = '\0';
  lr20xx_hal_wait_on_busy(context);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x10),SMTC_HAL_MCU_GPIO_STATE_LOW);
  smtc_hal_mcu_spi_rw_buffer
            (*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),command,(uint8_t *)0x0,command_length);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x10),SMTC_HAL_MCU_GPIO_STATE_HIGH);
  lr20xx_hal_wait_on_busy(context);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x10),SMTC_HAL_MCU_GPIO_STATE_LOW);
  smtc_hal_mcu_spi_rw_buffer
            (*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),dummy_byte,dummy_byte_rx,2);
  smtc_hal_mcu_spi_rw_buffer
            (*(smtc_hal_mcu_spi_inst_t *)((int)context + 4),(uint8_t *)0x0,data,data_length);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)context + 0x10),SMTC_HAL_MCU_GPIO_STATE_HIGH);
  return LR20XX_HAL_STATUS_OK;
}



lr20xx_hal_status_t
lr20xx_hal_direct_read_fifo
          (void *radio,uint8_t *command,uint16_t command_length,uint8_t *data,uint16_t data_length)
{
  uint8_t *data_local;
  uint16_t command_length_local;
  uint8_t *command_local;
  void *radio_local;
  lr20xx_hal_context_t *lr20xx_context;
  
  lr20xx_hal_wait_on_busy(radio);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)radio + 0x10),SMTC_HAL_MCU_GPIO_STATE_LOW);
  smtc_hal_mcu_spi_rw_buffer
            (*(smtc_hal_mcu_spi_inst_t *)((int)radio + 4),command,(uint8_t *)0x0,command_length);
  smtc_hal_mcu_spi_rw_buffer
            (*(smtc_hal_mcu_spi_inst_t *)((int)radio + 4),(uint8_t *)0x0,data,data_length);
  smtc_hal_mcu_gpio_set_state
            (*(smtc_hal_mcu_gpio_inst_t *)((int)radio + 0x10),SMTC_HAL_MCU_GPIO_STATE_HIGH);
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

