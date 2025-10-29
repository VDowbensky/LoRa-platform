
lr20xx_status_t lr20xx_system_reset(void *context)
{
  lr20xx_hal_status_t lVar1;
  void *context_local;
  
  lVar1 = lr20xx_hal_reset(context);
  return lVar1;
}



lr20xx_status_t lr20xx_system_wakeup(void *context)
{
  lr20xx_hal_status_t lVar1;
  void *context_local;
  
  lVar1 = lr20xx_hal_wakeup(context);
  return lVar1;
}



lr20xx_status_t
lr20xx_system_get_status
          (void *context,lr20xx_system_stat1_t *stat1,lr20xx_system_stat2_t *stat2,
          lr20xx_system_irq_mask_t *irq_status)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_irq_mask_t *irq_status_local;
  lr20xx_system_stat2_t *stat2_local;
  lr20xx_system_stat1_t *stat1_local;
  void *context_local;
  uint8_t data [6];
  lr20xx_status_t status;
  
  data[0] = '\0';
  data[1] = '\0';
  data[2] = '\0';
  data[3] = '\0';
  data[4] = '\0';
  data[5] = '\0';
  lVar1 = lr20xx_hal_direct_read(context,data,6);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    lr20xx_system_convert_stat1_byte_to_enum(data[0],stat1);
    lr20xx_system_convert_stat2_byte_to_enum(data[1],stat2);
    if (irq_status != (lr20xx_system_irq_mask_t *)0x0)
    {
      *irq_status = (uint)data[5] +
                    (uint)data[4] * 0x100 +
                    ((uint)data._0_4_ >> 0x10) * 0x1000000 + ((uint)data._0_4_ >> 0x18) * 0x10000;
    }
  }
  return lVar1;
}



lr20xx_status_t lr20xx_system_get_version(void *context,lr20xx_system_version_t *version)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_version_t *version_local;
  void *context_local;
  uint8_t rbuffer [2];
  uint8_t cbuffer [2];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x01';
  rbuffer[0] = '\0';
  rbuffer[1] = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,2,rbuffer,2);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    version->major = rbuffer[0];
    version->minor = rbuffer[1];
  }
  return lVar1;
}



lr20xx_status_t lr20xx_system_get_errors(void *context,lr20xx_system_errors_t *errors)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_errors_t *errors_local;
  void *context_local;
  uint8_t rbuffer [2];
  uint8_t cbuffer [2];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x10';
  rbuffer[0] = '\0';
  rbuffer[1] = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,2,rbuffer,2);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    *errors = ((ushort)rbuffer >> 8) + (short)rbuffer * 0x100;
  }
  return lVar1;
}



lr20xx_status_t lr20xx_system_clear_errors(void *context)
{
  lr20xx_hal_status_t lVar1;
  void *context_local;
  uint8_t cbuffer [2];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x11';
  lVar1 = lr20xx_hal_write(context,cbuffer,2,(uint8_t *)0x0,0);
  return lVar1;
}



// WARNING: Unknown calling convention

uint8_t lr20xx_system_dio_get_count(void)
{
  return '\a';
}



_Bool lr20xx_system_dio_get_nth(uint8_t nth,lr20xx_system_dio_t *dio)
{
  byte bVar1;
  lr20xx_system_dio_t *dio_local;
  uint8_t nth_local;
  
  bVar1 = lr20xx_system_dio_get_count();
  if (nth < bVar1)
  {
    *dio = dio_list[nth];
  }
  return nth < bVar1;
}



lr20xx_status_t
lr20xx_system_set_dio_function
          (void *context,lr20xx_system_dio_t dio,lr20xx_system_dio_func_t func,
          lr20xx_system_dio_drive_t drive)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_dio_drive_t drive_local;
  lr20xx_system_dio_func_t func_local;
  lr20xx_system_dio_t dio_local;
  void *context_local;
  uint8_t cbuffer [4];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x12';
  cbuffer[3] = drive + func * '\x10';
  cbuffer[2] = dio;
  lVar1 = lr20xx_hal_write(context,cbuffer,4,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_system_set_dio_rf_switch_cfg
          (void *context,lr20xx_system_dio_t dio,lr20xx_system_dio_rf_switch_cfg_t rf_switch_cfg)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_dio_rf_switch_cfg_t rf_switch_cfg_local;
  lr20xx_system_dio_t dio_local;
  void *context_local;
  uint8_t cbuffer [4];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x13';
  cbuffer[2] = dio;
  cbuffer[3] = rf_switch_cfg;
  lVar1 = lr20xx_hal_write(context,cbuffer,4,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_system_set_dio_irq_cfg
          (void *context,lr20xx_system_dio_t dio,lr20xx_system_irq_mask_t irq_cfg)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_irq_mask_t irq_cfg_local;
  lr20xx_system_dio_t dio_local;
  void *context_local;
  uint8_t cbuffer [7];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x15';
  cbuffer[3] = (uint8_t)(irq_cfg >> 0x18);
  cbuffer[4] = (uint8_t)(irq_cfg >> 0x10);
  cbuffer[5] = (uint8_t)(irq_cfg >> 8);
  cbuffer[6] = (uint8_t)irq_cfg;
  cbuffer[2] = dio;
  lVar1 = lr20xx_hal_write(context,cbuffer,7,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_system_clear_irq_status(void *context,lr20xx_system_irq_mask_t irqs_to_clear)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_irq_mask_t irqs_to_clear_local;
  void *context_local;
  uint8_t cbuffer [6];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x16';
  cbuffer[2] = (uint8_t)(irqs_to_clear >> 0x18);
  cbuffer[3] = (uint8_t)(irqs_to_clear >> 0x10);
  cbuffer[4] = (uint8_t)(irqs_to_clear >> 8);
  cbuffer[5] = (uint8_t)irqs_to_clear;
  lVar1 = lr20xx_hal_write(context,cbuffer,6,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_system_get_and_clear_irq_status(void *context,lr20xx_system_irq_mask_t *irqs)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_irq_mask_t *irqs_local;
  void *context_local;
  uint8_t rbuffer [4];
  uint8_t cbuffer [2];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x17';
  rbuffer[0] = '\0';
  rbuffer[1] = '\0';
  rbuffer[2] = '\0';
  rbuffer[3] = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,2,rbuffer,4);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    *irqs = ((uint)rbuffer >> 0x18) +
            ((uint)rbuffer >> 0x10 & 0xff) * 0x100 +
            (int)rbuffer * 0x1000000 + ((uint)rbuffer >> 8 & 0xff) * 0x10000;
  }
  return lVar1;
}



lr20xx_status_t lr20xx_system_cfg_lfclk(void *context,lr20xx_system_lfclk_cfg_t lfclock_cfg)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_lfclk_cfg_t lfclock_cfg_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x18';
  cbuffer[2] = lfclock_cfg;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_system_cfg_clk_output(void *context,lr20xx_system_hf_clk_scaling_t hf_clk_scaling)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_hf_clk_scaling_t hf_clk_scaling_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\x19';
  cbuffer[2] = hf_clk_scaling;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_system_set_tcxo_mode
          (void *context,lr20xx_system_tcxo_supply_voltage_t tune,uint32_t start_delay_in_rtc_step)
{
  lr20xx_hal_status_t lVar1;
  uint32_t start_delay_in_rtc_step_local;
  lr20xx_system_tcxo_supply_voltage_t tune_local;
  void *context_local;
  uint8_t cbuffer [7];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = ' ';
  cbuffer[3] = (uint8_t)(start_delay_in_rtc_step >> 0x18);
  cbuffer[4] = (uint8_t)(start_delay_in_rtc_step >> 0x10);
  cbuffer[5] = (uint8_t)(start_delay_in_rtc_step >> 8);
  cbuffer[6] = (uint8_t)start_delay_in_rtc_step;
  cbuffer[2] = tune;
  lVar1 = lr20xx_hal_write(context,cbuffer,7,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_system_set_reg_mode(void *context,lr20xx_system_reg_mode_t reg_mode)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_reg_mode_t reg_mode_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '!';
  cbuffer[2] = reg_mode;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_system_get_random_number
          (void *context,lr20xx_system_random_entropy_source_bitmask_t source,
          uint32_t *random_number)
{
  lr20xx_hal_status_t lVar1;
  uint32_t *random_number_local;
  lr20xx_system_random_entropy_source_bitmask_t source_local;
  void *context_local;
  uint8_t buffer [4];
  uint8_t cbuffer [3];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '&';
  buffer[0] = '\0';
  buffer[1] = '\0';
  buffer[2] = '\0';
  buffer[3] = '\0';
  cbuffer[2] = source;
  lVar1 = lr20xx_hal_read(context,cbuffer,3,buffer,4);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    *random_number =
         ((uint)buffer >> 0x18) +
         ((uint)buffer >> 0x10 & 0xff) * 0x100 +
         (int)buffer * 0x1000000 + ((uint)buffer >> 8 & 0xff) * 0x10000;
  }
  return lVar1;
}



lr20xx_status_t
lr20xx_system_set_sleep_mode(void *context,lr20xx_system_sleep_cfg_t *sleep_cfg,uint32_t sleep_time)
{
  lr20xx_hal_status_t lVar1;
  char cVar2;
  uint32_t sleep_time_local;
  lr20xx_system_sleep_cfg_t *sleep_cfg_local;
  void *context_local;
  uint8_t cbuffer [7];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '\'';
  if (sleep_cfg->is_ram_retention_enabled == false)
  {
    cVar2 = '\0';
  }
  else
  {
    cVar2 = '\x02';
  }
  cbuffer[2] = (sleep_cfg->is_clk_32k_enabled != false) + cVar2;
  cbuffer[3] = (uint8_t)(sleep_time >> 0x18);
  cbuffer[4] = (uint8_t)(sleep_time >> 0x10);
  cbuffer[5] = (uint8_t)(sleep_time >> 8);
  cbuffer[6] = (uint8_t)sleep_time;
  lVar1 = lr20xx_hal_write(context,cbuffer,7,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_system_set_standby_mode(void *context,lr20xx_system_standby_mode_t standby_mode)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_system_standby_mode_t standby_mode_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = '(';
  cbuffer[2] = standby_mode;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_system_set_fs_mode(void *context)
{
  lr20xx_hal_status_t lVar1;
  void *context_local;
  uint8_t cbuffer [2];
  
  cbuffer[0] = '\x01';
  cbuffer[1] = ')';
  lVar1 = lr20xx_hal_write(context,cbuffer,2,(uint8_t *)0x0,0);
  return lVar1;
}



void lr20xx_system_convert_stat1_byte_to_enum(uint8_t stat1_byte,lr20xx_system_stat1_t *stat1)
{
  lr20xx_system_stat1_t *stat1_local;
  uint8_t stat1_byte_local;
  
  if (stat1 != (lr20xx_system_stat1_t *)0x0)
  {
    stat1->is_interrupt_active = (stat1_byte & 1) != 0;
    stat1->command_status = stat1_byte >> 1;
  }
  return;
}



void lr20xx_system_convert_stat2_byte_to_enum(uint8_t stat2_byte,lr20xx_system_stat2_t *stat2)
{
  lr20xx_system_stat2_t *stat2_local;
  uint8_t stat2_byte_local;
  
  if (stat2 != (lr20xx_system_stat2_t *)0x0)
  {
    stat2->chip_mode = stat2_byte & (LR20XX_SYSTEM_CHIP_MODE_RX|LR20XX_SYSTEM_CHIP_MODE_FS);
    stat2->reset_status = stat2_byte >> 4;
  }
  return;
}
