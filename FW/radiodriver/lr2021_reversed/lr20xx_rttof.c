
lr20xx_status_t lr20xx_rttof_set_responder_address(void *context,uint32_t address,uint8_t length)
{
  lr20xx_hal_status_t lVar1;
  uint8_t length_local;
  uint32_t address_local;
  void *context_local;
  uint8_t cbuffer [7];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'x';
  cbuffer[2] = (uint8_t)(address >> 0x18);
  cbuffer[3] = (uint8_t)(address >> 0x10);
  cbuffer[4] = (uint8_t)(address >> 8);
  cbuffer[5] = (uint8_t)address;
  cbuffer[6] = length;
  lVar1 = lr20xx_hal_write(context,cbuffer,7,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_rttof_set_initiator_address(void *context,uint32_t address)
{
  lr20xx_hal_status_t lVar1;
  uint32_t address_local;
  void *context_local;
  uint8_t cbuffer [6];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'y';
  cbuffer[2] = (uint8_t)(address >> 0x18);
  cbuffer[3] = (uint8_t)(address >> 0x10);
  cbuffer[4] = (uint8_t)(address >> 8);
  cbuffer[5] = (uint8_t)address;
  lVar1 = lr20xx_hal_write(context,cbuffer,6,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_rttof_get_results(void *context,lr20xx_rttof_results_t *result)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_rttof_results_t *result_local;
  void *context_local;
  uint8_t data [4];
  uint8_t cbuffer [3];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = 'z';
  cbuffer[2] = '\0';
  data[0] = '\0';
  data[1] = '\0';
  data[2] = '\0';
  data[3] = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,3,data,4);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    result->val = ((uint)data >> 8 & 0xff) * 0x100 + ((uint)data & 0xff) * 0x10000 +
                  ((uint)data >> 0x10 & 0xff);
    lr20xx_rttof_fix_and_convert_rssi(context,data[3],'\0',&result->rssi,(int8_t *)0x0);
  }
  return lVar1;
}



lr20xx_status_t lr20xx_rttof_set_tx_rx_delay(void *context,uint32_t delay_in_rtc_step)
{
  lr20xx_hal_status_t lVar1;
  uint32_t delay_in_rtc_step_local;
  void *context_local;
  uint8_t cbuffer [6];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '{';
  cbuffer[2] = (uint8_t)(delay_in_rtc_step >> 0x18);
  cbuffer[3] = (uint8_t)(delay_in_rtc_step >> 0x10);
  cbuffer[4] = (uint8_t)(delay_in_rtc_step >> 8);
  cbuffer[5] = (uint8_t)delay_in_rtc_step;
  lVar1 = lr20xx_hal_write(context,cbuffer,6,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_rttof_set_params(void *context,lr20xx_rttof_params_t *params)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_rttof_params_t *params_local;
  void *context_local;
  uint8_t cbuffer [3];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '|';
  cbuffer[2] = params->nb_symbol | params->spy_mode << 6 | params->mode << 7;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,(uint8_t *)0x0,0);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    lVar1 = lr20xx_rttof_apply_rttof_stuck_workaround(context,params->mode);
  }
  return lVar1;
}



int32_t lr20xx_rttof_distance_raw_to_meter(lr20xx_radio_lora_bw_t rttof_bw,int32_t raw_distance)
{
  uint32_t uVar1;
  int32_t raw_distance_local;
  lr20xx_radio_lora_bw_t rttof_bw_local;
  int32_t denominator;
  int32_t numerator;
  uint8_t bitcnt;
  int32_t retval;
  
  retval = raw_distance;
  if (0x7fffff < raw_distance)
  {
    retval = raw_distance + -0x1000000;
  }
  uVar1 = lr20xx_radio_lora_get_bw_in_hz(rttof_bw);
  return (retval * 0x96) / (int)((uVar1 << 0xc) / 1000000);
}



lr20xx_status_t
lr20xx_rttof_fix_and_convert_rssi
          (void *context,uint8_t rssi1_raw,uint8_t rssi2_raw,int8_t *rssi1_val,int8_t *rssi2_val)
{
  uint8_t *rssi2_raw_fixed_00;
  int8_t *rssi1_val_local;
  uint8_t rssi2_raw_local;
  uint8_t rssi1_raw_local;
  void *context_local;
  uint8_t rssi2_raw_fixed;
  uint8_t rssi1_raw_fixed;
  lr20xx_status_t workaround_status;
  
  rssi1_raw_fixed = '\0';
  rssi2_raw_fixed = '\0';
  if (rssi2_val == (int8_t *)0x0)
  {
    rssi2_raw_fixed_00 = (uint8_t *)0x0;
  }
  else
  {
    rssi2_raw_fixed_00 = &rssi2_raw_fixed;
  }
  workaround_status =
       lr20xx_workarounds_rttof_rssi_computation
                 (context,rssi1_raw,rssi2_raw,&rssi1_raw_fixed,rssi2_raw_fixed_00);
  if (workaround_status == LR20XX_STATUS_OK)
  {
    lr20xx_rttof_convert_rssi(rssi1_raw_fixed,rssi2_raw_fixed,rssi1_val,rssi2_val);
  }
  return workaround_status;
}



void lr20xx_rttof_convert_rssi
               (uint8_t rssi1_raw,uint8_t rssi2_raw,int8_t *rssi1_val,int8_t *rssi2_val)
{
  int8_t *rssi2_val_local;
  int8_t *rssi1_val_local;
  uint8_t rssi2_raw_local;
  uint8_t rssi1_raw_local;
  
  *rssi1_val = -(rssi1_raw >> 1);
  if (rssi2_val != (int8_t *)0x0)
  {
    *rssi2_val = -(rssi2_raw >> 1);
  }
  return;
}



lr20xx_status_t
lr20xx_rttof_apply_rttof_stuck_workaround(void *context,lr20xx_rttof_mode_t rttof_mode)
{
  lr20xx_status_t lVar1;
  lr20xx_rttof_mode_t rttof_mode_local;
  void *context_local;
  
  if (rttof_mode == LR20XX_RTTOF_MODE_NORMAL)
  {
    lVar1 = lr20xx_workarounds_rttof_extended_stuck_second_request_disable(context);
  }
  else if (rttof_mode == LR20XX_RTTOF_MODE_EXTENDED)
  {
    lVar1 = lr20xx_workarounds_rttof_extended_stuck_second_request_enable(context);
  }
  else
  {
    lVar1 = LR20XX_STATUS_ERROR;
  }
  return lVar1;
}

