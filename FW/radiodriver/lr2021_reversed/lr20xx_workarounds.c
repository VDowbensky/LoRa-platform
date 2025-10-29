lr20xx_status_t lr20xx_workarounds_ook_set_detection_threshold_level(void *context,int16_t threshold_level_db)
{
  return lr20xx_regmem_write_regmem32_mask(context,0xf30e14,0x7f00000,((int)threshold_level_db + 0x4aU & 0x7f) << 0x14);
}



lr20xx_status_t lr20xx_workarounds_rttof_rssi_computation
          (void *context,uint8_t rssi1_raw_value,uint8_t rssi2_raw_value,uint8_t *rssi1_raw_fixed,
          uint8_t *rssi2_raw_fixed)
{
  int16_t power_offset;
  uint16_t max_gain;
  lr20xx_status_t return_code;
  
  max_gain = 0;
  power_offset = 0;
  return_code = lr20xx_workarounds_rttof_rssi_computation_get_gain_power(context,&max_gain,&power_offset);
  if (return_code == LR20XX_STATUS_OK)
  {
    *rssi1_raw_fixed = lr20xx_workarounds_rttof_rssi_computation_apply_correction(max_gain,power_offset,rssi1_raw_value);
    if (rssi2_raw_fixed != NULL)
    {
      *rssi2_raw_fixed = lr20xx_workarounds_rttof_rssi_computation_apply_correction(max_gain,power_offset,rssi2_raw_value);
    }
    return_code = LR20XX_STATUS_OK;
  }
  return return_code;
}



lr20xx_status_t lr20xx_workarounds_dcdc_reset(void *context)
{
  lr20xx_status_t return_code;
  
  return_code = lr20xx_regmem_write_regmem32_mask(context,0xf20024,0xf00000,0xf00000);
  if (return_code == LR20XX_STATUS_OK) 
  {
    return_code = lr20xx_regmem_write_regmem32_mask(context,0xf20024,0xf0000,0xf0000);
    if(return_code == LR20XX_STATUS_OK) return_code = lr20xx_workaround_dcdc_set_frequency(context,2800000);
  }
  return return_code;
}



lr20xx_status_t lr20xx_workarounds_dcdc_configure(void *context)
{
  lr20xx_status_t lVar1;
  lr20xx_status_t lVar2;
  void *context_local;
  uint32_t rx_path_raw;
  uint32_t adc_ctrl_raw;
  lr20xx_status_t return_code_5;
  lr20xx_status_t return_code_4;
  lr20xx_status_t return_code_3;
  lr20xx_status_t return_code_2;
  _Bool is_rx_hf;
  lr20xx_status_t return_code_1;
  uint32_t ana_dec;
  lr20xx_status_t return_code;
  
  adc_ctrl_raw = 0;
  return_code_1 = lr20xx_regmem_read_regmem32(context,0xf40200,&adc_ctrl_raw,'\x01');
  if (return_code_1 == LR20XX_STATUS_OK)
  {
    ana_dec = adc_ctrl_raw >> 8 & 7;
    rx_path_raw = 0;
    return_code = return_code_1;
    return_code_1 = lr20xx_regmem_read_regmem32(context,0xf40430,&rx_path_raw,'\x01');
    if (return_code_1 == LR20XX_STATUS_OK)
    {
      is_rx_hf = (rx_path_raw & 3) == 1;
      if ((is_rx_hf) || ((ana_dec != 1 && (ana_dec != 2))))
      {
        return_code_4 = lr20xx_regmem_write_regmem32_mask(context,0xf20024,0xf00000,0xf00000);
        if (return_code_4 != LR20XX_STATUS_OK)
        {
          return return_code_4;
        }
        return_code_5 = lr20xx_regmem_write_regmem32_mask(context,0xf20024,0xf0000,0xf0000);
        lVar2 = return_code_5;
        lVar1 = return_code_1;
      }
      else
      {
        return_code_2 = lr20xx_regmem_write_regmem32_mask(context,0xf20024,0xf00000,0xb00000);
        if (return_code_2 != LR20XX_STATUS_OK)
        {
          return return_code_2;
        }
        return_code_3 = lr20xx_regmem_write_regmem32_mask(context,0xf20024,0xf0000,0xd0000);
        lVar2 = return_code_3;
        lVar1 = return_code_1;
      }
      return_code_1 = lVar2;
      if (return_code_1 == LR20XX_STATUS_OK)
      {
        return_code_1 = lVar1;
        if (ana_dec == 1)
        {
          return_code_1 = lr20xx_workaround_dcdc_set_frequency(context,4300000);
        }
        else
        {
          return_code_1 = lr20xx_workaround_dcdc_set_frequency(context,2800000);
        }
      }
    }
  }
  return return_code_1;
}



lr20xx_status_t lr20xx_workarounds_rttof_extended_stuck_second_request_enable(void *context)
{
  return lr20xx_regmem_write_regmem32_mask(context,0xf30b50,0x7000000,0);
}



lr20xx_status_t lr20xx_workarounds_rttof_extended_stuck_second_request_disable(void *context)
{
  return lr20xx_regmem_write_regmem32_mask(context,0xf30b50,0x7000000,0x1000000);
}



lr20xx_status_t lr20xx_workarounds_rttof_rssi_computation_get_gain_power(void *context,uint16_t *max_gain,int16_t *power_offset)
{
  uint32_t power_offset_raw;
  //int16_t power_offset_raw_value;
  lr20xx_status_t return_code;
  
  uint32_t max_gain_raw = 0;
  lr20xx_status_t  return_code = lr20xx_regmem_read_regmem32(context,0xf301a4,&max_gain_raw,1);
  if (return_code == LR20XX_STATUS_OK)
  {
    *max_gain = (uint16_t)((max_gain_raw << 0x16) >> 0x16);
    power_offset_raw = 0;
    return_code = lr20xx_regmem_read_regmem32(context,0xf30128,&power_offset_raw,1);
    if (return_code == LR20XX_STATUS_OK)
    {
      uint16_t tmp = (uint16_t)(power_offset_raw >> 6) & 0x3f;
      if(tmp > 0x20) tmp -= 0x40;
      *power_offset = tmp;
      return_code = LR20XX_STATUS_OK;
    }
  }
  return return_code;
}



uint8_t lr20xx_workarounds_rttof_rssi_computation_apply_correction(uint16_t max_gain,int16_t power_offset,uint8_t raw_rssi)
{
  return (uint8_t)power_offset + (uint8_t)(max_gain >> 1) - 2*raw_rssi + 0xd0;
}



lr20xx_status_t lr20xx_workaround_dcdc_set_frequency(void *context,uint32_t frequency)
{
  uint in_fpscr; //???
  uint32_t rf_frequency;
  lr20xx_status_t return_code;
  
  float fVar1 = (float)VectorUnsignedToFloat(frequency,(byte)(in_fpscr >> 0x15) & 3); //???
  uint32_t freq_lf = VectorFloatToUnsigned(fVar1 * 1.048576,3);
  return_code = lr20xx_regmem_write_regmem32(context,0x80004c,&freq_lf,1);
  if (return_code == LR20XX_STATUS_OK)
  {
    rf_frequency = 0;
    return_code = lr20xx_workaround_dcdc_get_rf_frequency(context,&rf_frequency);
    if (return_code == LR20XX_STATUS_OK)
    {
      return_code = lr20xx_radio_common_set_rf_freq(context,rf_frequency);
    }
  }
  return return_code;
}



lr20xx_status_t lr20xx_workaround_dcdc_get_rf_frequency(void *context,uint32_t *frequency)
{
  uint32_t raw_rf_freq = 0;
  lr20xx_status_t return_code;
  
  return_code = lr20xx_regmem_read_regmem32(context,0xf40144,&raw_rf_freq,1);
  if (return_code == LR20XX_STATUS_OK)
  {
    *frequency = pll_step_to_hz(raw_rf_freq);
    return_code = LR20XX_STATUS_OK;
  }
  return return_code;
}

