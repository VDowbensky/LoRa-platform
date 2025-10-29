


lr20xx_status_t lr20xx_radio_common_calibrate_front_end(void *context,lr20xx_radio_common_raw_front_end_calibration_value_t *front_end_calibration_values,
          uint8_t n_front_end_calibration_values)
{
  uint8_t n_front_end_calibration_values_local;
  lr20xx_radio_common_raw_front_end_calibration_value_t *front_end_calibration_values_local;
  uint8_t raw_front_end_calibration_values [6];
  uint8_t cbuffer [2];
  uint8_t rx_path_frequency_index;
  
  cbuffer[0] = 0x01;
  cbuffer[1] = 0x23;
  raw_front_end_calibration_values[0] = 0;
  raw_front_end_calibration_values[1] = 0;
  raw_front_end_calibration_values[2] = 0;
  raw_front_end_calibration_values[3] = 0;
  raw_front_end_calibration_values[4] = 0;
  raw_front_end_calibration_values[5] = 0;
  for (rx_path_frequency_index = 0; rx_path_frequency_index < n_front_end_calibration_values;rx_path_frequency_index++)
  {
    raw_front_end_calibration_values[(uint)rx_path_frequency_index * 2] =     (uint8_t)(front_end_calibration_values[rx_path_frequency_index] >> 8);
    raw_front_end_calibration_values[(uint)rx_path_frequency_index * 2 + 1] = (uint8_t)front_end_calibration_values[rx_path_frequency_index];
  }
  return lr20xx_hal_write(context,cbuffer,2,raw_front_end_calibration_values,(uint16_t)n_front_end_calibration_values << 1);
}



lr20xx_status_t lr20xx_radio_common_calibrate_front_end_helper
          (void *context,
          lr20xx_radio_common_front_end_calibration_value_t *front_end_calibration_structures,
          uint8_t n_front_end_calibration_structures)
{
  lr20xx_radio_common_front_end_calibration_value_t *front_end_calibration_structures_local;
  lr20xx_radio_common_raw_front_end_calibration_value_t raw_calibration_values [3];
  uint16_t freq_4mhz;
  lr20xx_radio_common_rx_path_t rx_path;
  uint32_t freq_hz;
  uint8_t front_end_calibration_value_index;
  
  raw_calibration_values[0] = 0;
  raw_calibration_values[1] = 0;
  raw_calibration_values[2] = 0;
  for (front_end_calibration_value_index = 0;front_end_calibration_value_index < n_front_end_calibration_structures;front_end_calibration_value_index++)
  {
    if (front_end_calibration_structures[front_end_calibration_value_index].rx_path == LR20XX_RADIO_COMMON_RX_PATH_HF) 
    {
      raw_calibration_values[front_end_calibration_value_index] = 
      (uint16_t)((front_end_calibration_structures[front_end_calibration_value_index].frequency_in_hertz + 3999999) / 4000000) | 0x8000;
    }
  }
  return lr20xx_radio_common_calibrate_front_end(context,raw_calibration_values,n_front_end_calibration_structures);
}



uint32_t lr20xx_radio_common_convert_time_in_ms_to_rtc_step(uint32_t time_in_ms)
{
  uint32_t time_in_ms_local;
  
  return (time_in_ms << 0xf) / 1000;
}



lr20xx_status_t lr20xx_radio_common_set_rf_freq(void *context,uint32_t freq_in_hz)
{
  uint8_t cbuffer [6];
  
  cbuffer[0] = 0x02;
  cbuffer[1] = 0;
  cbuffer[2] = (uint8_t)(freq_in_hz >> 0x18);
  cbuffer[3] = (uint8_t)(freq_in_hz >> 0x10);
  cbuffer[4] = (uint8_t)(freq_in_hz >> 8);
  cbuffer[5] = (uint8_t)freq_in_hz;
  return lr20xx_hal_write(context,cbuffer,6,(uint8_t *)0x0,0);
}



lr20xx_status_t lr20xx_radio_common_set_rx_path
          (void *context,lr20xx_radio_common_rx_path_t rx_path,
          lr20xx_radio_common_rx_path_boost_mode_t boost_mode)
{
  lr20xx_hal_status_t return_code;
  uint8_t cbuffer [4];
  lr20xx_status_t write_status;
  
  cbuffer[0] = 0x02;
  cbuffer[1] = 0x01;
  cbuffer[2] = rx_path;
  cbuffer[3] = boost_mode;
  return_code = lr20xx_hal_write(context,cbuffer,4,NULL,0);
  if (return_code == LR20XX_HAL_STATUS_OK)
  {
    return_code = lr20xx_workarounds_dcdc_configure(context);
  }
  return return_code;
}



lr20xx_status_t lr20xx_radio_common_set_pa_cfg(void *context,lr20xx_radio_common_pa_cfg_t *pa_cfg)
{
  uint8_t cbuffer [5];
  
  cbuffer[0] = 0x02;
  cbuffer[1] = 0x02;
  cbuffer[2] = pa_cfg->pa_lf_mode + pa_cfg->pa_sel * -0x80;
  cbuffer[3] = pa_cfg->pa_lf_slices + pa_cfg->pa_lf_duty_cycle * 0x10;
  cbuffer[4] = pa_cfg->pa_hf_duty_cycle;
  return lr20xx_hal_write(context,cbuffer,5,(NULL,0);
}



lr20xx_status_t lr20xx_radio_common_set_tx_params(void *context,int8_t power_half_dbm,lr20xx_radio_common_ramp_time_t ramp_time)
{
  uint8_t cbuffer [4];
  
  cbuffer[0] = 0x02;
  cbuffer[1] = 0x03;
  cbuffer[2] = power_half_dbm;
  cbuffer[3] = ramp_time;
  return lr20xx_hal_write(context,cbuffer,4,NULL,0);
}



lr20xx_status_t lr20xx_radio_common_set_rx_tx_fallback_mode
          (void *context,lr20xx_radio_common_fallback_modes_t fallback_mode)
{
  uint8_t cbuffer [3];
  
  cbuffer[0] = 0x02;
  cbuffer[1] = 0x06;
  cbuffer[2] = fallback_mode;
  return lr20xx_hal_write(context,cbuffer,3,NULL,0);
}



lr20xx_status_t lr20xx_radio_common_set_pkt_type(void *context,lr20xx_radio_common_pkt_type_t pkt_type)
{
  lr20xx_hal_status_t return_code;
  lr20xx_radio_common_pkt_type_t pkt_type_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = 0x02;
  cbuffer[1] = 0x07;
  cbuffer[2] = pkt_type;
  return_code = lr20xx_hal_write(context,cbuffer,3,NULL,0);
  if (return_code == LR20XX_HAL_STATUS_OK)
  {
    return_code = lr20xx_workarounds_dcdc_reset(context);
  }
  return return_code;
}



lr20xx_status_t
lr20xx_radio_common_get_pkt_type(void *context,lr20xx_radio_common_pkt_type_t *pkt_type)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_common_pkt_type_t *pkt_type_local;
  void *context_local;
  uint8_t pkt_type_raw;
  uint8_t cbuffer [2];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\b';
  pkt_type_raw = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,2,&pkt_type_raw,1);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    *pkt_type = pkt_type_raw;
  }
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_common_set_rx_timeout_stop_event(void *context,_Bool is_stopped_on_preamble_detection)
{
  lr20xx_hal_status_t lVar1;
  _Bool is_stopped_on_preamble_detection_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\t';
  cbuffer[2] = is_stopped_on_preamble_detection;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_common_get_rssi_inst(void *context,int16_t *rssi_in_dbm,uint8_t *half_dbm_count)
{
  lr20xx_hal_status_t lVar1;
  uint8_t *half_dbm_count_local;
  int16_t *rssi_in_dbm_local;
  void *context_local;
  uint8_t rssi_raw [2];
  uint8_t cbuffer [2];
  lr20xx_status_t status;
  
  cbuffer[0] = 0x2;
  cbuffer[1] = 0xb;
  rssi_raw[0] = '\0';
  rssi_raw[1] = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,2,rssi_raw,2);
  if ((lVar1 == LR20XX_HAL_STATUS_OK) &&
     (*rssi_in_dbm = -((ushort)rssi_raw & 0xff), half_dbm_count != (uint8_t *)0x0))
  {
    *half_dbm_count = rssi_raw[1] & 1;
  }
  return lVar1;
}



lr20xx_status_t lr20xx_radio_common_set_rx(void *context,uint32_t timeout_in_ms)
{
  lr20xx_status_t lVar1;
  uint32_t timeout_in_rtc_step;
  uint32_t timeout_in_ms_local;
  void *context_local;
  
  timeout_in_rtc_step = lr20xx_radio_common_convert_time_in_ms_to_rtc_step(timeout_in_ms);
  lVar1 = lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(context,timeout_in_rtc_step);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(void *context,uint32_t timeout_in_rtc_step)
{
  lr20xx_hal_status_t lVar1;
  uint32_t timeout_in_rtc_step_local;
  void *context_local;
  uint8_t cbuffer [5];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\f';
  cbuffer[2] = (uint8_t)(timeout_in_rtc_step >> 0x10);
  cbuffer[3] = (uint8_t)(timeout_in_rtc_step >> 8);
  cbuffer[4] = (uint8_t)timeout_in_rtc_step;
  lVar1 = lr20xx_hal_write(context,cbuffer,5,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_radio_common_set_tx(void *context,uint32_t timeout_in_ms)
{
  lr20xx_status_t lVar1;
  uint32_t timeout_in_rtc_step;
  uint32_t timeout_in_ms_local;
  void *context_local;
  
  timeout_in_rtc_step = lr20xx_radio_common_convert_time_in_ms_to_rtc_step(timeout_in_ms);
  lVar1 = lr20xx_radio_common_set_tx_with_timeout_in_rtc_step(context,timeout_in_rtc_step);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_common_set_tx_with_timeout_in_rtc_step(void *context,uint32_t timeout_in_rtc_step)
{
  lr20xx_hal_status_t lVar1;
  uint32_t timeout_in_rtc_step_local;
  void *context_local;
  uint8_t cbuffer [5];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\r';
  cbuffer[2] = (uint8_t)(timeout_in_rtc_step >> 0x10);
  cbuffer[3] = (uint8_t)(timeout_in_rtc_step >> 8);
  cbuffer[4] = (uint8_t)timeout_in_rtc_step;
  lVar1 = lr20xx_hal_write(context,cbuffer,5,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_common_set_tx_test_mode(void *context,lr20xx_radio_common_tx_test_mode_t mode)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_common_tx_test_mode_t mode_local;
  void *context_local;
  uint8_t cbuffer [3];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\x0e';
  cbuffer[2] = mode;
  lVar1 = lr20xx_hal_write(context,cbuffer,3,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_common_set_rx_duty_cycle
          (void *context,uint32_t rx_period_in_ms,uint32_t sleep_period_in_ms,
          lr20xx_radio_common_rx_duty_cycle_mode_t mode)
{
  lr20xx_status_t lVar1;
  uint32_t rx_period_in_rtc_step;
  uint32_t sleep_period_in_rtc_step;
  lr20xx_radio_common_rx_duty_cycle_mode_t mode_local;
  uint32_t sleep_period_in_ms_local;
  uint32_t rx_period_in_ms_local;
  void *context_local;
  
  rx_period_in_rtc_step = lr20xx_radio_common_convert_time_in_ms_to_rtc_step(rx_period_in_ms);
  sleep_period_in_rtc_step = lr20xx_radio_common_convert_time_in_ms_to_rtc_step(sleep_period_in_ms);
  lVar1 = lr20xx_radio_common_set_rx_duty_cycle_with_timing_in_rtc_step
                    (context,rx_period_in_rtc_step,sleep_period_in_rtc_step,mode);
  return lVar1;
}



lr20xx_status_t
lr20xx_radio_common_set_rx_duty_cycle_with_timing_in_rtc_step
          (void *context,uint32_t rx_period_in_rtc_step,uint32_t sleep_period_in_rtc_step,
          lr20xx_radio_common_rx_duty_cycle_mode_t mode)
{
  lr20xx_hal_status_t lVar1;
  lr20xx_radio_common_rx_duty_cycle_mode_t mode_local;
  uint32_t sleep_period_in_rtc_step_local;
  uint32_t rx_period_in_rtc_step_local;
  void *context_local;
  uint8_t cbuffer [9];
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\x10';
  cbuffer[2] = (uint8_t)(rx_period_in_rtc_step >> 0x10);
  cbuffer[3] = (uint8_t)(rx_period_in_rtc_step >> 8);
  cbuffer[4] = (uint8_t)rx_period_in_rtc_step;
  cbuffer[5] = (uint8_t)(sleep_period_in_rtc_step >> 0x10);
  cbuffer[6] = (uint8_t)(sleep_period_in_rtc_step >> 8);
  cbuffer[7] = (uint8_t)sleep_period_in_rtc_step;
  cbuffer[8] = mode << 4;
  lVar1 = lr20xx_hal_write(context,cbuffer,9,(uint8_t *)0x0,0);
  return lVar1;
}



lr20xx_status_t lr20xx_radio_common_get_rx_packet_length(void *context,uint16_t *pkt_len)
{
  lr20xx_hal_status_t lVar1;
  uint16_t *pkt_len_local;
  void *context_local;
  uint8_t pkt_len_loc [2];
  uint8_t cbuffer [2];
  lr20xx_status_t status;
  
  cbuffer[0] = '\x02';
  cbuffer[1] = '\x12';
  pkt_len_loc[0] = '\0';
  pkt_len_loc[1] = '\0';
  lVar1 = lr20xx_hal_read(context,cbuffer,2,pkt_len_loc,2);
  if (lVar1 == LR20XX_HAL_STATUS_OK)
  {
    *pkt_len = ((ushort)pkt_len_loc >> 8) + (short)pkt_len_loc * 0x100;
  }
  return lVar1;
}
