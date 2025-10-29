

ral_status_t ral_lr20xx_init(void *context)
{
  _Bool _Var1;
  lr20xx_status_t lVar2;
  void *context_local;
  lr20xx_system_hf_clk_scaling_t hf_clk_scaling;
  lr20xx_system_dio_rf_switch_cfg_t rf_switch_cfg;
  lr20xx_system_dio_drive_t dio_drive;
  lr20xx_system_dio_func_t dio_function;
  lr20xx_system_dio_t dio;
  lr20xx_radio_common_front_end_calibration_value_t front_end_calibration_structures [3];
  uint16_t errors;
  lr20xx_system_lfclk_cfg_t lfclk_cfg;
  uint32_t startup_time_in_tick;
  lr20xx_system_tcxo_supply_voltage_t tcxo_supply_voltage;
  ral_xosc_cfg_t xosc_cfg;
  lr20xx_system_reg_mode_t reg_mode;
  uint32_t freq_val;
  uint8_t dio_count;
  ral_status_t status;
  uint8_t dio_nth;
  
  status = RAL_STATUS_ERROR;
  freq_val = 0x2ccccc;
  lr20xx_regmem_write_regmem32(context,0x80004c,&freq_val,'\x01');
  ral_lr20xx_bsp_get_reg_mode(context,&reg_mode);
  status = lr20xx_system_set_reg_mode(context,reg_mode);
  if (status == LR20XX_STATUS_OK)
  {
    startup_time_in_tick = 0;
    ral_lr20xx_bsp_get_xosc_cfg(context,&xosc_cfg,&tcxo_supply_voltage,&startup_time_in_tick);
    if ((xosc_cfg != RAL_XOSC_CFG_TCXO_RADIO_CTRL) ||
       (status = lr20xx_system_set_tcxo_mode(context,tcxo_supply_voltage,startup_time_in_tick),
       status == LR20XX_STATUS_OK))
    {
      ral_bsp_lr20xx_get_lfclk_cfg(context,&lfclk_cfg);
      status = lr20xx_system_cfg_lfclk(context,lfclk_cfg);
      if (status == LR20XX_STATUS_OK)
      {
        lr20xx_system_get_errors(context,&errors);
        if (errors != 0)
        {
          lr20xx_system_clear_errors(context);
        }
        dio_count = lr20xx_system_dio_get_count();
        for (dio_nth = '\0'; dio_nth < dio_count; dio_nth += '\x01')
        {
          dio_function = LR20XX_SYSTEM_DIO_FUNC_NONE;
          dio_drive = LR20XX_SYSTEM_DIO_DRIVE_NONE;
          _Var1 = lr20xx_system_dio_get_nth(dio_nth,&dio);
          if (!_Var1)
          {
            return RAL_STATUS_ERROR;
          }
          ral_lr20xx_bsp_get_dio_function(context,dio,&dio_function);
          ral_lr20xx_bsp_get_dio_sleep_drive(context,dio,&dio_drive);
          status = lr20xx_system_set_dio_function(context,dio,dio_function,dio_drive);
          if (status != LR20XX_STATUS_OK)
          {
            return status;
          }
          lVar2 = RAL_STATUS_OK;
          if (dio_function == LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH)
          {
            rf_switch_cfg = '\0';
            ral_lr20xx_bsp_get_dio_rf_switch_cfg(context,dio,&rf_switch_cfg);
            lVar2 = lr20xx_system_set_dio_rf_switch_cfg(context,dio,rf_switch_cfg);
            if (lVar2 != LR20XX_STATUS_OK)
            {
              return lVar2;
            }
          }
          status = lVar2;
          if (dio_function == LR20XX_SYSTEM_DIO_FUNC_HF_CLK_OUT)
          {
            hf_clk_scaling = LR20XX_SYSTEM_HF_CLK_SCALING_32_MHZ;
            ral_lr20xx_bsp_get_dio_hf_clk_scaling_cfg(context,&hf_clk_scaling);
            status = lr20xx_system_cfg_clk_output(context,hf_clk_scaling);
            if (status != LR20XX_STATUS_OK)
            {
              return status;
            }
          }
        }
        front_end_calibration_structures[0].rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF;
        front_end_calibration_structures[0]._1_3_ = 0;
        front_end_calibration_structures[0].frequency_in_hertz = 0;
        front_end_calibration_structures[1].rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF;
        front_end_calibration_structures[1]._1_3_ = 0;
        front_end_calibration_structures[1].frequency_in_hertz = 0;
        front_end_calibration_structures[2].rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF;
        front_end_calibration_structures[2]._1_3_ = 0;
        front_end_calibration_structures[2].frequency_in_hertz = 0;
        ral_lr20xx_bsp_get_front_end_calibration_cfg(context,front_end_calibration_structures);
        status = lr20xx_radio_common_calibrate_front_end_helper
                           (context,front_end_calibration_structures,'\x03');
      }
    }
  }
  return status;
}