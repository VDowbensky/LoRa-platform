int8_t radio_init_old(void)
{
  currfreq = radioconfig.freq / 1000;
	prevfreq = radioconfig.freq / 1000;
	switch(radioconfig.chip)
  {

		
		case 1121:
		{
			lr11xx_system_rfswitch_cfg_t rfsw_cfg;
			rfsw_cfg.enable = LR112X_RFSW_ENABLE;
			rfsw_cfg.standby = LR112X_RFSW_STBY;
			rfsw_cfg.rx = LR112X_RFSW_RX;
			rfsw_cfg.tx = LR112X_RFSW_SUBG_TX;
			rfsw_cfg.tx_hp = LR112X_RFSW_SUBG_TX_HP;
			rfsw_cfg.tx_hf = LR112X_RFSW_HF_TX;
			rfsw_cfg.gnss = LR112X_RFSW_GNSS;
			rfsw_cfg.wifi = LR112X_RFSW_HF_RX; //check
			
			
			lr11xx_system_reset(NULL);
			delay_ms(500);
			lr11xx_system_wakeup(NULL);
			//delay_ms(500);
			lr11xx_system_set_reg_mode(NULL,LR11XX_SYSTEM_REG_MODE_DCDC);
			//LR112X_SetStandby(0);
			//disable SPI CRC
			lr11xx_system_enable_spi_crc(NULL,false);
			lr11xx_system_set_dio_as_rf_switch(NULL,&rfsw_cfg); //for E80 - different
			//LR112X_printstatus();
			lr11xx_system_clear_errors(NULL);
			if(lr112x_tcxo) lr11xx_system_set_tcxo_mode(NULL,LR11XX_SYSTEM_TCXO_CTRL_1_8V,320);
			delay_ms(10);
			lr11xx_system_cfg_lfclk(NULL,LR11XX_SYSTEM_LFCLK_RC,true); //false
			lr11xx_system_clear_irq_status(NULL,LR11XX_SYSTEM_IRQ_ALL_MASK);
			lr11xx_system_set_standby(NULL,LR11XX_SYSTEM_STANDBY_CFG_XOSC);
			delay_ms(10);
			lr11xx_system_clear_errors(NULL);
			lr11xx_system_calibrate(NULL,0x3f);
			lr11xx_system_clear_errors(NULL);
			lr11xx_system_clear_irq_status(NULL,LR11XX_SYSTEM_IRQ_ALL_MASK);
			//lr11xx_system_get_version(NULL, &version);
			
			lr11xx_radio_set_lora_mod_params(NULL,&modparams);
			lr11xx_radio_set_lora_pkt_params(NULL,&pktparams);
			lr11xx_radio_set_lora_sync_word(NULL,radioconfig.sync);
			lr11xx_radio_set_rf_freq(NULL,radioconfig.freq);
			radio_set_power(radioconfig.txpower);
			//LR112X_SetTxParams(radioconfig.txpower,LR112X_PA_RAMP_48U);
			//calibrate image here
			lr11xx_system_calibrate_image(NULL,radioconfig.freq / 4000000, radioconfig.freq / 4000000 + 2); //must be rewritted
			//calibrate RSSI
			LR112X_RssiCal(radioconfig.freq);
			lr11xx_system_set_dio_irq_params(NULL,LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR,LR11XX_SYSTEM_IRQ_NONE);
			lr11xx_radio_set_rx_tx_fallback_mode(NULL,LR11XX_RADIO_FALLBACK_STDBY_XOSC);
			lr11xx_radio_cfg_rx_boosted(NULL,true); //false
			radio_rx();
			//The workaround is to set the bit 4 in the register 0x00F30024 when the chip ends a reception in the 2.4GHz band before launching a GNSS scan. - ???
			//LR112X_ReadRegMem32(lr,0x00f30024,buf,1);
			//buf[0] |= 0x10;
			//LR112X_WriteRegMem32(lr,0x00f30024,buf,1);
			return RADIO_OK;
		}

				
void lr1121_init(void)
{
		// system init
	uint16_t errors;
	lr11xx_system_version_t version;
		
	lr11xx_system_set_reg_mode(NULL,LR11XX_SYSTEM_REG_MODE_DCDC); // DC-DC
	lr11xx_system_set_dio_as_rf_switch(NULL, &rf_switch_setup);

	lr11xx_system_cfg_lfclk(NULL, LR11XX_SYSTEM_LFCLK_XTAL, true);
	lr11xx_system_clear_errors(NULL);
	lr11xx_system_calibrate(NULL, 0x3f);

	lr11xx_system_get_errors(NULL, &errors);
	lr11xx_system_clear_errors(NULL);
	lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);

	lr11xx_system_get_version(NULL, &version);
  
  
  

	// radio init
	lr11xx_radio_set_pkt_type(NULL, LR11XX_RADIO_PKT_TYPE_LORA);
	
	if(RF_FREQ_IN_HZ >=2400000000&&RF_FREQ_IN_HZ<=2500000000)
	{
		lr11xx_radio_set_rf_freq(NULL, RF_FREQ_IN_HZ);
		lr11xx_radio_set_pa_cfg(NULL, &pa_config_HF);	
	}
		
	else if(RF_FREQ_IN_HZ >=1900000000&&RF_FREQ_IN_HZ<=2200000000)
	{
		lr11xx_radio_set_rf_freq(NULL, RF_FREQ_IN_HZ);
		lr11xx_radio_set_pa_cfg(NULL, &pa_config_HF);	
	}
		
	else if(RF_FREQ_IN_HZ >=150000000&&RF_FREQ_IN_HZ<=960000000)
	{
		lr11xx_radio_set_rf_freq(NULL, RF_FREQ_IN_HZ);
		lr11xx_radio_set_pa_cfg(NULL, &pa_config_subGHz);	
	}
	
	lr11xx_radio_set_tx_params(NULL, TX_OUTPUT_POWER_DBM, LR11XX_RADIO_RAMP_48_US );	// // range [-17, +22] for sub-G, range [-18, 13] for 2.4G ( HF_PA )
	lr11xx_radio_set_rx_tx_fallback_mode(NULL, LR11XX_RADIO_FALLBACK_STDBY_RC);
	lr11xx_radio_cfg_rx_boosted(NULL, 0x00);		// enable_boost_mode

	lr11xx_radio_set_lora_mod_params(NULL, &lora_mod_params);
	lr11xx_radio_set_lora_pkt_params(NULL, &lora_pkt_params );

	air_time=lr11xx_radio_get_lora_time_on_air_in_ms(&lora_pkt_params,&lora_mod_params);
	air_time=air_time*4;

}