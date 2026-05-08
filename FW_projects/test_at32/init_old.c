int8_t radio_init_old(void)
{
  currfreq = radioconfig.freq / 1000;
	prevfreq = radioconfig.freq / 1000;
	switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_mod_params_lora_t modparams;
			modparams.bw = (sx126x_lora_bw_t)SX126X_bw[radioconfig.bw_index];
			modparams.cr = (sx126x_lora_cr_t)radioconfig.cr;
			modparams.sf = (sx126x_lora_sf_t)radioconfig.sf;
			modparams.ldro = radioconfig.ldropt;
			
			sx126x_pkt_params_lora_t pktparams;
			pktparams.header_type = (sx126x_lora_pkt_len_modes_t)radioconfig.header;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			
			sx126x_pa_cfg_params_t paconfig;
			paconfig.device_sel = 0;
			paconfig.pa_duty_cycle = 4;
			paconfig.hp_max = 7;
			paconfig.pa_lut = 1;
			
			
			sx126x_set_pkt_type(NULL,SX126X_PKT_TYPE_LORA);
			sx126x_set_rf_freq(NULL,radioconfig.freq);
			sx126x_set_buffer_base_address(NULL,0,0);
			sx126x_set_lora_mod_params(NULL,&modparams);
			

			SX126X_CalibrateIR();
			if(sx126x_tcxo == 0) //TCXO off
			{
				SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
				sx126x_set_trimming_capacitor_values(NULL,sx126x_xtatrim,sx126x_xtbtrim);
			}

    sx126x_cfg_rx_boosted(NULL,true);
    sx126x_set_dio2_as_rf_sw_ctrl(NULL,true);
    sx126x_set_dio_irq_params(NULL,SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR,SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE,SX126X_IRQ_NONE,SX126X_IRQ_NONE);

    return RADIO_OK;
		}
		
		case 1280:
		{
			sx128x_mod_params_lora_t modparams;
			modparams.bw = (sx128x_lora_bw_t)SX128X_bw[radioconfig.bw_index];
			modparams.sf = (sx128x_lora_sf_t)(radioconfig.sf << 4);
			modparams.cr = (sx128x_lora_cr_t)radioconfig.cr;
			
			sx128x_pkt_params_lora_t pktparams;
			uint8_t m,e;
			SX128X_CalcPreamble(radioconfig.prelen,&m,&e);
			pktparams.preamble_len.mant = m;
			pktparams.preamble_len.exp = e;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			if(radioconfig.header == false) pktparams.header_type = SX128X_LORA_RANGING_PKT_EXPLICIT;
			else pktparams.header_type = SX128X_LORA_RANGING_PKT_IMPLICIT;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			
			
			sx128x_set_pkt_type(NULL,SX128X_PKT_TYPE_LORA);
			sx128x_set_lora_mod_params(NULL,&modparams);
			sx128x_set_lora_pkt_params(NULL,&pktparams);
			//set sync
			sx128x_set_lora_sync_word(NULL,radioconfig.sync & 0xff);
			sx128x_set_rf_freq(NULL,radioconfig.freq);
			sx128x_set_buffer_base_address(NULL,0,0);
			sx128x_set_tx_params(NULL,radioconfig.txpower,SX128X_RAMP_02_US);
			//SX128X_CalibrateIR();
			sx128x_set_lna_settings(NULL,SX128X_LNA_HIGH_SENSITIVITY_MODE); //SX128X_LNA_LOW_POWER_MODE
			sx128x_set_dio_irq_params(NULL,SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE | SX128X_IRQ_CRC_ERROR,SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE,SX128X_IRQ_NONE,SX128X_IRQ_NONE);
			return RADIO_OK;
		}
		
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
			
			lr11xx_radio_mod_params_lora_t modparams;
			modparams.bw = (lr11xx_radio_lora_bw_t)LR112X_bw[radioconfig.bw_index];
			modparams.sf = (lr11xx_radio_lora_sf_t)radioconfig.sf;
			modparams.cr = (lr11xx_radio_lora_cr_t)radioconfig.cr;
			modparams.ldro = radioconfig.ldropt;
			
			lr11xx_radio_pkt_params_lora_t pktparams;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.header_type = (lr11xx_radio_lora_pkt_len_modes_t)radioconfig.header;
			pktparams.crc = (lr11xx_radio_lora_crc_t)radioconfig.crc;
			pktparams.iq = (lr11xx_radio_lora_iq_t)radioconfig.invertiq;
			
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
			lr11xx_radio_set_pkt_type(NULL,LR11XX_RADIO_PKT_TYPE_LORA);
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

		case 2021:
		{
						
			lr20xx_radio_lora_mod_params_t modparams;
			modparams.bw = (lr20xx_radio_lora_bw_t)LR202X_bw[radioconfig.bw_index];
			modparams.sf = (lr20xx_radio_lora_sf_t)radioconfig.sf;
			modparams.cr = (lr20xx_radio_lora_cr_t)radioconfig.cr;
			modparams.ppm = LR20XX_RADIO_LORA_NO_PPM; //LR20XX_RADIO_LORA_PPM_1_4
			
			lr20xx_radio_lora_pkt_params_t pktparams;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.pkt_mode = (lr20xx_radio_lora_pkt_mode_t)radioconfig.header; //???
			pktparams.crc = (lr20xx_radio_lora_crc_t)radioconfig.crc;
			pktparams.iq = (lr20xx_radio_lora_iq_t)radioconfig.invertiq;
			

			
			radio_set_freq(radioconfig.freq / 1000);
			radio_set_power(radioconfig.txpower);
			

			/* in 0.5dBm steps, range [-19, +44] for sub-G, range [-38, 24] for 2.4G ( HF_PA ) */
			//lr20xx_radio_common_set_tx_params(NULL, TX_OUTPUT_POWER_HALF_DBM_STEPS, LR20XX_RADIO_COMMON_RAMP_32_US);
			lr20xx_radio_common_set_tx_params(NULL,radioconfig.txpower*2, LR20XX_RADIO_COMMON_RAMP_32_US);
			//lr20xx_radio_common_set_rx_path( NULL, rx_path, LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE );
			
			lr20xx_radio_common_set_rx_tx_fallback_mode( NULL, LR20XX_RADIO_FALLBACK_STDBY_XOSC );	
			lr20xx_radio_lora_set_modulation_params(NULL, &modparams);
			lr20xx_radio_lora_set_packet_params(NULL, &pktparams);
			lr20xx_radio_lora_set_syncword(NULL, radioconfig.sync);
			////////////////////
			//radio_set_power(radioconfig.txpower);
			radio_rx();
			return RADIO_OK;
		}
				
		case 3029:
		

		PAN_SetFreq(radioconfig.freq);          /* Set the frequency */
		PAN_SetBW(PAN_bw[radioconfig.bw_index]);              /* Set the bandwidth */
		PAN_SetSF(radioconfig.sf);              /* Set the spreading factor */
		PAN_SetCR(radioconfig.cr);              /* Set the channel coding rate */
		PAN_SetCRC(radioconfig.crc);            /* Set the CRC check. Disable for regular LoRa compatibility! */
		PAN_SetLDR(radioconfig.ldropt);            /* Set the low-rate mode */
		PAN_SetPreamLen(radioconfig.prelen);  /* Set the preamble length */
		PAN_SetInvertIQ(radioconfig.invertiq); /* Set IQ to non-inverted */
		PAN_SetSyncWord(radioconfig.sync & 0xff);
		PAN_SetRegulatorMode(USE_LDO);         /* Set the chip to LDO power mode */
		//PAN_SetChipMode(CHIPMODE_MODE0);
		//PAN_SetChipMode(CHIPMODE_MODE1);       /* Set the chip mode to MODE1 */
		//enable interrupts
		PAN_SetPageRegBits(0,0x58,PAN_IRQ_TX_DONE | PAN_IRQ_RX_DONE | PAN_IRQ_CRC_ERR);
		//PAN_SetPageRegBits(0,0x58,0xff);
		//STB3 now
		radio_rx();
		return RADIO_OK;
		
    default:
    return INVALID_CHIP;
	}
}