#include "radio_func.h"

//globals
int8_t minpower;
int8_t maxpower;

uint32_t currfreq;
uint32_t prevfreq;

//buffers
uint8_t radio_txbuffer[RADIO_TXBUF_SIZE];
uint8_t txlen;
uint8_t radio_rxbuffer[RADIO_RXBUF_SIZE];
uint8_t rxlen;

bool packet_sent = false;
bool packet_received = false;
bool crc_error = false; 

//radio configuration structure
radioconfig_t radioconfig;
rxpacketstatus_t pktstatus;

//init radio
int8_t radio_initconfig(uint16_t chip,uint8_t tcxo)
{
  switch(chip)
  {
    case 1262:
    radioconfig.chip = 1262;
    radioconfig.id = 0;
    radioconfig.freq = 433125000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = SX126X_LORA_SF11;
    radioconfig.bw_index = 8; //LORA_BW_250;
    radioconfig.cr = SX126X_LORA_CR_4_5;
    radioconfig.ldropt = 0;
    //packet
    radioconfig.sync = 0x2b; //0x24b4;
    radioconfig.prelen = 16;
    radioconfig.header = 0;
    radioconfig.paylen = 0;
    radioconfig.crc = 1;
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
		if(tcxo) 
		{
			sx126x_tcxo = 1;
			sx126x_tcxo_voltage = 2; //1.8V
		}
		else sx126x_tcxo = 0;
		break;
		
		case 1280:
    radioconfig.chip = 1280;
    radioconfig.id = 0;
    radioconfig.freq = 2400000000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = SX128X_LORA_RANGING_SF11;
    radioconfig.bw_index = 0; //SX128X_LORA_BW_200
    radioconfig.cr = SX128X_LORA_RANGING_CR_4_5;
    radioconfig.ldropt = 0; //not used in 1280
    //packet
    radioconfig.sync = 0x2b; //0x24b4;
    radioconfig.prelen = 0x23; //len = 12. mant = 3, exp = 2
    radioconfig.header = 0;
    radioconfig.paylen = 0;
    radioconfig.crc = 1;
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
//		if(tcxo) 
//		{
//			sx128x_tcxo = 1;
//			sx128x_tcxo_voltage = 2; //1.8V
//		}
//		else sx128x_tcxo = 0;
		break;

    case 1121:
    radioconfig.chip = 1121;
    radioconfig.id = 0;
    radioconfig.freq = 433125000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = LR11XX_RADIO_LORA_SF11;
    radioconfig.bw_index = 8; //LORA_BW_250;
    radioconfig.cr = LR11XX_RADIO_LORA_CR_4_5;
    radioconfig.ldropt = 0;
    //packet
    radioconfig.sync = 0x2b; //0x24b4;
    radioconfig.prelen = 16;
    radioconfig.header = 0;
    radioconfig.paylen = 0;
    radioconfig.crc = 1;
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
		if(tcxo) 
		{
			lr112x_tcxo = 1;
			lr112x_tcxo_voltage = 2; //1.8V
		}
		else lr112x_tcxo = 0;
		break;
		
    case 2021:
    radioconfig.chip = 2021;
    radioconfig.id = 0;
    radioconfig.freq = 433125000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = LR20XX_RADIO_LORA_SF11;
    radioconfig.bw_index = 8; //LORA_BW_250;
    radioconfig.cr = LR20XX_RADIO_LORA_CR_4_5;
    radioconfig.ldropt = 0;
    //packet
    radioconfig.sync = 0x2b; //0x24b4;
    radioconfig.prelen = 16;
    radioconfig.header = 0;
    radioconfig.paylen = 0;
    radioconfig.crc = 1;
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
		if(tcxo) 
		{
			lr202x_tcxo = 1;
			lr202x_tcxo_voltage = 2; //1.8V
		}
		else lr202x_tcxo = 0;
		break;
		
    case 3029:
    radioconfig.chip = 3029;
    radioconfig.id = 0;
    radioconfig.freq = 433125000;
    radioconfig.txpower = 10;
    //modulation
    radioconfig.sf = LORA_SF_11;
    radioconfig.bw_index = 8; //LORA_BW_250;
    radioconfig.cr = LORA_CR_4_5;
    radioconfig.ldropt = 0;
    //packet
    radioconfig.sync = 0x2b;
    radioconfig.prelen = 16;
    radioconfig.header = 0;
    radioconfig.paylen = 255; //???
    //radioconfig.crc = 1;
		radioconfig.crc = 0; //mandatory for SX126x compatibility
    radioconfig.invertiq = 0;
    //params[64]; //maybe different
//		if(tcxo) 
//		{
//			sx126x_tcxo = 1;
//			sx126x_tcxo_voltage = 2; //1.8V
//		}
//		else sx126x_tcxo = 0;
		break;

    default:
    return INVALID_CHIP;
  }
	writeconfig();
	radio_init();
	return RADIO_OK;
}

int8_t radio_init(void)
{
  currfreq = radioconfig.freq / 1000;
	prevfreq = radioconfig.freq / 1000;
	switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_mod_params_lora_t modparams;
			modparams.bw = SX126X_bw[radioconfig.bw_index];
			modparams.cr = radioconfig.cr;
			modparams.sf = radioconfig.sf;
			modparams.ldro = radioconfig.ldropt;
			
			sx126x_pkt_params_lora_t pktparams;
			pktparams.header_type = radioconfig.header;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			
			sx126x_pa_cfg_params_t paconfig;
			paconfig.device_sel = 0;
			paconfig.pa_duty_cycle = 4;
			paconfig.hp_max = 7;
			paconfig.pa_lut = 1;
			
			sx126x_reset(NULL);
			sx126x_wakeup(NULL);
			delay_ms(10);
			SX126X_setopmode(RADIO_OPMODE_STBYRC); //check
			sx126x_set_reg_mode(NULL,SX126X_REG_MODE_DCDC);
			//set TCXO here if needed
			if(sx126x_tcxo != 0)
			{
				sx126x_set_dio3_as_tcxo_ctrl(NULL,sx126x_tcxo_voltage,1000); //check
				sx126x_cal(NULL,SX126X_CAL_ALL);
			}
			sx126x_set_pkt_type(NULL,SX126X_PKT_TYPE_LORA);
			sx126x_set_rf_freq(NULL,radioconfig.freq);
			sx126x_set_buffer_base_address(NULL,0,0);
			sx126x_set_lora_mod_params(NULL,&modparams);
			sx126x_set_lora_pkt_params(NULL,&pktparams);
			sx126x_set_lora_sync_word(NULL,radioconfig.sync & 0xff);
			//sx126x_write_register(NULL,SX126X_REG_LR_SYNCWORD,(radioconfig.sync & 0xff00) >> 8);
			//sx126x_write_register(NULL,SX126X_REG_LRSYNC_L,radioconfig.sync & 0xff);
			sx126x_set_rx(NULL,0xffffff);
			//SX126X_setopmode(OPMODE_STBYXOSC);
			SX126X_CalibrateIR();
			if(sx126x_tcxo == 0) //TCXO off
			{
				SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
				sx126x_set_trimming_capacitor_values(NULL,sx126x_xtatrim,sx126x_xtbtrim);
			}
    //SX126X_SetTxParams();
		sx126x_set_tx_params(NULL,radioconfig.txpower,SX126X_RAMP_10_US);
		sx126x_set_pa_cfg(NULL,&paconfig);
    sx126x_cfg_rx_boosted(NULL,true);
    sx126x_set_dio2_as_rf_sw_ctrl(NULL,true);
    sx126x_set_dio_irq_params(NULL,SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR,SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE,SX126X_IRQ_NONE,SX126X_IRQ_NONE);
    //sx126x_set_dio_irq_params(NULL,SX126X_IRQ_ALL,SX126X_IRQ_ALL,SX126X_IRQ_NONE,SX126X_IRQ_NONE);
		SX126X_setopmode(RADIO_OPMODE_RX);
    return RADIO_OK;
		}
		
		case 1280:
		{
			sx128x_mod_params_lora_t modparams;
			modparams.bw = SX128X_bw[radioconfig.bw_index];
			modparams.sf = radioconfig.sf << 4;
			modparams.cr = radioconfig.cr;
			
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
			
			sx128x_reset(NULL);
			sx128x_wakeup(NULL);
			//SX128X_setopmode(SX128X_OPMODE_STBYRC);
			sx128x_set_reg_mode(NULL,SX128X_REG_MODE_DCDC);
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
			modparams.bw = LR112X_bw[radioconfig.bw_index];
			modparams.sf = radioconfig.sf;
			modparams.cr = radioconfig.cr;
			modparams.ldro = radioconfig.ldropt;
			
			lr11xx_radio_pkt_params_lora_t pktparams;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.header_type = radioconfig.header;
			pktparams.crc = radioconfig.crc;
			pktparams.iq = radioconfig.invertiq;
			
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
			return RADIO_TODO;
		}
				
		case 3029:
		if(PAN_Init(radioconfig.freq) != PAN_OK) return RADIO_INVALID_MODE; //in STB3 state
		//PAN_SetTxPower(radioconfig.txpower);                    /* Set the power level */
		PAN_setpower(radioconfig.txpower);
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


//set RF frequency
int8_t radio_set_freq(uint32_t khz)
{
  currfreq = khz;
	switch(radioconfig.chip)
  {
    case 1262:
		radioconfig.freq = khz * 1000;
		prevopmode = opmode;
		SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
		sx126x_set_rf_freq(NULL,radioconfig.freq);
		SX126X_setopmode(prevopmode);
    return RADIO_OK;
		
		case 1280:
		radioconfig.freq = khz * 1000;
		prevopmode = opmode;
		SX128X_setopmode(RADIO_OPMODE_STBYXOSC);
		sx128x_set_rf_freq(NULL,radioconfig.freq);
		SX128X_setopmode(prevopmode);
		return RADIO_OK;
		
		case 1121:
		{
			lr11xx_radio_pa_cfg_t pa_cfg;
			
			radioconfig.freq = khz * 1000;
			prevopmode = opmode;
			LR112X_setopmode(RADIO_OPMODE_STBYXOSC);
//#define LR11XX_PA_DUTYCYCLE_SUBG		0x04
//#define LR11XX_PA_HPSEL_SUBG				0x07

//#define LR11XX_PA_DUTYCYCLE_HF			0x04
//#define LR11XX_PA_HPSEL_HF					0x00
			if(radioconfig.freq > LR112X_SEPARATION_FREQ)
			{
				pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_HF;
				pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
				pa_cfg.pa_duty_cycle = 4;
				pa_cfg.pa_hp_sel = 0;
			}
			else
			{
				pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_LP; //LR11XX_RADIO_PA_SEL_HP
				pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG; //LR11XX_RADIO_PA_REG_SUPPLY_VBAT
				pa_cfg.pa_duty_cycle = 4;
				pa_cfg.pa_hp_sel = 7;
			}
			lr11xx_radio_set_pa_cfg(NULL,&pa_cfg);
			lr11xx_radio_set_rf_freq(NULL,radioconfig.freq);
			LR112X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		//prevopmode = opmode;
		radioconfig.freq = khz * 1000;
		if(PAN_SetFreq(radioconfig.freq) == PAN_FAIL) return RADIO_INVALID_PARAMETER; 
		return RADIO_OK;
    
    default:
    return INVALID_CHIP;
  }
}

//set tx power
int8_t radio_set_power(int8_t dbm)
{
  switch(radioconfig.chip)
  {
    case 1262:
		if((dbm < -9) || (dbm > 22)) return RADIO_INVALID_PARAMETER;
		prevopmode = opmode;
		SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
		radioconfig.txpower = dbm;
		sx126x_set_tx_params(NULL,radioconfig.txpower,SX126X_RAMP_10_US);
		SX126X_setopmode(prevopmode);
    return RADIO_OK;
		
		case 1280:
		{
			int8_t pwr;
			if((dbm < -18) || (dbm > 13)) return RADIO_INVALID_PARAMETER;
			//prevopmode = opmode;
			//SX128X_setopmode(RADIO_OPMODE_STBYXOSC);
			radioconfig.txpower = dbm;
			pwr = dbm + 18;
			printf("dbm=%d,pwr=%d\r\n",dbm,pwr);
			sx128x_set_tx_params(NULL,radioconfig.txpower,SX128X_RAMP_02_US);
			//SX128X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
		case 1121:
		{
			lr11xx_radio_pa_cfg_t pa_cfg;
			//prevopmode = opmode;
			//LR112X_setopmode(RADIO_OPMODE_STBYXOSC);
			if(radioconfig.freq > LR112X_SEPARATION_FREQ)
			{
				//check value
				if((dbm < -18) || (dbm > 13)) return RADIO_INVALID_PARAMETER;
				radioconfig.txpower = dbm;
				pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_HF;
				pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
				pa_cfg.pa_duty_cycle = 4;
				pa_cfg.pa_hp_sel = 0;
			}
			else
			{
				//check value
				if((dbm < -9) || (dbm > 22)) return RADIO_INVALID_PARAMETER;
				pa_cfg.pa_sel = LR11XX_RADIO_PA_SEL_LP; //LR11XX_RADIO_PA_SEL_HP
				pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG; //LR11XX_RADIO_PA_REG_SUPPLY_VBAT
				pa_cfg.pa_duty_cycle = 4;
				pa_cfg.pa_hp_sel = 7;
				radioconfig.txpower = dbm;
			}
			lr11xx_radio_set_pa_cfg(NULL,&pa_cfg);
			lr11xx_radio_set_tx_params(NULL,dbm,LR11XX_RADIO_RAMP_16_US);
			//LR112X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		if(dbm < 0) return RADIO_INVALID_PARAMETER;
		if(dbm > 20) return RADIO_INVALID_PARAMETER;
		radioconfig.txpower = dbm;
		PAN_setpower(dbm);
		return RADIO_OK;
		
    default:
    return INVALID_CHIP;
  }
}

//set modulation parameters
int8_t radio_setmodparams(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			uint8_t bw_index;
			sx126x_mod_params_lora_t modparams;
			//BW in kHz
			if((bw_khz < 7) || (bw_khz > 500)) return RADIO_INVALID_PARAMETER;
			if((sf < 5) || (sf > 12)) return RADIO_INVALID_PARAMETER;
			if(cr > 4) return RADIO_INVALID_PARAMETER;
			if(ldropt > 1) return RADIO_INVALID_PARAMETER;
			if(bw_khz <= 8) bw_index = 0;
			else if(bw_khz <= 11) bw_index = 1;
			else if(bw_khz <= 16) bw_index = 2;
			else if(bw_khz <= 21) bw_index = 3;
			else if(bw_khz <= 32) bw_index = 4;
			else if(bw_khz <= 42) bw_index = 5;
			else if(bw_khz <= 63) bw_index = 6;
			else if(bw_khz <= 125) bw_index = 7;
			else if(bw_khz <= 250) bw_index = 8;
			else bw_index = 9;
			radioconfig.bw_index = bw_index;
			radioconfig.sf = sf;
			radioconfig.cr = cr;
			radioconfig.ldropt = ldropt;
			prevopmode = opmode;
			SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
			modparams.bw = SX126X_bw[radioconfig.bw_index];
			modparams.cr = radioconfig.cr;
			modparams.sf = radioconfig.sf;
			modparams.ldro = radioconfig.ldropt;
			sx126x_set_lora_mod_params(NULL,&modparams);
			SX126X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
    case 1280:
		{
			uint8_t bw_index;
			sx128x_mod_params_lora_t modparams;
			//BW in kHz
			if((bw_khz < 7) || (bw_khz > 500)) return RADIO_INVALID_PARAMETER;
			if((sf < 5) || (sf > 12)) return RADIO_INVALID_PARAMETER;
			if(cr > 8) return RADIO_INVALID_PARAMETER;
			if(ldropt > 1) return RADIO_INVALID_PARAMETER;
			if(bw_khz <= 204) bw_index = 0;
			else if(bw_khz <= 407) bw_index = 1;
			else if(bw_khz <= 813) bw_index = 2;
			else bw_index = 3;
			radioconfig.bw_index = bw_index;
			radioconfig.sf = sf;
			radioconfig.cr = cr;
			//radioconfig.ldropt = ldropt;
			prevopmode = opmode;
			SX128X_setopmode(RADIO_OPMODE_STBYXOSC);
			modparams.bw = SX128X_bw[radioconfig.bw_index];
			modparams.sf = radioconfig.sf << 4;
			modparams.cr = radioconfig.cr;
			sx128x_set_lora_mod_params(NULL,&modparams);
			SX128X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
    case 1121:
		{
			uint8_t bw_index;
			lr11xx_radio_mod_params_lora_t modparams;
			//BW in kHz
			if((bw_khz < 200) || (bw_khz > 1625)) return RADIO_INVALID_PARAMETER;
			if((sf < 5) || (sf > 12)) return RADIO_INVALID_PARAMETER;
			if(cr > 8) return RADIO_INVALID_PARAMETER;
			//if(ldropt > 1) return RADIO_INVALID_PARAMETER;
			if(bw_khz <= 8) bw_index = 0;
			else if(bw_khz <= 11) bw_index = 1;
			else if(bw_khz <= 16) bw_index = 2;
			else if(bw_khz <= 21) bw_index = 3;
			else if(bw_khz <= 32) bw_index = 4;
			else if(bw_khz <= 42) bw_index = 5;
			else if(bw_khz <= 63) bw_index = 6;
			else if(bw_khz <= 125) bw_index = 7;
			else if(bw_khz <= 204) bw_index = 10;
			else if(bw_khz <= 251) bw_index = 8;
			else if(bw_khz <= 407) bw_index = 11;
			else if(bw_khz <= 501) bw_index = 9;
			else bw_index = 12;
			radioconfig.bw_index = bw_index;
			radioconfig.sf = sf;
			radioconfig.cr = cr;
			radioconfig.ldropt = ldropt;
			prevopmode = opmode;
			LR112X_setopmode(RADIO_OPMODE_STBYXOSC);
			modparams.bw = LR112X_bw[radioconfig.bw_index];
			modparams.sf = radioconfig.sf;
			modparams.cr = radioconfig.cr;
			modparams.ldro = radioconfig.ldropt;
			lr11xx_radio_set_lora_mod_params(NULL,&modparams);
			LR112X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		{
			uint8_t bw_index;
			//BW in kHz
			if((bw_khz < 7) || (bw_khz > 500)) return RADIO_INVALID_PARAMETER;
			if((sf < 5) || (sf > 12)) return RADIO_INVALID_PARAMETER;
			if(cr > 4) return RADIO_INVALID_PARAMETER;
			if(ldropt > 1) return RADIO_INVALID_PARAMETER;
			if(bw_khz <= 8) bw_index = 0;
			else if(bw_khz <= 11) bw_index = 1;
			else if(bw_khz <= 16) bw_index = 2;
			else if(bw_khz <= 21) bw_index = 3;
			else if(bw_khz <= 32) bw_index = 4;
			else if(bw_khz <= 42) bw_index = 5;
			else if(bw_khz <= 63) bw_index = 6;
			else if(bw_khz <= 125) bw_index = 7;
			else if(bw_khz <= 250) bw_index = 8;
			else bw_index = 9;
			radioconfig.bw_index = bw_index;
			radioconfig.sf = sf;
			radioconfig.cr = cr;
			radioconfig.ldropt = ldropt;
			//to STB3
			PAN_setopmode(RADIO_OPMODE_STBYXOSC);
			PAN_SetBW(PAN_bw[radioconfig.bw_index]);              /* Set the bandwidth */
			PAN_SetSF(radioconfig.sf);              /* Set the spreading factor */
			PAN_SetCR(radioconfig.cr);              /* Set the channel coding rate */
			PAN_SetLDR(radioconfig.ldropt);            /* Set the low-rate mode */
			//return to prev.mode
			PAN_setopmode(prevopmode); 
			return RADIO_OK;
		}
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_getmodparams(uint16_t *bw_khz,uint8_t *sf,uint8_t *cr,uint8_t *ldropt)
{
  switch(radioconfig.chip)
  {
    case 1262:
		*sf = radioconfig.sf;
		*bw_khz = SX126X_bw_kHz[radioconfig.bw_index];
		*cr = radioconfig.cr;
		*ldropt = radioconfig.ldropt;
		return RADIO_OK;
		
    case 1280:
		*sf = radioconfig.sf;
		*bw_khz = SX128X_bw_kHz[radioconfig.bw_index];
		*cr = radioconfig.cr;
		*ldropt = 0;
		return RADIO_OK;
		
    case 1121:
		*sf = radioconfig.sf;
		*bw_khz = LR112X_bw_kHz[radioconfig.bw_index];
		*cr = radioconfig.cr;
		*ldropt = radioconfig.ldropt;
		return RADIO_OK;
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		*sf = radioconfig.sf;
		*bw_khz = PAN_bw_kHz[radioconfig.bw_index];
		*cr = radioconfig.cr;
		*ldropt = radioconfig.ldropt;
		return RADIO_OK;
		
		default:
    return INVALID_CHIP;
  }
}

//set packet parameters
int8_t radio_setpktparams(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_pkt_params_lora_t pktparams;
			if(header > 1) return RADIO_INVALID_PARAMETER;
			if(crc > 1) return RADIO_INVALID_PARAMETER;
			if(invertiq > 1) return RADIO_INVALID_PARAMETER;
			radioconfig.sync = sync;
			radioconfig.prelen = prelen;
			radioconfig.paylen = paylen;
			radioconfig.header = header;
			radioconfig.crc = crc;
			radioconfig.invertiq = invertiq;
			prevopmode = opmode;
			SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
			pktparams.header_type = radioconfig.header;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			sx126x_set_lora_pkt_params(NULL,&pktparams);
			sx126x_set_lora_sync_word(NULL,radioconfig.sync & 0xff);
			SX126X_setopmode(prevopmode);
			return RADIO_OK;
		}
    case 1280:
		{
			sx128x_pkt_params_lora_t pktparams;
			uint8_t m,e;
			if(header > 1) return RADIO_INVALID_PARAMETER;
			if(crc > 1) return RADIO_INVALID_PARAMETER;
			if(invertiq > 1) return RADIO_INVALID_PARAMETER;
			radioconfig.sync = sync;
			radioconfig.prelen = prelen;
			radioconfig.paylen = paylen;
			radioconfig.header = header;
			radioconfig.crc = crc;
			radioconfig.invertiq = invertiq;
			prevopmode = opmode;
			SX128X_setopmode(RADIO_OPMODE_STBYXOSC);
			SX128X_CalcPreamble(radioconfig.prelen,&m,&e);
			pktparams.preamble_len.mant = m;
			pktparams.preamble_len.exp = e;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			if(radioconfig.header == false) pktparams.header_type = SX128X_LORA_RANGING_PKT_EXPLICIT;
			else pktparams.header_type = SX128X_LORA_RANGING_PKT_IMPLICIT;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			sx128x_set_lora_pkt_params(NULL,&pktparams);
			//set sync
			sx128x_set_lora_sync_word(NULL,radioconfig.sync & 0xff);
			SX128X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
    case 1121:
		{
			lr11xx_radio_pkt_params_lora_t pktparams;
			if(header > 1) return RADIO_INVALID_PARAMETER;
			if(crc > 1) return RADIO_INVALID_PARAMETER;
			if(invertiq > 1) return RADIO_INVALID_PARAMETER;
			radioconfig.sync = sync;
			radioconfig.prelen = prelen;
			radioconfig.paylen = paylen;
			radioconfig.header = header;
			radioconfig.crc = crc;
			radioconfig.invertiq = invertiq;
			prevopmode = opmode;
			LR112X_setopmode(RADIO_OPMODE_STBYXOSC);
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.header_type = radioconfig.header;
			pktparams.crc = radioconfig.crc;
			pktparams.iq = radioconfig.invertiq;
			lr11xx_radio_set_lora_pkt_params(NULL,&pktparams);
			lr11xx_radio_set_lora_sync_word(NULL,radioconfig.sync & 0xff);
			LR112X_setopmode(prevopmode);
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		if(header > 1) return RADIO_INVALID_PARAMETER;
		if(crc > 1) return RADIO_INVALID_PARAMETER;
		if(invertiq > 1) return RADIO_INVALID_PARAMETER;
		radioconfig.sync = sync;
		radioconfig.prelen = prelen;
		radioconfig.paylen = paylen;
		radioconfig.header = header;
		radioconfig.crc = crc;
		radioconfig.invertiq = invertiq;
		//to STB3
		prevopmode = opmode;
		PAN_setopmode(RADIO_OPMODE_STBYXOSC); 
		PAN_SetCRC(radioconfig.crc);            /* Set the CRC check */
		PAN_SetPreamLen(radioconfig.prelen);  /* Set the preamble length */
		PAN_SetInvertIQ(radioconfig.invertiq); /* Set IQ to non-inverted */
		PAN_SetSyncWord(radioconfig.sync & 0xff);
		//return to prev.mode
		PAN_setopmode(prevopmode); 
		return RADIO_OK;
    
    default:
    return INVALID_CHIP;
  }
}

//send one packet
int8_t radio_sendpacket(uint8_t *buf)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_pkt_params_lora_t pktparams;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = txlen;
			pktparams.header_type = radioconfig.header;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			sx126x_set_lora_pkt_params(NULL,&pktparams);
			sx126x_write_buffer(NULL,0,buf,txlen);
			SX126X_setopmode(RADIO_OPMODE_TX);
			return RADIO_OK;
		}
		
    case 1280:
		{
			sx128x_pkt_params_lora_t pktparams;
			uint8_t m,e;
			SX128X_CalcPreamble(radioconfig.prelen,&m,&e);
			pktparams.preamble_len.mant = m;
			pktparams.preamble_len.exp = e;
			pktparams.pld_len_in_bytes = txlen;
			if(radioconfig.header == false) pktparams.header_type = SX128X_LORA_RANGING_PKT_EXPLICIT;
			else pktparams.header_type = SX128X_LORA_RANGING_PKT_IMPLICIT;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			sx128x_set_lora_pkt_params(NULL,&pktparams);
			sx128x_write_buffer(NULL,0,buf,txlen);
			SX128X_setopmode(RADIO_OPMODE_TX);
			return RADIO_OK;
		}
		
    case 1121:
		{
			lr11xx_radio_pkt_params_lora_t pktparams;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = txlen;
			pktparams.header_type = radioconfig.header;
			pktparams.crc = radioconfig.crc;
			pktparams.iq = radioconfig.invertiq;
			lr11xx_radio_set_lora_pkt_params(NULL,&pktparams);
			lr11xx_regmem_write_buffer8(NULL,buf,txlen);
			LR112X_setopmode(RADIO_OPMODE_TX);
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		//PAN_setopmode(RADIO_OPMODE_STBYXOSC);
		PAN_SetTx(buf, txlen);
		return RADIO_OK;
    
    default:
    return INVALID_CHIP;
  }
}
//retrieve packet info
int8_t radio_getpktstatus(rxpacketstatus_t *status)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_pkt_status_lora_t pktstatus;
			sx126x_get_lora_pkt_status(NULL,&pktstatus);
			status->rssi_pkt = pktstatus.rssi_pkt_in_dbm;
			status->snr_pkt = pktstatus.snr_pkt_in_db; 
			status->signal_rssi_pkt = pktstatus.signal_rssi_pkt_in_dbm;
			return RADIO_OK;
		}
		
    case 1280:
		{
			sx128x_pkt_status_lora_t pktstatus;
			sx128x_get_lora_pkt_status(NULL,&pktstatus);
			status->rssi_pkt = pktstatus.rssi; 
			status->snr_pkt = pktstatus.snr;
			status->signal_rssi_pkt = status->rssi_pkt + status->snr_pkt; //???
			return RADIO_OK;
		}
		
    case 1121:
		{
			lr11xx_radio_pkt_status_lora_t pktstatus;
			lr11xx_radio_get_lora_pkt_status(NULL,&pktstatus);
			status->rssi_pkt = pktstatus.rssi_pkt_in_dbm;
			status->snr_pkt = pktstatus.snr_pkt_in_db;
			status->signal_rssi_pkt = pktstatus.signal_rssi_pkt_in_dbm;
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		status->rssi_pkt = g_RfRxPkt.Rssi; //recompute
		status->snr_pkt = g_RfRxPkt.Snr; //recompute
		//status->signal_rssi_pkt = ?
		return RADIO_OK;		
    
    default:
    return INVALID_CHIP;
  }
}

//receive one packet
int8_t radio_getpacket(uint8_t *buf)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_rx_buffer_status_t status;
			sx126x_get_rx_buffer_status(NULL,&status);
			rxlen = status.pld_len_in_bytes;
			sx126x_read_buffer(NULL,status.buffer_start_pointer,buf,rxlen);
			return RADIO_OK;
		}
		
    case 1280:
		{
			sx128x_rx_buffer_status_t status;
			sx128x_get_rx_buffer_status(NULL,&status);
			rxlen = status.pld_len_in_bytes;
			sx128x_read_buffer(NULL,status.buffer_start_pointer,buf,rxlen);
			return RADIO_OK;
		}
		
    case 1121:
		{
			lr11xx_radio_rx_buffer_status_t status;
			lr11xx_radio_get_rx_buffer_status(NULL,&status);
			rxlen = status.pld_len_in_bytes;
			lr11xx_regmem_read_buffer8(NULL,buf,status.buffer_start_pointer,rxlen);
			lr11xx_regmem_clear_rxbuffer(NULL); //???
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		rxlen = g_RfRxPkt.RxLen;
		memcpy((void*)buf,(void*)g_RfRxPkt.RxBuf,rxlen);
		return RADIO_OK;
		
		default:
    return INVALID_CHIP;
  }
}

//helpers
int8_t radio_getstats(rxstats_t *stats)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_stats_lora_t lora_stats;
			sx126x_get_lora_stats(NULL,&lora_stats);
			stats->pkt_received = lora_stats.nb_pkt_received;
			stats->crc_error = lora_stats.nb_pkt_crc_error;
			stats->header_error = lora_stats.nb_pkt_header_error;
			stats->false_sync = 0;
		}
    return RADIO_OK;
		
    case 1280:
    return FEATURE_NOT_SUPPORTED;
		
    case 1121:
		{
			lr11xx_radio_stats_lora_t lora_stats;
			lr11xx_radio_get_lora_stats(NULL,&lora_stats);
			stats->pkt_received = lora_stats.nb_pkt_received;
			stats->crc_error = lora_stats.nb_pkt_crc_error;
			stats->header_error = lora_stats.nb_pkt_falsesync;
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_clearstats(void)
{
  switch(radioconfig.chip)
  {
    case 1262:
		sx126x_reset_stats(NULL);
    return RADIO_OK;
		
    case 1280:
    return FEATURE_NOT_SUPPORTED;
		
    case 1121:
    lr11xx_radio_reset_stats(NULL);
    return RADIO_OK;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

//irq handler
void radio_irq_handler(void)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			//read SX126x status
			uint16_t irqstatus;
			sx126x_get_irq_status(NULL,&irqstatus);
			sx126x_clear_irq_status(NULL,SX126X_IRQ_ALL);
			//printf("Flags:0x%02X\r\n",irqstatus);
			if(irqstatus & SX126X_IRQ_TX_DONE) packet_sent = true;
			if(irqstatus & SX126X_IRQ_RX_DONE) packet_received = true;
			if(irqstatus & SX126X_IRQ_CRC_ERROR) crc_error = true;
//			if(irqstatus & SX126X_IRQ_PREAMBLE_DETECTED) {};
//			if(irqstatus & SX126X_IRQ_SYNC_WORD_VALID) {};
//			if(irqstatus & SX126X_IRQ_HEADER_VALID) {};
//			if(irqstatus & SX126X_IRQ_HEADER_ERROR) {};
//			if(irqstatus & SX126X_IRQ_CAD_DONE) {};
//			if(irqstatus & SX126X_IRQ_CAD_DETECTED) {};
//			if(irqstatus & SX126X_IRQ_TIMEOUT) {};
//			if(irqstatus & SX126X_IRQ_LR_FHSS_HOP) {};
			break;   
		}
		
    case 1280:
		{
			//read status
			uint16_t irqstatus;
			sx128x_get_irq_status(NULL,&irqstatus);
			sx128x_clear_irq_status(NULL,SX128X_IRQ_ALL);
			if(irqstatus & SX128X_IRQ_TX_DONE) packet_sent = true;
			if(irqstatus & SX128X_IRQ_RX_DONE) packet_received = true;
			if(irqstatus & SX128X_IRQ_CRC_ERROR) crc_error = true;
//			if(irqstatus & SX128X_IRQ_SYNCWORD_VALID) {};
//			if(irqstatus & SX128X_IRQ_SYNCWORD_ERROR) {};
//			if(irqstatus & SX128X_IRQ_HEADER_VALID) {};
//			if(irqstatus & SX128X_IRQ_HEADER_ERROR) {};
//			if(irqstatus & SX128X_IRQ_RANGING_SLAVE_RESPONSE_DONE) {};
//			if(irqstatus & SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED) {};
//			if(irqstatus & SX128X_IRQ_RANGING_MASTER_RESULT_VALID) {};
//			if(irqstatus & SX128X_IRQ_RANGING_MASTER_RESULT_TIMEOUT) {};
//			if(irqstatus & SX128X_IRQ_RANGING_SLAVE_REQUEST_VALID) {};
//			if(irqstatus & SX128X_IRQ_CAD_DONE) {};
//			if(irqstatus & SX128X_IRQ_CAD_ACTIVITY_DETECTED) {};
//			if(irqstatus & SX128X_IRQ_RX_TX_TIMEOUT) {};
//			if(irqstatus & SX128X_IRQ_PREAMBLE_DETECTED) {};
			break;   
		}
		
    case 1121:
		{
			//read status
			uint32_t irqstatus;
			lr11xx_system_get_irq_status(NULL,&irqstatus);
			lr11xx_system_clear_irq_status(NULL,LR11XX_SYSTEM_IRQ_ALL_MASK);
			//printf("IRQ:0x%08X\r\n",irqstatus);
			if(irqstatus & LR11XX_SYSTEM_IRQ_TX_DONE) packet_sent = true;
			if(irqstatus & LR11XX_SYSTEM_IRQ_RX_DONE) packet_received = true;
			if(irqstatus & LR11XX_SYSTEM_IRQ_CRC_ERROR) crc_error = true;
//			if(irqstatus & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED) {};
//			if(irqstatus & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) {};
//			if(irqstatus & LR11XX_SYSTEM_IRQ_HEADER_ERROR) {};
//			if(irqstatus & LR11XX_SYSTEM_IRQ_CAD_DONE) {};
//			if(irqstatus & LR11XX_SYSTEM_IRQ_CAD_DETECTED) {};
//			if(irqstatus & LR11XX_SYSTEM_IRQ_TIMEOUT) {};
//			if(irqstatus & LR11XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP) {};
//			if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_REQ_VALID) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT) {};		
//			if(irqstatus & LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE) {};		
//			if(irqstatus & LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_EOL) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_CMD_ERROR) {};		
//			if(irqstatus & LR11XX_SYSTEM_IRQ_ERROR) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_FSK_ADDR_ERROR) {};	
//			if(irqstatus & LR11XX_SYSTEM_IRQ_LORA_RX_TIMESTAMP) {};		
			break;   
		}
		
		case 2021:
		break;
		
		case 3029:
		PAN_irq_handler();
		break;
		
    default:
    break;
  }
}

//working modes
int8_t radio_rx(void)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_pkt_params_lora_t pktparams;
			pktparams.header_type = radioconfig.header;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			sx126x_set_lora_pkt_params(NULL,&pktparams);
			SX126X_setopmode(RADIO_OPMODE_RX);
			return RADIO_OK;
		}
		
    case 1280:
		{
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
			sx128x_set_lora_pkt_params(NULL,&pktparams);
			SX128X_setopmode(RADIO_OPMODE_RX);
			return RADIO_OK;
		}
		
    case 1121:
		{
			lr11xx_radio_pkt_params_lora_t pktparams;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.header_type = radioconfig.header;
			pktparams.crc = radioconfig.crc;
			pktparams.iq = radioconfig.invertiq;
			lr11xx_radio_set_lora_pkt_params(NULL,&pktparams);
			LR112X_setopmode(RADIO_OPMODE_RX);
			return RADIO_OK;
		}
		
		case 2021:
		{
			return RADIO_TODO;
		}
		
		case 3029:
		PAN_EnterContinousRxState();
		return RADIO_OK;
		
		default:
    return INVALID_CHIP;
  }
}

int8_t radio_getrssi(float *dbm)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			int16_t rssi;
			sx126x_get_rssi_inst(NULL,&rssi);
			*dbm = rssi;
			return RADIO_OK;
		}
		
    case 1280:
		{
			int16_t rssi;
			sx128x_get_rssi_inst(NULL,&rssi);
			*dbm = rssi;
			return RADIO_OK;
		}
		
    case 1121:
		{
			int8_t rssi;
			lr11xx_radio_get_rssi_inst(NULL,&rssi);
			*dbm = rssi;
			return RADIO_OK;
		}
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		*dbm = (float)PAN_GetRealTimeRssi();
		return RADIO_OK;
   
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_stream(uint8_t stream)
{
  if(stream == 0) 
	{
		txmode = 0;
		txled_off();
		if(radioconfig.chip == 3029) 
		{
			PAN_StopTxContinuousWave();
		}
		return radio_rx();
	}
  switch(radioconfig.chip)
  {
    case 1262:
    if(stream > 2) return RADIO_INVALID_PARAMETER;
		//if(txmode != 0) return RADIO_INVALID_MODE;
		//prevopmode = opmode;
    if(stream == 1) 
		{
			sx126x_set_tx_cw(NULL);
			txmode = 1;
		}
    else 
		{
			sx126x_set_tx_infinite_preamble(NULL);
			txmode = 2;
		}
		txled_on();
    return RADIO_OK;
		
    case 1280:
    if(stream > 2) return RADIO_INVALID_PARAMETER;
		//if(txmode != 0) return RADIO_INVALID_MODE;
		//prevopmode = opmode;
    if(stream == 1) 
		{
			sx128x_set_tx_cw(NULL);
			txmode = 1;
		}
    else 
		{
			sx128x_set_tx_infinite_preamble(NULL);
			txmode = 2;
		}
		txled_on();
    return RADIO_OK;
		
    case 1121:
    if(stream > 2) return RADIO_INVALID_PARAMETER;
		//if(txmode != 0) return RADIO_INVALID_MODE;
		//prevopmode = opmode;
    if(stream == 1) 
		{
			lr11xx_radio_set_tx_cw(NULL);
			txmode = 1;
		}
    else 
		{
			lr11xx_radio_set_tx_infinite_preamble(NULL);
			txmode = 2;
		}
		txled_on();
    return RADIO_OK;
		
		case 2021:
		return RADIO_TODO;
		
		case 3029:
		if(stream == 1)
		{
			PAN_TurnonTxAnt();
			PAN_StartTxContinuousWave();
			txmode = 1;
			txled_on();
			return RADIO_OK;
		}
		else return RADIO_INVALID_PARAMETER;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_sleep(uint8_t node)
{
  switch(radioconfig.chip)
  {
    case 1262:
    return RADIO_TODO;
		
    case 1280:
    return RADIO_TODO;
		
    case 1121:
    return RADIO_TODO;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_wakeup(uint8_t mode)
{
  switch(radioconfig.chip)
  {
    case 1262:
    return RADIO_TODO;
		
    case 1280:
    return RADIO_TODO;
		
    case 1121:
    return RADIO_TODO;
		
		case 3029:
		return RADIO_TODO;
    
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_setxotrim(uint8_t trim)
{
  switch(radioconfig.chip)
  {
    case 1262:
		if(trim > 94) return RADIO_INVALID_PARAMETER;
		sx126x_xtatrim = trim / 2;
		sx126x_xtbtrim = trim - sx126x_xtatrim;
		prevopmode = opmode;
		SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
		sx126x_set_trimming_capacitor_values(NULL,sx126x_xtatrim,sx126x_xtbtrim);
		SX126X_setopmode(prevopmode);
		return RADIO_OK;
		
    case 1280: //0x093c
		return RADIO_TODO;
		
    case 1121:
    return FEATURE_NOT_SUPPORTED;
		
		case 2021:
    return FEATURE_NOT_SUPPORTED;
		
		case 3029:
		return FEATURE_NOT_SUPPORTED;

    default:
    return INVALID_CHIP;
  }
}

int8_t radio_getxotrim(uint8_t *trim)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			uint8_t buf[2];
			sx126x_read_register(NULL,SX126X_REG_XTATRIM,buf,2);
			*trim = buf[0] + buf[1];
			return RADIO_OK;
		}
		
    case 1280: //0x093c
		return RADIO_TODO;
		
    case 1121:
    return FEATURE_NOT_SUPPORTED;
		
		case 3029:
		return RADIO_TODO;

    default:
    return INVALID_CHIP;
  }
}

int8_t radio_readreg(uint32_t reg,uint32_t *val)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			uint8_t regval;
			if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
			sx126x_read_register(NULL,reg,&regval,1);
			*val = regval;
			return RADIO_OK;
		}
		
    case 1280:
		{
			uint8_t regval;
			if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
			sx128x_read_register(NULL,reg,&regval,1);
			*val = regval;
			return RADIO_OK;
		}
		
    case 1121:
		{
			uint32_t regval;
			if(reg > 0xffffff) return RADIO_INVALID_PARAMETER; //24 bits - ???
			lr11xx_regmem_read_regmem32(NULL,reg,&regval,1);
			return RADIO_OK;
		}
		
		case 2021:
		{
			uint32_t regval;
			if(reg > 0xffffff) return RADIO_INVALID_PARAMETER; //24 bits - ???
			lr20xx_regmem_read_regmem32(NULL,reg,&regval,1);
			return RADIO_OK;
		}
		
		case 3029:
		{
			*val = PAN_ReadReg(reg);
			return RADIO_OK;
		}

    default:
    return INVALID_CHIP;
  }
}

int8_t radio_writereg(uint32_t reg,uint32_t val)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			uint8_t regval = val & 0xff;
			if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
			if(val > 0xff) return RADIO_INVALID_PARAMETER;
			sx126x_write_register(NULL,reg,&regval,1);
			return RADIO_OK;
		}
		
    case 1280:
		{
			uint8_t regval = val & 0xff;
			if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
			if(val > 0xff) return RADIO_INVALID_PARAMETER;
			sx128x_write_register(NULL,reg,&regval,1);
			return RADIO_OK;
		}
		
    case 1121:
		if(reg > 0xffffff) return RADIO_INVALID_PARAMETER; //24 bits - ???
		lr11xx_regmem_write_regmem32(NULL,reg,&val,1);
		return RADIO_OK;
		
		case 2021:
		if(reg > 0xffffff) return RADIO_INVALID_PARAMETER; //24 bits - ???
		lr20xx_regmem_write_regmem32(NULL,reg,&val,1);
		return RADIO_OK;	
		
		case 3029:
		{
			uint8_t regval = val & 0xff;
			PAN_WriteReg(reg,regval);
			return RADIO_OK;
		}
		
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_get_chip_version(uint8_t *hw,uint8_t *use_case,uint8_t *fw_major,uint8_t *fw_minor)
{
  switch(radioconfig.chip)
  {
    case 1262:
		return RADIO_TODO;

    case 1280:
		{
			uint8_t buf[2];
			sx128x_read_register(NULL,SX128X_REG_FW_VERSION,buf,2);
			*hw = 0;
			*use_case = 0;
			*fw_major = buf[0];
			*fw_minor = buf[1];
			return RADIO_OK;
		}

    case 1121:
		{
			lr11xx_system_version_t version;
			lr11xx_system_get_version(NULL,&version);
			*hw = version.hw;
			*use_case = version.type;
			*fw_major = version.fw >> 8;
			*fw_minor = version.fw & 0xff;
			return RADIO_OK;
		}

    case 2021:
		{
			lr20xx_system_version_t version;
			lr20xx_system_get_version(NULL,&version);
			*hw = 0;
			*use_case = 0;
			*fw_major = version.major;
			*fw_minor = version.minor;
			return RADIO_OK;
		}
		
		case 3029:
		return RADIO_TODO;
		
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_get_status(uint8_t *chip_mode,uint8_t *cmd_status)
{
  switch(radioconfig.chip)
  {
		case 1262:
		{	
			sx126x_chip_status_t status;
			sx126x_get_status(NULL,&status);
			*chip_mode = status.chip_mode;
			*cmd_status = status.cmd_status;
			return RADIO_OK;
		}

    case 1280:
		{
			sx128x_chip_status_t status;
			sx128x_get_status(NULL,&status);
			*chip_mode = status.chip_mode;
			*cmd_status = status.cmd_status;
			return RADIO_OK;
		}

    case 1121:
		{
			lr11xx_system_stat1_t s1;
			lr11xx_system_stat2_t s2;
			lr11xx_system_get_status(NULL,&s1,&s2,NULL);
			*chip_mode = s2.chip_mode;
			*cmd_status = s1.command_status;
			return RADIO_OK;
		}
		
		case 2021:
		{
			lr20xx_system_stat1_t s1;
			lr20xx_system_stat2_t s2;
			lr20xx_system_get_status(NULL,&s1,&s2,NULL);
			*chip_mode = s2.chip_mode;
			*cmd_status = s1.command_status;
			return RADIO_OK;
		}
		
		case 3029:
		return RADIO_TODO;
		
    default:
    return INVALID_CHIP;
  }
}

uint8_t radio_setopmode(uint8_t mode)
{
  switch(radioconfig.chip)
  {
    case 1262:
		SX126X_setopmode(mode);
		return RADIO_OK;
		
    case 1280:
		SX128X_setopmode(mode);
		return RADIO_OK;
		
    case 1121:
		LR112X_setopmode(mode);
		opmode = mode;
		return RADIO_OK;
		
		case 3029:
		PAN_setopmode(mode);
		return RADIO_OK;
		
    default:
    return INVALID_CHIP;
  }
}



