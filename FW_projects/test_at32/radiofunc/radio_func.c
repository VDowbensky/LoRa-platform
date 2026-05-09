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
  radioconfig.id = 0;
	//modulation
	radioconfig.sf = SX126X_LORA_SF11;
	//radioconfig.bw = 250; //LORA_BW_250;
	radioconfig.cr = SX126X_LORA_CR_4_5;
	radioconfig.ldropt = 0;
	//packet
	radioconfig.sync = 0x2b; //0x24b4;
	radioconfig.prelen = 16;
	radioconfig.header = 0;
	radioconfig.paylen = 0;
	radioconfig.crc = 1;
	radioconfig.invertiq = 0;
	radioconfig.txpower = 10;
	
	switch(chip)
  {
    case 1262:
    radioconfig.chip = 1262;
		radioconfig.freq = 433125000;
		radioconfig.bw = 250; //LORA_BW_250;
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
    radioconfig.freq = 2400000000;
		radioconfig.bw = 206; //LORA_BW_206;
		break;

    case 1121:
    radioconfig.chip = 1121;
    radioconfig.freq = 433125000;
		radioconfig.bw = 250; //LORA_BW_250;
		if(tcxo) 
		{
			lr112x_tcxo = 1;
			lr112x_tcxo_voltage = 2; //1.8V
		}
		else lr112x_tcxo = 0;
		break;
		
    case 2021:
    radioconfig.chip = 2021;
    radioconfig.freq = 433125000;
		radioconfig.bw = 250; //LORA_BW_250;
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
    radioconfig.freq = 433125000;
		radioconfig.bw = 250; //LORA_BW_250;
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
	uint8_t retval = 0;
  currfreq = radioconfig.freq / 1000;
	prevfreq = radioconfig.freq / 1000;
	retval = radio_system_init();
	printf("retval:%d\r\n",retval);
	if(retval != RADIO_OK) return retval;
	retval = radio_set_rf_freq(radioconfig.freq);
	printf("retval:%d\r\n",retval);
	if(retval != RADIO_OK) return retval;
	retval = radio_set_power_dbm(radioconfig.txpower);
	printf("retval:%d\r\n",retval);
	if(retval != RADIO_OK) return retval;
	retval = radio_set_lora();
	printf("retval:%d\r\n",retval);
	if(retval != RADIO_OK) return retval;
	retval = radio_set_mod_params(radioconfig.bw,radioconfig.sf,radioconfig.cr,radioconfig.ldropt);
	printf("retval:%d\r\n",retval);
	if(retval != RADIO_OK) return retval;
	retval = radio_set_pkt_params(radioconfig.sync,radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	printf("retval:%d\r\n",retval);
	if(retval != RADIO_OK) return retval;
	retval = radio_specific_settings();
	printf("retval:%d\r\n",retval);
	if(retval != RADIO_OK) return retval;
	return radio_rx();
}

int8_t radio_system_init(void)
{
	prevopmode = RADIO_OPMODE_STBYRC;
	switch(radioconfig.chip)
	{
		case 1262:
		{
			sx126x_status_t err = sx126x_reset(NULL);
			if(err != SX126X_STATUS_OK) return err;
			err = sx126x_wakeup(NULL);
			if(err != SX126X_STATUS_OK) return err;
			delay_ms(10);
			err = sx126x_set_standby(NULL,SX126X_STANDBY_CFG_RC);
			if(err != SX126X_STATUS_OK) return err;
			err = sx126x_set_reg_mode(NULL,SX126X_REG_MODE_DCDC);
			if(err != SX126X_STATUS_OK) return err;
			//set TCXO here if needed
			if(sx126x_tcxo != 0)
			{
				err = sx126x_set_dio3_as_tcxo_ctrl(NULL,(sx126x_tcxo_ctrl_voltages_t)sx126x_tcxo_voltage,1000); //check
				if(err != SX126X_STATUS_OK) return err;
				err = sx126x_cal(NULL,SX126X_CAL_ALL);
				if(err != SX126X_STATUS_OK) return err;
			}
			return RADIO_OK;
		}
		case 1280:
		{
			sx128x_status_t err = sx128x_reset(NULL);
			printf("err:%d\r\n",err);
			if(err != SX128X_STATUS_OK) return err;
			//err = sx128x_wakeup(NULL);
			//printf("err:%d\r\n",err);
			//if(err != SX128X_STATUS_OK) return err;
			//SX128X_setopmode(SX128X_OPMODE_STBYRC);
			err = sx128x_set_reg_mode(NULL,SX128X_REG_MODE_DCDC);
			printf("err:%d\r\n",err);
			if(err != SX128X_STATUS_OK) return err;
			return RADIO_OK;
		}
		case 1121:
		{
			uint16_t errors;	
			lr11xx_system_version_t version;
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
			
			lr11xx_system_set_reg_mode(NULL,LR11XX_SYSTEM_REG_MODE_DCDC); // DC-DC
			lr11xx_system_enable_spi_crc(NULL,false);
			lr11xx_system_set_dio_as_rf_switch(NULL, &rfsw_cfg);
			
			lr11xx_system_clear_errors(NULL);
			if(lr112x_tcxo) lr11xx_system_set_tcxo_mode(NULL,LR11XX_SYSTEM_TCXO_CTRL_1_8V,320);
			delay_ms(10);
			
			lr11xx_system_cfg_lfclk(NULL, LR11XX_SYSTEM_LFCLK_XTAL, true);
			lr11xx_system_get_errors(NULL, &errors);	
			lr11xx_system_clear_errors(NULL);
			lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);
			//lr11xx_system_calibrate(NULL, 0x3f);
			lr11xx_system_set_standby(NULL,LR11XX_SYSTEM_STANDBY_CFG_XOSC);
			delay_ms(10);
			lr11xx_system_get_version(NULL, &version);

			return RADIO_OK;
		}
		case 2021:
		{
			lr20xx_system_version_t version;
			lr20xx_status_t err = lr20xx_system_reset(NULL);
			printf("err0:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;
			//delay_ms(500);
			// Workaround SIMO
			const uint32_t freq_val = 2.8e6 * 1.048576;
			err = lr20xx_regmem_write_regmem32( NULL, 0x80004c, &freq_val, 1 );
			printf("err1:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;
			// Configure the regulator
			err = lr20xx_system_set_reg_mode(NULL,LR20XX_SYSTEM_REG_MODE_DCDC); // DC-DC
			printf("err2:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;
			//TCXO
			if(lr202x_tcxo) 
			{
				err = lr20xx_system_set_tcxo_mode(NULL,LR20XX_SYSTEM_TCXO_CTRL_3_0V,64000); //check
				printf("err3:%d\r\n",err);
				if(err != LR20XX_STATUS_OK) return err;
			}
			err = lr20xx_system_cfg_lfclk(NULL, LR20XX_SYSTEM_LFCLK_RC);//32.768
			
			printf("err4:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;
			uint16_t errors;
			err = lr20xx_system_get_errors( NULL, &errors );
			printf("err5:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;
			if(errors != 0) 
			{
				err = lr20xx_system_clear_errors(NULL);
				printf("err6:%d\r\n",err);
				if(err != LR20XX_STATUS_OK) return err;
			}
			err = lr20xx_system_clear_irq_status(NULL, LR20XX_SYSTEM_IRQ_ALL_MASK);
			printf("err7:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;			
			//IRQ 
			err = lr20xx_system_set_dio_function( NULL, LR20XX_SYSTEM_DIO_9, LR20XX_SYSTEM_DIO_FUNC_IRQ, LR20XX_SYSTEM_DIO_DRIVE_PULL_UP );
			printf("err8:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;
			err = lr20xx_system_set_dio_irq_cfg( NULL, LR20XX_SYSTEM_DIO_9, LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR);
			printf("err9:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;
			err = lr20xx_system_cfg_clk_output( NULL, LR20XX_SYSTEM_HF_CLK_SCALING_32_MHZ );
			printf("err10:%d\r\n",err);
			if(err != LR20XX_STATUS_OK) return err;
			//Calibration
			lr20xx_radio_common_front_end_calibration_value_t front_end_calibration_structures[3] = { 0 };
			printf("err11:%d\r\n",err);
			//LR20xx_bsp_get_front_end_calibration_cfg( NULL, front_end_calibration_structures );
			//err = lr20xx_radio_common_calibrate_front_end_helper( NULL, front_end_calibration_structures, 3 );
			//printf("err12:%d\r\n",err);
			//if(err != LR20XX_STATUS_OK) return err;
			err = lr20xx_system_get_version(NULL, &version);
			printf("err13:%d\r\n",err);			
			if(err != LR20XX_STATUS_OK) return err;
			return RADIO_OK;
		}
		case 3029:
		{
			if(PAN_Init(radioconfig.freq) != PAN_OK) return RADIO_INVALID_MODE; //in STB3 state
			return RADIO_OK;
		}
		default:
		return INVALID_CHIP;
	}
}



//Common functions
//set RF frequency
int8_t radio_set_freq(uint32_t khz)
{
	currfreq = khz;
	int8_t err = RADIO_OK;
	//prevopmode = opmode;
	//radio_setopmode(RADIO_OPMODE_STBYXOSC);
	err = radio_set_rf_freq(currfreq * 1000);
	//radio_setopmode(prevopmode);
	return err;
}

int8_t radio_set_power(int8_t dbm)
{
	int8_t err = RADIO_OK;
	//prevopmode = opmode;
	//radio_setopmode(RADIO_OPMODE_STBYXOSC);
	err = radio_set_power_dbm(dbm);
	//radio_setopmode(prevopmode);
	return err;
}

int8_t radio_set_lora(void)
{
	int8_t err;
	switch(radioconfig.chip)
	{
		case 1262: err = (int8_t)sx126x_set_pkt_type(NULL,SX126X_PKT_TYPE_LORA);break;
		case 1280: err = (int8_t)sx128x_set_pkt_type(NULL,SX128X_PKT_TYPE_LORA);break;
		case 1121: 
		{
			err = (int8_t)lr11xx_radio_set_pkt_type(NULL,LR11XX_RADIO_PKT_TYPE_LORA);
			if(err != SX128X_STATUS_OK) return err;
			uint8_t fix[] = SX128X_REG_RSSI_SNR_BUGFIX_BLOB;
			err = (int8_t)sx128x_write_register(NULL, SX128X_REG_RSSI_SNR_BUGFIX_ADDRESS, fix, sizeof( fix ) );
			break;
		}
		case 2021: err = (int8_t)lr20xx_radio_common_set_pkt_type(NULL, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);break;
		case 3029: err = RADIO_OK;break;
		default: err = INVALID_CHIP;break;
//		
//		case 1262: err = (int8_t)sx126x_set_pkt_type(NULL,SX126X_PKT_TYPE_LORA);break;
//		case 1280: err = (int8_t)sx128x_set_pkt_type(NULL,SX128X_PKT_TYPE_LORA);break;
//		case 1121: err = (int8_t)lr11xx_radio_set_pkt_type(NULL,LR11XX_RADIO_PKT_TYPE_LORA);break;
//		case 2021: err = (int8_t)lr20xx_radio_common_set_pkt_type(NULL, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);break;
//		case 3029: err = RADIO_OK;
//		default: err = INVALID_CHIP;
	}
	//return err;
	return RADIO_OK;
}

int8_t radio_specific_settings(void)
{
	int8_t err;
	switch(radioconfig.chip)
	{
		case 1262:
		{
			SX126X_CalibrateIR();
			if(sx126x_tcxo == 0) //TCXO off
			{
				err = (int8_t)sx126x_set_trimming_capacitor_values(NULL,sx126x_xtatrim,sx126x_xtbtrim);
				if(err != RADIO_OK) return err;
			}
			err = (int8_t)sx126x_cfg_rx_boosted(NULL,true);
			if(err != RADIO_OK) return err;
			err = (int8_t)sx126x_set_dio2_as_rf_sw_ctrl(NULL,true);
			if(err != RADIO_OK) return err;
			return (int8_t)sx126x_set_dio_irq_params(NULL,SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR,SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE,SX126X_IRQ_NONE,SX126X_IRQ_NONE);
		}
		case 1280:
		{
			err = (int8_t)sx128x_set_buffer_base_address(NULL,0,0);	
			if(err != RADIO_OK) return err;
			err = (int8_t)sx128x_set_lna_settings(NULL,SX128X_LNA_HIGH_SENSITIVITY_MODE); //SX128X_LNA_LOW_POWER_MODE
			if(err != RADIO_OK) return err;
			return (int8_t)sx128x_set_dio_irq_params(NULL,SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE | SX128X_IRQ_CRC_ERROR,SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE,SX128X_IRQ_NONE,SX128X_IRQ_NONE);
		}
		case 1121:
		{
			err = (int8_t)lr11xx_radio_set_rx_tx_fallback_mode(NULL,LR11XX_RADIO_FALLBACK_STDBY_XOSC);
			if(err != RADIO_OK) return err;
			err = lr11xx_radio_cfg_rx_boosted(NULL, 0x00);// enable_boost_mode
			if(err != RADIO_OK) return err;
			err = lr11xx_system_calibrate_image(NULL,radioconfig.freq / 4000000, radioconfig.freq / 4000000 + 2); //must be rewritted
			if(err != RADIO_OK) return err;
			//calibrate RSSI
			LR112X_RssiCal(radioconfig.freq);
			return (int8_t)lr11xx_system_set_dio_irq_params(NULL,LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR,LR11XX_SYSTEM_IRQ_NONE);
		}
		case 2021:
		return (int8_t)lr20xx_radio_common_set_rx_tx_fallback_mode(NULL, LR20XX_RADIO_FALLBACK_STDBY_XOSC);	//???
		//DIO,IRQ
			
		case 3029:
		PAN_SetFreq(radioconfig.freq);          /* Set the frequency */
		PAN_SetRegulatorMode(USE_LDO);         /* Set the chip to LDO power mode */
		//PAN_SetChipMode(CHIPMODE_MODE0);
		//PAN_SetChipMode(CHIPMODE_MODE1);       /* Set the chip mode to MODE1 */
		//enable interrupts
		PAN_SetPageRegBits(0,0x58,PAN_IRQ_TX_DONE | PAN_IRQ_RX_DONE | PAN_IRQ_CRC_ERR);
		//PAN_SetPageRegBits(0,0x58,0xff);
		return RADIO_OK;
		
		default:
		return INVALID_CHIP;
	}
}

int8_t radio_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
{
	int8_t err = RADIO_OK;
//	if((bw_khz < 7) || (bw_khz > 1000)) return RADIO_INVALID_PARAMETER;
//	if((sf < 5) || (sf > 12)) return RADIO_INVALID_PARAMETER;
//	if(cr > 4) return RADIO_INVALID_PARAMETER;
//	if(ldropt > 1) return RADIO_INVALID_PARAMETER;
//	radioconfig.bw = bw_khz;
//	radioconfig.sf = sf;
//	radioconfig.cr = cr;
//	radioconfig.ldropt = ldropt;
	//prevopmode = opmode;
	//radio_setopmode(RADIO_OPMODE_STBYXOSC);
	err = radio_setmodparams(bw_khz,sf,cr,ldropt);
	//radio_setopmode(prevopmode);
	return err;
}

int8_t radio_set_pkt_params(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
	int8_t err = RADIO_OK;
//	if(header > 1) return RADIO_INVALID_PARAMETER;
//	if(crc > 1) return RADIO_INVALID_PARAMETER;
//	if(invertiq > 1) return RADIO_INVALID_PARAMETER;
//	radioconfig.sync = sync;
//	radioconfig.prelen = prelen;
//	radioconfig.paylen = paylen;
//	radioconfig.header = header;
//	radioconfig.crc = crc;
//	radioconfig.invertiq = invertiq;
	//prevopmode = opmode;
	//radio_setopmode(RADIO_OPMODE_STBYXOSC);
	err = radio_setpktparams(sync,prelen,paylen,header,crc,invertiq);
	//radio_setopmode(prevopmode);
	return err;
}

//Chip specific functions
int8_t radio_set_rf_freq(uint32_t Hz)
{
	switch(radioconfig.chip)
	{
		case 1262:
		{
			sx126x_status_t err = sx126x_set_rf_freq(NULL,Hz);
			if(err != SX126X_STATUS_OK) return err;
			return RADIO_OK;
		}
		case 1280:
		{
			sx128x_status_t err = sx128x_set_rf_freq(NULL,Hz);
			if(err != SX128X_STATUS_OK) return err;
			return RADIO_OK;
		}
		case 1121:
		{
			lr11xx_status_t err = lr11xx_radio_set_rf_freq(NULL, Hz);
			if(err != LR11XX_STATUS_OK) return err;
			if (Hz < 1900000000) 
			{
				err = lr11xx_radio_set_pa_cfg(NULL, &pa_config_subGHz);
				if(err != LR11XX_STATUS_OK) return err;
			}
			else
			{
				err = lr11xx_radio_set_pa_cfg(NULL, &pa_config_HF);
				if(err != LR11XX_STATUS_OK) return err;
			}
			return RADIO_OK;
		}
		case 2021:
		{
			lr20xx_status_t err = lr20xx_radio_common_set_rf_freq(NULL, Hz);
			if(err != LR20XX_STATUS_OK) return err;
			if (Hz < 1900000000) 
			{ // SubGHz
				err = lr20xx_radio_common_set_pa_cfg(NULL, &pa_config_lf);
				if(err != LR20XX_STATUS_OK) return err;
				lr20xx_radio_common_set_rx_path( NULL,LR20XX_RADIO_COMMON_RX_PATH_LF,LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
				if(err != LR20XX_STATUS_OK) return err;
			}
			else 
			{
				err = lr20xx_radio_common_set_pa_cfg(NULL, &pa_config_hf);
				if(err != LR20XX_STATUS_OK) return err;
				err = lr20xx_radio_common_set_rx_path( NULL,LR20XX_RADIO_COMMON_RX_PATH_HF,LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
				if(err != LR20XX_STATUS_OK) return err;
			}
			return RADIO_OK;
		}
		case 3029:
		{
			if(PAN_SetFreq(Hz) == PAN_FAIL) return RADIO_INVALID_PARAMETER; 
			return RADIO_OK;
		}
		default:
		return INVALID_CHIP;	
	}
}
//set tx power
int8_t radio_set_power_dbm(int8_t dbm)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_status_t err = sx126x_set_tx_params(NULL,dbm,SX126X_RAMP_10_US);
			if(err != SX126X_STATUS_OK) return err;
			return RADIO_OK;
		}
		case 1280:
		{
			sx128x_status_t err = sx128x_set_tx_params(NULL,dbm,SX128X_RAMP_02_US);
			if(err != SX128X_STATUS_OK) return err;
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
		{
			lr20xx_status_t err = lr20xx_radio_common_set_tx_params(NULL,radioconfig.txpower*2, LR20XX_RADIO_COMMON_RAMP_32_US);
			if(err != LR20XX_STATUS_OK) return err;
			return RADIO_OK;
		}
		case 3029:
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
			uint8_t bw_value;
			sx126x_mod_params_lora_t modparams;
			if(bw_khz <= 8) bw_value = SX126X_LORA_BW_007;
			else if(bw_khz <= 11) bw_value = SX126X_LORA_BW_010;
			else if(bw_khz <= 16) bw_value = SX126X_LORA_BW_015;
			else if(bw_khz <= 21) bw_value = SX126X_LORA_BW_020;
			else if(bw_khz <= 32) bw_value = SX126X_LORA_BW_031;
			else if(bw_khz <= 42) bw_value = SX126X_LORA_BW_041;
			else if(bw_khz <= 63) bw_value = SX126X_LORA_BW_062;
			else if(bw_khz <= 125) bw_value = SX126X_LORA_BW_125;
			else if(bw_khz <= 250) bw_value = SX126X_LORA_BW_250;
			else bw_value = SX126X_LORA_BW_500;
			modparams.bw = (sx126x_lora_bw_t)bw_value;
			modparams.cr = (sx126x_lora_cr_t)cr;
			modparams.sf = (sx126x_lora_sf_t)sf;
			modparams.ldro = ldropt;
			return(int8_t)sx126x_set_lora_mod_params(NULL,&modparams);
		}
    case 1280:
		{
			uint8_t bw_value;
			sx128x_mod_params_lora_t modparams;
			if(bw_khz <= 204) bw_value = SX128X_LORA_RANGING_BW_200;
			else if(bw_khz <= 407) bw_value = SX128X_LORA_RANGING_BW_400;
			else if(bw_khz <= 813) bw_value = SX128X_LORA_RANGING_BW_800;
			else bw_value = SX128X_LORA_RANGING_BW_1600;
			modparams.bw = (sx128x_lora_bw_t)bw_value;
			modparams.sf = (sx128x_lora_sf_t)(sf << 4);
			modparams.cr = (sx128x_lora_cr_t)cr;
			return (int8_t)sx128x_set_lora_mod_params(NULL,&modparams);
		}
    case 1121:
		{
			uint8_t bw_value;
			lr11xx_radio_mod_params_lora_t modparams;
			if(bw_khz <= 11) bw_value = LR11XX_RADIO_LORA_BW_10;
			else if(bw_khz <= 16) bw_value = LR11XX_RADIO_LORA_BW_15;
			else if(bw_khz <= 21) bw_value = LR11XX_RADIO_LORA_BW_20;
			else if(bw_khz <= 32) bw_value = LR11XX_RADIO_LORA_BW_31;
			else if(bw_khz <= 42) bw_value = LR11XX_RADIO_LORA_BW_41;
			else if(bw_khz <= 63) bw_value = LR11XX_RADIO_LORA_BW_62;
			else if(bw_khz <= 125) bw_value = LR11XX_RADIO_LORA_BW_125;
			else if(bw_khz <= 204) bw_value = LR11XX_RADIO_LORA_BW_200;
			else if(bw_khz <= 251) bw_value = LR11XX_RADIO_LORA_BW_250;
			else if(bw_khz <= 407) bw_value = LR11XX_RADIO_LORA_BW_400;
			else if(bw_khz <= 501) bw_value = LR11XX_RADIO_LORA_BW_500;
			else bw_value = LR11XX_RADIO_LORA_BW_800;
			modparams.bw = (lr11xx_radio_lora_bw_t)bw_value;
			modparams.sf = (lr11xx_radio_lora_sf_t)radioconfig.sf;
			modparams.cr = (lr11xx_radio_lora_cr_t)radioconfig.cr;
			modparams.ldro = radioconfig.ldropt;
			return (int8_t)lr11xx_radio_set_lora_mod_params(NULL,&modparams);
		}
		case 2021:
		{
			uint8_t bw_value;
			lr20xx_radio_lora_mod_params_t modparams;
			if(bw_khz <= 8) bw_value = LR20XX_RADIO_LORA_BW_7;
			else if(bw_khz <= 11) bw_value = LR20XX_RADIO_LORA_BW_10;
			else if(bw_khz <= 16) bw_value = LR20XX_RADIO_LORA_BW_15;
			else if(bw_khz <= 21) bw_value = LR20XX_RADIO_LORA_BW_20;
			else if(bw_khz <= 32) bw_value = LR20XX_RADIO_LORA_BW_31;
			else if(bw_khz <= 42) bw_value = LR20XX_RADIO_LORA_BW_41;
			else if(bw_khz <= 63) bw_value = LR20XX_RADIO_LORA_BW_62;
			else if(bw_khz <= 84) bw_value = LR20XX_RADIO_LORA_BW_83;
			else if(bw_khz <= 102) bw_value = LR20XX_RADIO_LORA_BW_101;
			else if(bw_khz <= 125) bw_value = LR11XX_RADIO_LORA_BW_125;
			else if(bw_khz <= 204) bw_value = LR20XX_RADIO_LORA_BW_203;
			else if(bw_khz <= 251) bw_value = LR11XX_RADIO_LORA_BW_250;
			else if(bw_khz <= 407) bw_value = LR20XX_RADIO_LORA_BW_406;
			else if(bw_khz <= 501) bw_value = LR11XX_RADIO_LORA_BW_500;
			else if(bw_khz <= 813) bw_value = LR20XX_RADIO_LORA_BW_812;
			else bw_value = LR20XX_RADIO_LORA_BW_1000;
			modparams.bw = (lr20xx_radio_lora_bw_t)bw_value;
			modparams.sf = (lr20xx_radio_lora_sf_t)sf;
			modparams.cr = (lr20xx_radio_lora_cr_t)cr;
			modparams.ppm = (lr20xx_radio_lora_ppm_t)ldropt; //??? LR20XX_RADIO_LORA_NO_PPM  = 0x00,  //!< No PPM offset: use full range of modulation LR20XX_RADIO_LORA_PPM_1_4 = 0x01
			return (int8_t)lr20xx_radio_lora_set_modulation_params(NULL,&modparams);
		}
		case 3029:
		{
			uint8_t bw_value;
			if(bw_khz <= 63) bw_value = PAN_BW_062K;
			else if(bw_khz <= 125) bw_value = PAN_BW_125K;
			else if(bw_khz <= 250) bw_value = PAN_BW_250K;
			else bw_value = PAN_BW_500K;
			PAN_SetBW(bw_value);              /* Set the bandwidth */
			PAN_SetSF(radioconfig.sf);              /* Set the spreading factor */
			PAN_SetCR(radioconfig.cr);              /* Set the channel coding rate */
			PAN_SetLDR(radioconfig.ldropt);            /* Set the low-rate mode */
			return RADIO_OK;
		}
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_getmodparams(uint16_t *bw_khz,uint8_t *sf,uint8_t *cr,uint8_t *ldropt)
{
	*sf = radioconfig.sf;
	*bw_khz = radioconfig.bw;
	*cr = radioconfig.cr;
	*ldropt = radioconfig.ldropt;
	return RADIO_OK;
}

//set packet parameters
int8_t radio_setpktparams(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
  switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_pkt_params_lora_t pktparams;
			pktparams.header_type = (sx126x_lora_pkt_len_modes_t)radioconfig.header;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.crc_is_on = radioconfig.crc;
			pktparams.invert_iq_is_on = radioconfig.invertiq;
			sx126x_status_t err = sx126x_set_lora_pkt_params(NULL,&pktparams);
			if(err != SX126X_STATUS_OK) return (int8_t)err;
			return (int8_t)sx126x_set_lora_sync_word(NULL,radioconfig.sync);
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
			sx128x_status_t err = sx128x_set_lora_pkt_params(NULL,&pktparams);
			if(err != SX128X_STATUS_OK) return (int8_t)err;
			//set sync
			return (int8_t)sx128x_set_lora_sync_word(NULL,radioconfig.sync);
		}
    case 1121:
		{
			lr11xx_radio_pkt_params_lora_t pktparams;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.header_type = (lr11xx_radio_lora_pkt_len_modes_t)radioconfig.header;
			pktparams.crc = (lr11xx_radio_lora_crc_t)radioconfig.crc;
			pktparams.iq = (lr11xx_radio_lora_iq_t)radioconfig.invertiq;
			lr11xx_status_t err = lr11xx_radio_set_lora_pkt_params(NULL,&pktparams);
			if(err != LR11XX_STATUS_OK) return (int8_t)err;
			return (int8_t)lr11xx_radio_set_lora_sync_word(NULL,radioconfig.sync & 0xff);
		}
		case 2021:
		{
			lr20xx_radio_lora_pkt_params_t pktparams;
			pktparams.preamble_len_in_symb = radioconfig.prelen;
			pktparams.pld_len_in_bytes = radioconfig.paylen;
			pktparams.pkt_mode = (lr20xx_radio_lora_pkt_mode_t)radioconfig.header;
			pktparams.crc = (lr20xx_radio_lora_crc_t)radioconfig.crc;
			pktparams.iq = (lr20xx_radio_lora_iq_t)radioconfig.invertiq;
			lr20xx_status_t err = lr20xx_radio_lora_set_packet_params(NULL,&pktparams);
			if(err != LR20XX_STATUS_OK) return (int8_t)err;
			return (int8_t)lr20xx_radio_lora_set_syncword(NULL,radioconfig.sync);
		}
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
  int8_t err = radio_setpktparams(radioconfig.sync,radioconfig.prelen,txlen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	if(err != RADIO_OK) return err;
	switch(radioconfig.chip)
  {
    case 1262:
		err = (int8_t)sx126x_write_buffer(NULL,0,buf,txlen);
		if(err != RADIO_OK) return err;
		break;
		
    case 1280:
		err = (int8_t)sx128x_write_buffer(NULL,0,buf,txlen);
		if(err != RADIO_OK) return err;
		break;
		
    case 1121:
		err = (int8_t)lr11xx_regmem_write_buffer8(NULL,buf,txlen);
		if(err != RADIO_OK) return err;
		break;
		
		case 2021:
		//err = (int8_t)lr20xx_radio_fifo_clear_tx(NULL);
		//if(err != RADIO_OK) return err;
		err = (int8_t)lr20xx_radio_fifo_write_tx(NULL,buf,txlen);
		if(err != RADIO_OK) return err;
		break;
		
		case 3029:
		PAN_SetTx(buf, txlen);
		return RADIO_OK;
    
    default:
    return INVALID_CHIP;
  }
	return radio_setopmode(RADIO_OPMODE_TX);
}
//retrieve packet info
int8_t radio_getpktstatus(rxpacketstatus_t *status)
{
  int8_t err;
	switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_pkt_status_lora_t pktstatus;
			err = (int8_t)sx126x_get_lora_pkt_status(NULL,&pktstatus);
			if(err != RADIO_OK) return err;
			status->rssi_pkt = pktstatus.rssi_pkt_in_dbm;
			status->snr_pkt = pktstatus.snr_pkt_in_db; 
			status->signal_rssi_pkt = pktstatus.signal_rssi_pkt_in_dbm;
			return RADIO_OK;
		}
    case 1280:
		{
			sx128x_pkt_status_lora_t pktstatus;
			err = (int8_t)sx128x_get_lora_pkt_status(NULL,&pktstatus);
			if(err != RADIO_OK) return err;
			status->rssi_pkt = pktstatus.rssi; 
			status->snr_pkt = pktstatus.snr;
			status->signal_rssi_pkt = status->rssi_pkt + status->snr_pkt; //???
			return RADIO_OK;
		}
    case 1121:
		{
			lr11xx_radio_pkt_status_lora_t pktstatus;
			err = lr11xx_radio_get_lora_pkt_status(NULL,&pktstatus);
			if(err != RADIO_OK) return err;
			status->rssi_pkt = pktstatus.rssi_pkt_in_dbm;
			status->snr_pkt = pktstatus.snr_pkt_in_db;
			status->signal_rssi_pkt = pktstatus.signal_rssi_pkt_in_dbm;
			return RADIO_OK;
		}
		case 2021:
		{
			lr20xx_radio_lora_packet_status_t pktstatus;
			err = (int8_t)lr20xx_radio_lora_get_packet_status(NULL,&pktstatus);
			if(err != RADIO_OK) return err;
			status->rssi_pkt = pktstatus.rssi_pkt_in_dbm;
			status->snr_pkt = pktstatus.snr_pkt_raw;
			status->signal_rssi_pkt = pktstatus.rssi_signal_pkt_in_dbm;
			return RADIO_OK;
		}
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
  int8_t err;
	switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_rx_buffer_status_t status;
			err = (int8_t)sx126x_get_rx_buffer_status(NULL,&status);
			if(err != RADIO_OK) return err;
			rxlen = status.pld_len_in_bytes;
			return(int8_t)sx126x_read_buffer(NULL,status.buffer_start_pointer,buf,rxlen);
		}
    case 1280:
		{
			sx128x_rx_buffer_status_t status;
			err = (int8_t)sx128x_get_rx_buffer_status(NULL,&status);
			if(err != RADIO_OK) return err;
			rxlen = status.pld_len_in_bytes;
			return(int8_t)sx128x_read_buffer(NULL,status.buffer_start_pointer,buf,rxlen);
		}
    case 1121:
		{
			lr11xx_radio_rx_buffer_status_t status;
			err = (int8_t)lr11xx_radio_get_rx_buffer_status(NULL,&status);
			if(err != RADIO_OK) return err;
			rxlen = status.pld_len_in_bytes;
			return(int8_t)lr11xx_regmem_read_buffer8(NULL,buf,status.buffer_start_pointer,rxlen);
			//lr11xx_regmem_clear_rxbuffer(NULL); //???
		}
		case 2021:
		{
			lr20xx_radio_lora_packet_status_t pktstatus;
			err = (int8_t)lr20xx_radio_lora_get_packet_status(NULL,&pktstatus);
			if(err != RADIO_OK) return err;
			rxlen = pktstatus.packet_length_bytes;
			return(int8_t)lr20xx_radio_fifo_read_rx(NULL,buf,rxlen);
			//lr20xx_radio_fifo_clear_rx(NULL); //???
		}
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
  int8_t err;
	switch(radioconfig.chip)
  {
    case 1262:
		{
			sx126x_stats_lora_t lora_stats;
			err = (int8_t)sx126x_get_lora_stats(NULL,&lora_stats);
			if(err != RADIO_OK) return err;
			stats->pkt_received = lora_stats.nb_pkt_received;
			stats->crc_error = lora_stats.nb_pkt_crc_error;
			stats->header_error = lora_stats.nb_pkt_header_error;
			stats->false_sync = 0;
			return RADIO_OK;
		}
    case 1280:
    return FEATURE_NOT_SUPPORTED;
		
    case 1121:
		{
			lr11xx_radio_stats_lora_t lora_stats;
			err = (int8_t)lr11xx_radio_get_lora_stats(NULL,&lora_stats);
			if(err != RADIO_OK) return err;
			stats->pkt_received = lora_stats.nb_pkt_received;
			stats->crc_error = lora_stats.nb_pkt_crc_error;
			stats->header_error = lora_stats.nb_pkt_falsesync;
			return RADIO_OK;
		}
		
		case 2021:
		{
			lr20xx_radio_lora_rx_statistics_t lora_stats;
			err = (int8_t)lr20xx_radio_lora_get_rx_statistics(NULL,&lora_stats);
			if(err != RADIO_OK) return err;
			stats->pkt_received = lora_stats.n_received_packets;
			stats->crc_error = lora_stats.n_crc_errors;
			stats->header_error = lora_stats.n_false_synchronisation;
			stats->header_error = lora_stats.n_header_errors;
			return RADIO_OK;
		}

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
		return(int8_t)sx126x_reset_stats(NULL);
		
    case 1280:
    return FEATURE_NOT_SUPPORTED;
		
    case 1121:
    return(int8_t)lr11xx_radio_reset_stats(NULL);
		
		case 2021:
		return(int8_t)lr20xx_radio_common_reset_rx_stats(NULL);
		
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
		{
			//read status
			uint32_t irqstatus;
			lr20xx_system_get_and_clear_irq_status(NULL,&irqstatus);
			//lr20xx_system_clear_irq_status(NULL,LR20XX_SYSTEM_IRQ_ALL_MASK);
			//printf("IRQ:0x%08X\r\n",irqstatus);
			if(irqstatus & LR20XX_SYSTEM_IRQ_TX_DONE) packet_sent = true;
			if(irqstatus & LR20XX_SYSTEM_IRQ_RX_DONE) packet_received = true;
			if(irqstatus & LR20XX_SYSTEM_IRQ_CRC_ERROR) crc_error = true;
//			if(irqstatus & LR20XX_SYSTEM_IRQ_FIFO_RX) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_FIFO_TX) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_TX_TIMESTAMP) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_RX_TIMESTAMP) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_CAD_DETECTED) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_LORA_RX_HEADER_TIMESTAMP) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_LOW_BATTERY) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_PA_OVP_OCP) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_ERROR) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_CMD_ERROR) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_CAD_DONE) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_TIMEOUT) {};		
//			if(irqstatus & LR20XX_SYSTEM_IRQ_LEN_ERROR) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_ADDR_ERROR) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP) {};	
//			if(irqstatus & LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_RESPONSE_DONE) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_DISCARDED) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_EXCHANGE_VALID) {};
//			if(irqstatus & LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_TIMEOUT) {};
		}
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
  int8_t err = radio_setpktparams(radioconfig.sync,radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	if(err != RADIO_OK) return err;
	return radio_setopmode(RADIO_OPMODE_RX);
}

int8_t radio_getrssi(float *dbm)
{
  int8_t err;
	switch(radioconfig.chip)
  {
    case 1262:
		{
			int16_t rssi;
			err = (int8_t)sx126x_get_rssi_inst(NULL,&rssi);
			if(err != RADIO_OK) return err;
			*dbm = rssi;
			return RADIO_OK;
		}
    case 1280:
		{
			int16_t rssi;
			err = (int8_t)sx128x_get_rssi_inst(NULL,&rssi);
			if(err != RADIO_OK) return err;
			*dbm = rssi;
			return RADIO_OK;
		}
    case 1121:
		{
			int8_t rssi;
			err = (int8_t)lr11xx_radio_get_rssi_inst(NULL,&rssi);
			if(err != RADIO_OK) return err;
			*dbm = rssi;
			return RADIO_OK;
		}
		case 2021:
		{
			int16_t rssi;
			uint8_t half_dbm_cnt;
			err = (int8_t)lr20xx_radio_common_get_rssi_inst(NULL,&rssi,&half_dbm_cnt);
			if(err != RADIO_OK) return err;
			*dbm = (rssi + half_dbm_cnt) / 2.0f; //to be checked
			return RADIO_OK;
		}
		case 3029:
		*dbm = (float)PAN_GetRealTimeRssi();
		return RADIO_OK;
   
    default:
    return INVALID_CHIP;
  }
}

int8_t radio_stream(uint8_t stream)
{
  int8_t err;
	if(stream == 0) 
	{
		txmode = 0;
		txled_off();
		if(radioconfig.chip == 3029) PAN_StopTxContinuousWave();
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
			err = (int8_t)sx126x_set_tx_cw(NULL);
			txmode = 1;
		}
    else 
		{
			err = (int8_t)sx126x_set_tx_infinite_preamble(NULL);
			txmode = 2;
		}
		txled_on();
    return err;
		
    case 1280:
    if(stream > 2) return RADIO_INVALID_PARAMETER;
    if(stream == 1) 
		{
			err = (int8_t)sx128x_set_tx_cw(NULL);
			txmode = 1;
		}
    else 
		{
			err = (int8_t)sx128x_set_tx_infinite_preamble(NULL);
			txmode = 2;
		}
		txled_on();
    return err;
		
    case 1121:
    if(stream > 2) return RADIO_INVALID_PARAMETER;
    if(stream == 1) 
		{
			err = (int8_t)lr11xx_radio_set_tx_cw(NULL);
			txmode = 1;
		}
    else 
		{
			err = (int8_t)lr11xx_radio_set_tx_infinite_preamble(NULL);
			txmode = 2;
		}
		txled_on();
    return err;
		
		case 2021:
    if(stream > 3) return RADIO_INVALID_PARAMETER;
    if(stream == 1) 
		{
			err = (int8_t)lr20xx_radio_common_set_tx_test_mode(NULL,LR20XX_RADIO_COMMON_TX_TEST_MODE_CONTINUOUS_WAVE);
			txmode = 1;
		}
    else if(stream == 2)
		{
			err = (int8_t)lr20xx_radio_common_set_tx_test_mode(NULL,LR20XX_RADIO_COMMON_TX_TEST_MODE_INFINITE_PREAMBLE);
			txmode = 2;
		}
		else
		{
			err = (int8_t)lr20xx_radio_common_set_tx_test_mode(NULL,LR20XX_RADIO_COMMON_TX_TEST_MODE_PRBS9);
			txmode = 3;
		}
		txled_on();
    return err;
		
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
		
		case 2021:
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
		
		case 2021:
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
		
		case 2021:
		return FEATURE_NOT_SUPPORTED;	
		
		case 3029:
		return FEATURE_NOT_SUPPORTED;

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
    case 1262: //0x035C, 0x0320
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

int8_t radio_setopmode(uint8_t mode)
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
		
		case 2021:
		LR202x_setopmode(mode);
		opmode = mode;
		return RADIO_OK;	
		
		case 3029:
		PAN_setopmode(mode);
		return RADIO_OK;
		
    default:
    return INVALID_CHIP;
  }
}
