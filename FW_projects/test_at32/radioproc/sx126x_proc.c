#include "sx126x_proc.h"

sx126x_status_t SX126x_init(void)
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
		err = sx126x_clear_device_errors(NULL);
		if(err != SX126X_STATUS_OK) return err;
	}
	err = (int8_t)sx126x_set_pkt_type(NULL,SX126X_PKT_TYPE_LORA);
	if(err != SX126X_STATUS_OK) return err;
	err = sx126x_set_rf_freq(NULL,radioconfig.freq);
	if(err != SX126X_STATUS_OK) return err;
	err = sx126x_set_buffer_base_address(NULL,0,0);
	if(err != SX126X_STATUS_OK) return err;
	err = SX126x_set_mod_params(radioconfig.bw,radioconfig.sf,radioconfig.cr,radioconfig.ldropt);
	if(err != SX126X_STATUS_OK) return err;
	err = SX126x_set_packet_params(radioconfig.sync,radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	if(err != SX126X_STATUS_OK) return err;
	err = sx126x_set_rx_with_timeout_in_rtc_step(NULL,0xffffff);
	if(err != SX126X_STATUS_OK) return err;
	SX126X_CalibrateIR();
	//err = sx126x_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
	//if(err != SX126X_STATUS_OK) return err;
	if(sx126x_tcxo == 0) //TCXO off
	{
		err = sx126x_set_trimming_capacitor_values(NULL,sx126x_xtatrim,sx126x_xtbtrim);
		if(err != RADIO_OK) return err;
	}
	sx126x_pa_cfg_params_t pa_cfg;
	pa_cfg.device_sel = 0;
	pa_cfg.hp_max = 7;
	pa_cfg.pa_duty_cycle = 4;
	pa_cfg.pa_lut = 1;
	err = sx126x_set_pa_cfg(NULL,&pa_cfg);
	if(err != SX126X_STATUS_OK) return err;
	err = sx126x_set_ocp_value(NULL,56); //140mA
	if(err != SX126X_STATUS_OK) return err;
	err = sx126x_set_tx_params(NULL,radioconfig.txpower,SX126X_RAMP_10_US);
	if(err != SX126X_STATUS_OK) return err;
	err = (int8_t)sx126x_cfg_rx_boosted(NULL,true);
	if(err != RADIO_OK) return err;
	err = (int8_t)sx126x_set_dio2_as_rf_sw_ctrl(NULL,true);
	if(err != RADIO_OK) return err;
	err = (int8_t)sx126x_set_dio_irq_params(NULL,SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR,SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE,SX126X_IRQ_NONE,SX126X_IRQ_NONE);
	if(err != RADIO_OK) return err;
	SX126X_setopmode(RADIO_OPMODE_RX);
	return SX126X_STATUS_OK;
}

sx126x_status_t SX126x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
{
	sx126x_status_t err;
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
	err = sx126x_set_lora_mod_params(NULL,&modparams);
	if(err != RADIO_OK) return err;
	return sx126x_tx_modulation_workaround(NULL,SX126X_PKT_TYPE_LORA,(sx126x_lora_bw_t)bw_value);
}

sx126x_status_t SX126x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
	sx126x_pkt_params_lora_t pktparams;
	pktparams.header_type = (sx126x_lora_pkt_len_modes_t)header;
	pktparams.preamble_len_in_symb = prelen;
	pktparams.pld_len_in_bytes = paylen;
	pktparams.crc_is_on = crc;
	pktparams.invert_iq_is_on = invertiq;
	sx126x_status_t err = sx126x_set_lora_pkt_params(NULL,&pktparams);
	if(err != SX126X_STATUS_OK) return err;
	return sx126x_set_lora_sync_word(NULL,radioconfig.sync);
}


void SX126X_setopmode(uint8_t mode)
{
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
    opmode = RADIO_OPMODE_SLEEP;
    sx126x_set_sleep(NULL,SX126X_SLEEP_CFG_COLD_START); //SX126X_SLEEP_CFG_WARM_START
    break;

    case RADIO_OPMODE_STBYRC:
    opmode = RADIO_OPMODE_STBYRC;
    sx126x_set_standby(NULL,SX126X_STANDBY_CFG_RC);
    break;

    case RADIO_OPMODE_STBYXOSC:
    opmode = RADIO_OPMODE_STBYXOSC;
    sx126x_set_standby(NULL,SX126X_STANDBY_CFG_XOSC);
    break;

    case RADIO_OPMODE_FS:
    opmode = RADIO_OPMODE_FS;
    sx126x_set_fs(NULL);
    break;

    case RADIO_OPMODE_TX:
    opmode = RADIO_OPMODE_TX;
    sx126x_set_tx_with_timeout_in_rtc_step(NULL,0xffffff); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
    opmode = RADIO_OPMODE_RX;
    sx126x_set_rx_with_timeout_in_rtc_step(NULL,0xffffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
    opmode = RADIO_OPMODE_TXSTREAMCW;
    sx126x_set_tx_cw(NULL);
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
    opmode = RADIO_OPMODE_TXSTREAMPRE;
    sx126x_set_tx_infinite_preamble(NULL);
    break;
  }
}

//boost LNA
void SX126X_LNAboost(bool boost)
{
	sx126x_cfg_rx_boosted(NULL,boost);
}

//calibrate IR according to RF ftequency
//The calibration frequencies are computed as follows:
//Calibration freq = CalFreq * 4 MHz where CalFreq1 < CalFreq2
//Example: 0x6B = 428 MHz.
//When CalFreq1 = CalFreq2, the image calibration is done at a single frequency.
//For frequencies between CalFreq1 and CalFreq2, the calibration coefficient is linearly
//interpolated from the values obtained during the image calibration at CalFreq1 and CalFreq2.
//For frequencies < CalFreq1, the coefficient obtained during the image calibration at CalFreq1 is used.
//For frequencies > CalFreq2, the coefficient obtained during the image calibration at CalFreq2 is used.

void SX126X_CalibrateIR(void)
{
  uint8_t f1,f2;
  uint32_t f;
  f = radioconfig.freq / 1000000UL;
  if(f > 1020) f = 1020;
  f1 = (uint8_t)(f / 4);
  f = (f * 3) / 2; //probably 1.5 times
  if(f > 1020) f = 1020;
  f2 = (uint8_t)(f / 4);
  sx126x_cal_img(NULL,f1,f2);
  //restore Ctune needed
}

void SX126X_irq_handler(void)
{
	//read SX126x status
	uint16_t irqstatus;
	sx126x_get_irq_status(NULL,&irqstatus);
	sx126x_clear_irq_status(NULL,SX126X_IRQ_ALL);
	//printf("Flags:0x%02X\r\n",irqstatus);
	if(irqstatus & SX126X_IRQ_TX_DONE) packet_sent = true;
	if(irqstatus & SX126X_IRQ_RX_DONE) packet_received = true;
	if(irqstatus & SX126X_IRQ_CRC_ERROR) crc_error = true;
	//if(irqstatus & SX126X_IRQ_PREAMBLE_DETECTED) {};
	//if(irqstatus & SX126X_IRQ_SYNC_WORD_VALID) {};
	//if(irqstatus & SX126X_IRQ_HEADER_VALID) {};
	//if(irqstatus & SX126X_IRQ_HEADER_ERROR) {};
	//if(irqstatus & SX126X_IRQ_CAD_DONE) {};
	//if(irqstatus & SX126X_IRQ_CAD_DETECTED) {};
	//if(irqstatus & SX126X_IRQ_TIMEOUT) {};
	//if(irqstatus & SX126X_IRQ_LR_FHSS_HOP) {};
}



