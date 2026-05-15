#include "lr112x_proc.h"
#include "lr11xx.h"
#include "radio_proc.h"


//#define K_FREQ	1.0000051f //-6.8 ppm
//#define K_FREQ	1.0f

void LR112X_printerrors(void);
void LR112X_printstatus(void);
void LR112X_configcommon(void);

const lr11xx_radio_rssi_calibration_table_t calib_0_600 =     {12,12,14, 0, 1, 3, 4, 4, 3, 6, 6, 6, 6, 6, 6, 6, 6, 0};
const lr11xx_radio_rssi_calibration_table_t calib_600_2000 =  { 2, 2, 2, 3, 3, 4, 5, 4, 4, 6, 5, 5, 6, 6, 6, 7, 6, 0};
const lr11xx_radio_rssi_calibration_table_t calib_2000_2700 = { 6, 7, 6, 4, 3, 4,14,12,14,12,12,12,12, 8, 8, 9, 9, 2030};

const lr11xx_radio_pa_cfg_t pa_config_HF = 
{  //1.9G/2.4G
	.pa_sel = LR11XX_RADIO_PA_SEL_HF, 
	.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG, 
	.pa_duty_cycle = 0x00, 
	.pa_hp_sel = 0x00 
};

const lr11xx_radio_pa_cfg_t pa_config_subGHz = 
{
	.pa_sel = LR11XX_RADIO_PA_SEL_HP,         				//!< Power Amplifier selection
	.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT, //!< Power Amplifier regulator supply source
	.pa_duty_cycle = 0x04,  													//!< Power Amplifier duty cycle (Default 0x04)
	.pa_hp_sel = 0x07      														//!< Number of slices for HPA (Default 0x07)
};

lr11xx_status_t LR112x_init(void)
{
	uint16_t errors;	
	lr11xx_system_version_t version;
	lr11xx_system_rfswitch_cfg_t rfsw_cfg;
	lr11xx_status_t err;
	
	rfsw_cfg.enable = LR112X_RFSW_ENABLE;
	rfsw_cfg.standby = LR112X_RFSW_STBY;
	rfsw_cfg.rx = LR112X_RFSW_RX;
	rfsw_cfg.tx = LR112X_RFSW_SUBG_TX;
	rfsw_cfg.tx_hp = LR112X_RFSW_SUBG_TX_HP;
	rfsw_cfg.tx_hf = LR112X_RFSW_HF_TX;
	rfsw_cfg.gnss = LR112X_RFSW_GNSS;
	rfsw_cfg.wifi = LR112X_RFSW_HF_RX; //check
	
	err = lr11xx_system_reset(NULL);
	if(err != LR11XX_STATUS_OK) return err;
	delay_ms(500);
	err = lr11xx_system_wakeup(NULL);
	if(err != LR11XX_STATUS_OK) return err;		
	err = lr11xx_system_set_reg_mode(NULL,LR11XX_SYSTEM_REG_MODE_DCDC); // DC-DC
	if(err != LR11XX_STATUS_OK) return err;	
	//err = lr11xx_system_enable_spi_crc(NULL,false);
	//if(err != LR11XX_STATUS_OK) return err;	
	err = lr11xx_system_set_dio_as_rf_switch(NULL, &rfsw_cfg);
	if(err != LR11XX_STATUS_OK) return err;			
	err = lr11xx_system_clear_errors(NULL);
	if(err != LR11XX_STATUS_OK) return err;	
	if(lr112x_tcxo) 
	{
		err = lr11xx_system_set_tcxo_mode(NULL,LR11XX_SYSTEM_TCXO_CTRL_1_8V,320);
		if(err != LR11XX_STATUS_OK) return err;	
	}
	delay_ms(10);
	err = lr11xx_system_cfg_lfclk(NULL, LR11XX_SYSTEM_LFCLK_RC, true); //LR11XX_SYSTEM_LFCLK_XTAL
	if(err != LR11XX_STATUS_OK) return err;	
	err = lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);
	if(err != LR11XX_STATUS_OK) return err;	
	err = lr11xx_system_set_standby(NULL,LR11XX_SYSTEM_STANDBY_CFG_XOSC);
	if(err != LR11XX_STATUS_OK) return err;	
	delay_ms(10);
	err = lr11xx_system_get_errors(NULL, &errors);
	if(err != LR11XX_STATUS_OK) return err;
	err = lr11xx_system_clear_errors(NULL);
	if(err != LR11XX_STATUS_OK) return err;	
	
	err = lr11xx_system_calibrate(NULL, 0x3f);
	if(err != LR11XX_STATUS_OK) return err;	
	
	err = lr11xx_system_clear_errors(NULL);
	if(err != LR11XX_STATUS_OK) return err;	
	err = lr11xx_system_clear_irq_status(NULL, LR11XX_SYSTEM_IRQ_ALL_MASK);
	if(err != LR11XX_STATUS_OK) return err;	
	
	err = lr11xx_system_get_version(NULL, &version);
	if(err != LR11XX_STATUS_OK) return err;	
	
	err = lr11xx_radio_set_pkt_type(NULL,LR11XX_RADIO_PKT_TYPE_LORA);
	if(err != LR11XX_STATUS_OK) return err;
	err = LR112x_set_mod_params(radioconfig.bw,radioconfig.sf,radioconfig.cr,radioconfig.ldropt);
	if(err != LR11XX_STATUS_OK) return err;
	err = LR112x_set_packet_params(radioconfig.sync,radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	if(err != LR11XX_STATUS_OK) return err;
	err = LR112X_set_freq(radioconfig.freq); //PA config here
	if(err != LR11XX_STATUS_OK) return err;
	err = lr11xx_radio_set_tx_params(NULL,radioconfig.txpower,LR11XX_RADIO_RAMP_16_US);
	if(err != LR11XX_STATUS_OK) return err;
	err = lr11xx_system_calibrate_image(NULL,radioconfig.freq / 4000000, radioconfig.freq / 4000000 + 2); //must be rewritted
	if(err != LR11XX_STATUS_OK) return err;
	//calibrate RSSI
	LR112X_RssiCal(radioconfig.freq);
	
	err = lr11xx_system_set_dio_irq_params(NULL,LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR,LR11XX_SYSTEM_IRQ_NONE);
	if(err != LR11XX_STATUS_OK) return err;
	err = lr11xx_radio_set_rx_tx_fallback_mode(NULL,LR11XX_RADIO_FALLBACK_STDBY_XOSC);
	if(err != LR11XX_STATUS_OK) return err;
	err = lr11xx_radio_cfg_rx_boosted(NULL, 0x00);// enable_boost_mode
	if(err != LR11XX_STATUS_OK) return err;
	LR112X_setopmode(RADIO_OPMODE_RX);
	return LR11XX_STATUS_OK;
}

lr11xx_status_t LR112X_set_freq(uint32_t Hz)
{
	lr11xx_status_t err = lr11xx_radio_set_rf_freq(NULL, Hz);
	if(err != LR11XX_STATUS_OK) return err;
	if (Hz < 1900000000) err = lr11xx_radio_set_pa_cfg(NULL, &pa_config_subGHz);
	else err = lr11xx_radio_set_pa_cfg(NULL, &pa_config_HF);
	if(err != LR11XX_STATUS_OK) return err;
	if(Hz < 600000000) err = lr11xx_radio_set_rssi_calibration(NULL,&calib_0_600);
	else if(Hz < 2000000000) err = lr11xx_radio_set_rssi_calibration(NULL,&calib_600_2000);
	else err = lr11xx_radio_set_rssi_calibration(NULL,&calib_2000_2700);
	return err;
}

lr11xx_status_t LR112x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
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
	return lr11xx_radio_set_lora_mod_params(NULL,&modparams);
}

lr11xx_status_t LR112x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
	lr11xx_radio_pkt_params_lora_t pktparams;
	pktparams.preamble_len_in_symb = radioconfig.prelen;
	pktparams.pld_len_in_bytes = radioconfig.paylen;
	pktparams.header_type = (lr11xx_radio_lora_pkt_len_modes_t)radioconfig.header;
	pktparams.crc = (lr11xx_radio_lora_crc_t)radioconfig.crc;
	pktparams.iq = (lr11xx_radio_lora_iq_t)radioconfig.invertiq;
	lr11xx_status_t err = lr11xx_radio_set_lora_pkt_params(NULL,&pktparams);
	if(err != LR11XX_STATUS_OK) return (int8_t)err;
	return lr11xx_radio_set_lora_sync_word(NULL,radioconfig.sync & 0xff);
}


void LR112X_setopmode(uint8_t mode)
{
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
		{
			opmode = RADIO_OPMODE_SLEEP;
			lr11xx_system_sleep_cfg_t sleepcfg;
			sleepcfg.is_rtc_timeout = false;
			sleepcfg.is_warm_start = false;
			lr11xx_system_set_sleep(NULL,sleepcfg,0);
			break;
		}

    case RADIO_OPMODE_STBYRC:
		opmode = RADIO_OPMODE_STBYRC;
    lr11xx_system_set_standby(NULL,LR11XX_SYSTEM_STANDBY_CFG_RC);
    break;

    case RADIO_OPMODE_STBYXOSC:
		opmode = RADIO_OPMODE_STBYXOSC;
    lr11xx_system_set_standby(NULL,LR11XX_SYSTEM_STANDBY_CFG_XOSC);
    break;

    case RADIO_OPMODE_FS:
		opmode = RADIO_OPMODE_FS;
    lr11xx_system_set_fs(NULL);
    break;

    case RADIO_OPMODE_TX:
		opmode = RADIO_OPMODE_TX;
    lr11xx_radio_set_tx(NULL,0); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
		opmode = RADIO_OPMODE_RX;
    lr11xx_radio_set_rx(NULL,0xffffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
		opmode = RADIO_OPMODE_TXSTREAMCW;
    lr11xx_radio_set_tx_cw(NULL);
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
		opmode = RADIO_OPMODE_TXSTREAMPRE;
    lr11xx_radio_set_tx_infinite_preamble(NULL);
    break;
  }
	//LR112X_printerrors(0);
}

void LR112X_RssiCal(uint32_t freq)
{
	if(freq <= 60000000) lr11xx_radio_set_rssi_calibration(NULL,&calib_0_600);
	else if((freq > 60000000) && (freq <= 200000000)) lr11xx_radio_set_rssi_calibration(NULL,&calib_600_2000);
	else lr11xx_radio_set_rssi_calibration(NULL,&calib_2000_2700);
}

void LR112X_printstatus(void)
{
	lr11xx_system_stat1_t stat1;
	lr11xx_system_stat2_t stat2;
	lr11xx_system_irq_mask_t irqstatus;
	lr11xx_system_get_status(NULL,&stat1,&stat2,&irqstatus);

	printf("Status=0x%02X,0x%02X\r\n",stat1.command_status,stat2.chip_mode); //mask reset source	`		
}

void LR112X_printerrors(void)
{
	
	uint16_t errors;
	lr11xx_system_get_errors(NULL,&errors);
	printf("Errors: 0x%04X\r\n",errors);
}

void LR112X_irq_handler(void)
{
	//read status
	uint32_t irqstatus;
	lr11xx_system_get_irq_status(NULL,&irqstatus);
	lr11xx_system_clear_irq_status(NULL,LR11XX_SYSTEM_IRQ_ALL_MASK);
	//printf("IRQ:0x%08X\r\n",irqstatus);
	if(irqstatus & LR11XX_SYSTEM_IRQ_TX_DONE) packet_sent = true;
	if(irqstatus & LR11XX_SYSTEM_IRQ_RX_DONE) packet_received = true;
	if(irqstatus & LR11XX_SYSTEM_IRQ_CRC_ERROR) crc_error = true;
	//if(irqstatus & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED) {};
	//if(irqstatus & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) {};
	//if(irqstatus & LR11XX_SYSTEM_IRQ_HEADER_ERROR) {};
	//if(irqstatus & LR11XX_SYSTEM_IRQ_CAD_DONE) {};
	//if(irqstatus & LR11XX_SYSTEM_IRQ_CAD_DETECTED) {};
	//if(irqstatus & LR11XX_SYSTEM_IRQ_TIMEOUT) {};
	//if(irqstatus & LR11XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP) {};
	//if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_REQ_VALID) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT) {};		
	//if(irqstatus & LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE) {};		
	//if(irqstatus & LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_EOL) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_CMD_ERROR) {};		
	//if(irqstatus & LR11XX_SYSTEM_IRQ_ERROR) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_FSK_ADDR_ERROR) {};	
	//if(irqstatus & LR11XX_SYSTEM_IRQ_LORA_RX_TIMESTAMP) {};	
}

//GainOffset: Global offset added to the Gain Tune values. The offset is a 12-bit signed value, where 1lsb = 0.5dB.
//The power seen by the LR1121 analog front-end is affected by external components such as the matching network, or RF switches. An incorrect RSSI results in a sensitivity degradation in (G)FSK mode and an incorrect gain selection in LoRa and GFSK mode. An incorrect gain can result in a missed detection (packet loss) or decreased resistance to interference.
//By default, the chip is calibrated for the 868-915MHz band on the LR1121 EVK.

//below 600MHz 0  12 12 14 0 1 3 4 4 3 6 6 6 6 6 6 6 6
//from 600MHz to 2GHz 0 2  2  2  3  3  4  5  4  4  6  5  5  6  6  6  7  6
//above 2GHz 2030 6 7 6 4 3 4 14 12 14 12 12 12 12 8 8 9 9

// uint8_t gain[] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 }; // NB: for the 2G4 path, the max gain is 16 - 17-20 can be ignored.
//    float power[] = { -35.0, -41.0, -45.0, -50.0, -53.5, -60.0, -65.0, -69.5, -75.0, -81.0, -82.5, -83.5, -84.0, -85.0, -86.0, -86.5, -87.0 };
//    /* Configure the chip at the system level */ 
//    lr11xx_system_reset( context );
//    lr11xx_system_set_reg_mode( context, reg_mode ); 
//    lr11xx_system_set_dio_as_rf_switch( context, rf_switch_cfg );
//    lr11xx_system_set_tcxo_mode( context, tune, timeout );  // Optional - only if there is a TCXO
//    lr11xx_system_clear_errors( context );
//    lr11xx_system_calibrate( context, 0x3F );  // 0x3F to enable all fields
//    /* Configure the chip at the modem level */ 
//    lr11xx_radio_set_pkt_type( context, LR11XX_RADIO_PKT_TYPE_GFSK ); 
//    lr11xx_radio_set_rf_freq( context, freq_in_hz );
//    lr11xx_system_calibrate_image_in_mhz( context, freq1_in_mhz, freq2_in_mhz ); 
//    lr11xx_radio_set_gfsk_mod_params( context, mod_params );  // Rx BW has to be set to
//LR11XX_RADIO_GFSK_BW_234300 - other modulation parameters can be anything 
//    lr11xx_radio_set_gfsk_pkt_params( context, pkt_params );  // Packet parameters can be
//anything
///* Configure the chip to be controlled manually */
//    lr11xx_regmem_write_regmem32_mask( context, 0x00F20214, 0x00080000, 0x00080000 );
//    lr11xx_regmem_write_regmem32_mask( context, 0x00F20230, 0x71110000, 0x71100000 );
//			
// lr11xx_radio_set_rssi_calibration( context, rssi_cal_table );  // All parrameters of rssi_cal_table set to 0
//    lr11xx_radio_set_rx_with_timeout_in_rtc_step( context, 0xFFFFFF );
//    for( int i = 0; i++; i < 17 )  // 17 is the number of elements in gain array 
//    {
//        const uint8_t gain_step = MIN( gain[i], 13 );
//        const uint8_t lna_boost = ( gain > 13 ) ? gain - 13 : 0;
//        lr11xx_regmem_write_regmem32_mask( context, 0x00F20214, 0x00F00000, gain_step << 20 ); 
//        lr11xx_regmem_write_regmem32_mask( context, 0x00F3008C, 0x00070000, lna_boost << 16 );

///* Wait for 1 ms */
///* Insert here a control for your test equipment to generate a tone at RF frequency set to freq_in_hz with an output power set to power[i] dBm */
//        lr11xx_radio_get_rssi_inst( context, rssi_in_dbm );
///* Add a way to log (gain[i], power[i], rssi_in_dbm) triplet to be able to compute offset and
//tunes for the RSSI calibration */
//    }		
