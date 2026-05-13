#include "lr202x_proc.h"
#include "lr20xx.h"
#include "radio_proc.h"

void LR202x_printerrors(void);
void LR202x_printstatus(void);

lr20xx_status_t LR202x_init(void)
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
	err = lr20xx_system_cfg_lfclk(NULL, LR20XX_SYSTEM_LFCLK_RC);//32.768
	printf("err3:%d\r\n",err);
	if(err != LR20XX_STATUS_OK) return err;
	uint16_t errors;
	err = lr20xx_system_get_errors( NULL, &errors );
	printf("err4:%d\r\n",err);
	if(err != LR20XX_STATUS_OK) return err;
	if(errors != 0) 
	{
		err = lr20xx_system_clear_errors(NULL);
		printf("err5:%d\r\n",err);
		if(err != LR20XX_STATUS_OK) return err;
	}
	err = lr20xx_system_clear_irq_status(NULL, LR20XX_SYSTEM_IRQ_ALL_MASK);
	printf("err7:%d\r\n",err);
	if(err != LR20XX_STATUS_OK) return err;
	lr20xx_system_set_dio_function( NULL, LR20XX_SYSTEM_DIO_9, LR20XX_SYSTEM_DIO_FUNC_IRQ, LR20XX_SYSTEM_DIO_DRIVE_PULL_UP );
	lr20xx_system_set_dio_irq_cfg( NULL, LR20XX_SYSTEM_DIO_9,  NULL, LR20XX_SYSTEM_DIO_9, LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR);
	lr20xx_system_cfg_clk_output( NULL, LR20XX_SYSTEM_HF_CLK_SCALING_32_MHZ );
	
	//TCXO
	if(lr202x_tcxo) 
	{
		err = lr20xx_system_set_tcxo_mode(NULL,LR20XX_SYSTEM_TCXO_CTRL_3_0V,64000); //check
		printf("err3:%d\r\n",err);
		if(err != LR20XX_STATUS_OK) return err;
	}
	
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
	LR20xx_bsp_get_front_end_calibration_cfg( NULL, front_end_calibration_structures );
	err = lr20xx_radio_common_calibrate_front_end_helper( NULL, front_end_calibration_structures, 3 );
	printf("err11:%d\r\n",err);
	
	if(err != LR20XX_STATUS_OK) return err;
	err = lr20xx_system_get_version(NULL, &version);
	printf("err12:%d\r\n",err);			
	if(err != LR20XX_STATUS_OK) return err;
	err = LR202x_set_freq(radioconfig.freq);
	printf("err13:%d\r\n",err);	
	if(err != LR20XX_STATUS_OK) return err;
	err = lr20xx_radio_common_set_tx_params(NULL,radioconfig.txpower*2, LR20XX_RADIO_COMMON_RAMP_16_US);
	printf("err14:%d\r\n",err);	
	if(err != LR20XX_STATUS_OK) return err;
	err = lr20xx_radio_common_set_pkt_type(NULL, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
	printf("err15:%d\r\n",err);	
	if(err != LR20XX_STATUS_OK) return err;
	err = LR202x_set_mod_params(radioconfig.bw,radioconfig.sf,radioconfig.cr,radioconfig.ldropt);
	printf("err16:%d\r\n",err);	
	if(err != LR20XX_STATUS_OK) return err;
	err = LR202x_set_packet_params(radioconfig.sync,radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	printf("err17:%d\r\n",err);	
	if(err != LR20XX_STATUS_OK) return err;
	err = lr20xx_radio_common_set_rx_tx_fallback_mode(NULL, LR20XX_RADIO_FALLBACK_STDBY_XOSC);	//???
	printf("err18:%d\r\n",err);	
	if(err != LR20XX_STATUS_OK) return err;
	//DIO,IRQ
	
	LR202x_setopmode(RADIO_OPMODE_RX);
	return RADIO_OK;
}

lr20xx_status_t LR202x_set_freq(uint32_t Hz)
{
	lr20xx_status_t err = lr20xx_radio_common_set_rf_freq(NULL, Hz);
	if(err != LR20XX_STATUS_OK) return err;
	if (Hz < 1900000000) 
	{ // SubGHz
		err = lr20xx_radio_common_set_pa_cfg(NULL, &pa_config_lf);
		if(err != LR20XX_STATUS_OK) return err;
		err = lr20xx_radio_common_set_rx_path( NULL,LR20XX_RADIO_COMMON_RX_PATH_LF,LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
		if(err != LR20XX_STATUS_OK) return err;
	}
	else 
	{
		err = lr20xx_radio_common_set_pa_cfg(NULL, &pa_config_hf);
		if(err != LR20XX_STATUS_OK) return err;
		err = lr20xx_radio_common_set_rx_path( NULL,LR20XX_RADIO_COMMON_RX_PATH_HF,LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
		if(err != LR20XX_STATUS_OK) return err;
	}
	return LR20XX_STATUS_OK;
}

lr20xx_status_t LR202x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
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

lr20xx_status_t LR202x_set_packet_params(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
	lr20xx_radio_lora_pkt_params_t pktparams;
	
	pktparams.preamble_len_in_symb = prelen;
	pktparams.pld_len_in_bytes = paylen;
	pktparams.pkt_mode = (lr20xx_radio_lora_pkt_mode_t)header;
	pktparams.crc = (lr20xx_radio_lora_crc_t)crc;
	pktparams.iq = (lr20xx_radio_lora_iq_t)invertiq;
	lr20xx_status_t err = lr20xx_radio_lora_set_packet_params(NULL,&pktparams);
	if(err != LR20XX_STATUS_OK) return err;
	return (int8_t)lr20xx_radio_lora_set_syncword(NULL,sync);
}

void LR202x_setopmode(uint8_t mode)
{
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
		{
			opmode = RADIO_OPMODE_SLEEP;
			lr20xx_system_sleep_cfg_t sleepcfg;
			sleepcfg.is_clk_32k_enabled = false;
			sleepcfg.is_ram_retention_enabled = false;
			lr20xx_system_set_sleep_mode(NULL,&sleepcfg,0);
			break;
		}

    case RADIO_OPMODE_STBYRC:
		opmode = RADIO_OPMODE_STBYRC;
    lr20xx_system_set_standby_mode(NULL,LR20XX_SYSTEM_STANDBY_MODE_RC);
    break;

    case RADIO_OPMODE_STBYXOSC:
		opmode = RADIO_OPMODE_STBYXOSC;
    lr20xx_system_set_standby_mode(NULL,LR20XX_SYSTEM_STANDBY_MODE_XOSC);
    break;

    case RADIO_OPMODE_FS:
		opmode = RADIO_OPMODE_FS;
    lr20xx_system_set_fs_mode(NULL);
    break;

    case RADIO_OPMODE_TX:
		opmode = RADIO_OPMODE_TX;
    lr20xx_radio_common_set_tx(NULL,0); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
		opmode = RADIO_OPMODE_RX;
    lr20xx_radio_common_set_rx(NULL,0xffffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
		opmode = RADIO_OPMODE_TXSTREAMCW;
    lr20xx_radio_common_set_tx_test_mode(NULL,LR20XX_RADIO_COMMON_TX_TEST_MODE_CONTINUOUS_WAVE);
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
		opmode = RADIO_OPMODE_TXSTREAMPRE;
    lr20xx_radio_common_set_tx_test_mode(NULL,LR20XX_RADIO_COMMON_TX_TEST_MODE_INFINITE_PREAMBLE);
    break;
		
		//add PN9
  }
	//LR112X_printerrors(0);
}

void LR20xx_RssiCal(uint32_t freq)
{
//	if(freq <= 60000000) lr11xx_radio_set_rssi_calibration(NULL,&calib_0_600);
//	else if((freq > 60000000) && (freq <= 200000000)) lr11xx_radio_set_rssi_calibration(NULL,&calib_600_2000);
//	else lr11xx_radio_set_rssi_calibration(NULL,&calib_2000_2700);
}

void LR20xx_printstatus(void)
{
	lr20xx_system_stat1_t stat1;
	lr20xx_system_stat2_t stat2;
	lr20xx_system_irq_mask_t irqstatus;
	lr20xx_system_get_status(NULL,&stat1,&stat2,&irqstatus);

	printf("Status=0x%02X,0x%02X\r\n",stat1.command_status,stat2.chip_mode); //mask reset source	`		
}

void LR20xx_printerrors(void)
{
	uint16_t errors;
	lr20xx_system_get_errors(NULL,&errors);
	printf("Errors: 0x%04X\r\n",errors);
}

void LR20xx_bsp_get_front_end_calibration_cfg(const void* context, lr20xx_radio_common_front_end_calibration_value_t *front_end_calibration_structures)
{
    lr20xx_radio_common_rx_path_t            rx_path    = LR20XX_RADIO_COMMON_RX_PATH_LF;
    lr20xx_radio_common_rx_path_boost_mode_t boost_mode = LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE;

    uint32_t freq_in_hz[3] = {
        470000000,   // Frequency 0 (range from 430MHz to 510MHz)
        897500000,   // Frequency 1 (range from 867MHz to 928MHz)
        2441000000,  // Frequency 2 (range from 2.403GHz to 2.479GHz)
    };

    for( uint8_t i = 0; i < 3; i++ )
    {
        LR20xx_bsp_get_rx_cfg( context, freq_in_hz[i], &rx_path, &boost_mode );
        front_end_calibration_structures[i].rx_path = rx_path;
        front_end_calibration_structures[i].frequency_in_hertz = freq_in_hz[i];
    };
}

void LR20xx_bsp_get_rx_cfg( const void* context, const uint32_t freq_in_hz, lr20xx_radio_common_rx_path_t* rx_path,lr20xx_radio_common_rx_path_boost_mode_t* boost_mode )
{
    if( freq_in_hz >= 1600000000 )  // 1.6GHz
    {
        *rx_path = LR20XX_RADIO_COMMON_RX_PATH_HF;
    }
    else
    {
        *rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF;
    }

    *boost_mode = LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE;
}

const lr20xx_radio_common_pa_cfg_t pa_config_lf = 
{
	.pa_sel = LR20XX_RADIO_COMMON_PA_SEL_LF,
	.pa_lf_mode = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
	.pa_lf_duty_cycle = 7,
	.pa_lf_slices = 6,
	.pa_hf_duty_cycle = 16
};

const lr20xx_radio_common_pa_cfg_t pa_config_hf =
{
	.pa_sel = LR20XX_RADIO_COMMON_PA_SEL_HF,
	.pa_lf_mode = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
	.pa_lf_duty_cycle = 7,
	.pa_lf_slices = 6,
	.pa_hf_duty_cycle = 16
};


