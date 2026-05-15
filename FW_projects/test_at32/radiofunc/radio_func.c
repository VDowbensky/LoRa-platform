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
	radioconfig.crc = 1;
	radioconfig.invertiq = 0;
	radioconfig.txpower = 10;
	
	switch(chip)
  {
    case 1262:
    radioconfig.chip = 1262;
		radioconfig.freq = 433125000;
		radioconfig.bw = 250; //LORA_BW_250;
		radioconfig.paylen = 0;
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
		radioconfig.paylen = 0;
		break;

    case 1121:
    radioconfig.chip = 1121;
    radioconfig.freq = 433125000;
		radioconfig.bw = 250; //LORA_BW_250;
		radioconfig.paylen = 255;
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
		radioconfig.paylen = 255;
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
		radioconfig.paylen = 0;
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
	int8_t err = RADIO_OK;
	switch(radioconfig.chip)
	{
		case 1262:
		err = (int8_t)SX126x_init();
		break;
		case 1280:
		err = (int8_t)SX128x_init();
		break;
		case 1121:
		err = (int8_t)LR112x_init();
		break;
		case 2021:
		err = (int8_t)LR202x_init();
		break;
		case 3029:
		err = PAN3029_init();
		break;
		default:
		err = INVALID_CHIP;
		break;
	}
	radio_rx();
	return err;
}

//Common functions
//set RF frequency
int8_t radio_set_freq(uint32_t khz)
{
	int8_t err = RADIO_OK;
	currfreq = khz;
	prevopmode = opmode;
	radio_setopmode(RADIO_OPMODE_STBYXOSC);
	switch(radioconfig.chip)
	{
		case 1262:
		err = (int8_t)sx126x_set_rf_freq(NULL,khz * 1000);
		break;
			
		case 1280:
		err = (int8_t)sx128x_set_rf_freq(NULL,khz * 1000);
		break;
			
		case 1121:
		err = (int8_t)LR112X_set_freq(khz * 1000);
		break;
			
		case 2021:
		err = (int8_t)LR202x_set_freq(khz * 1000);
		break;
			
		case 3029:
		PAN_SetFreq(khz * 1000);
		break;
			
		default:
		err = INVALID_CHIP;
		break;
	}
	radio_setopmode(prevopmode);
	return err;
}

int8_t radio_set_power(int8_t dbm)
{
	int8_t err = RADIO_OK;
	prevopmode = opmode;
	radio_setopmode(RADIO_OPMODE_STBYXOSC);
	switch(radioconfig.chip)
	{
		case 1262:
		if((dbm < -9) || (dbm > 22)) err = RADIO_INVALID_PARAMETER;
		else err = (int8_t)sx126x_set_tx_params(NULL,dbm,SX126X_RAMP_10_US);
		break;
		
		case 1280:
		if((dbm < -18) || (dbm > 13)) err = RADIO_INVALID_PARAMETER;
		else err = (int8_t)sx128x_set_tx_params(NULL,dbm,SX128X_RAMP_10_US);
		break;
		
		case 1121:
//		- 17dBm (0xEF) to +14dBm (0x0E) by steps of 1dB if the low power PA is selected
//		- 9dBm (0xF7) to +22dBm (0x16) by steps of 1dB if the high power PA is selected
//		-18dBm (0xEE) to +13dBm (0x0F) by steps of 1dB if the high frequency PA is selected
		if((dbm < -18) || (dbm > 22)) err = RADIO_INVALID_PARAMETER; //must be corrected
		else err = (int8_t)lr11xx_radio_set_tx_params(NULL,dbm,LR11XX_RADIO_RAMP_16_US);
		break;
		
		case 2021:
//		PA_LF: [-19: 44] corresponding to output power range of [-9.5:22]dBm
//		PA_HF: [-39: 24] corresponding to output power range of [-19.5:12]dBm
		if((dbm < -19) || (dbm > 22)) err = RADIO_INVALID_PARAMETER; //must be corrected
		else err = (int8_t)lr20xx_radio_common_set_tx_params(NULL,dbm*2, LR20XX_RADIO_COMMON_RAMP_16_US);
		break;
		
		case 3029:
		if((dbm < 0) || (dbm > 20)) err = RADIO_INVALID_PARAMETER;
		else PAN_setpower(dbm);
		break;
		
		default:
		err = INVALID_CHIP;
		break;
	}
	radio_setopmode(prevopmode);
	return err;
}

//set modulation parameters
int8_t radio_setmodparams(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
{
  int8_t err = RADIO_OK;
	prevopmode = opmode;
	radio_setopmode(RADIO_OPMODE_STBYXOSC);
	switch(radioconfig.chip)
  {
    case 1262:
		err = (int8_t)SX126x_set_mod_params(bw_khz,sf,cr,ldropt);
		break;
		
    case 1280:
		err = (int8_t)SX128x_set_mod_params(bw_khz,sf,cr);
		break;
		
    case 1121:
		err = (int8_t)LR112x_set_mod_params(bw_khz,sf,cr,ldropt);
		break;
		
		case 2021:
		err = (int8_t)LR202x_set_mod_params(bw_khz,sf,cr,ldropt);
		break;
		
		case 3029:
		PAN_set_mod_params(bw_khz,sf,cr,ldropt);
		break;
		
    default:
    err = INVALID_CHIP;
		break;
  }
	radio_setopmode(prevopmode);
	return err;
}

//set packet parameters
int8_t radio_setpktparams(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
  int8_t err = RADIO_OK;
	prevopmode = opmode;
	radio_setopmode(RADIO_OPMODE_STBYXOSC);
	switch(radioconfig.chip)
  {
    case 1262:
		err = (int8_t)SX126x_set_packet_params(sync,prelen,paylen,header,crc,invertiq);
		break;
		
    case 1280:
		err = (int8_t)SX128x_set_packet_params(sync,prelen,paylen,header,crc,invertiq);
		break;
		
    case 1121:
		err = (int8_t)LR112x_set_packet_params(sync,prelen,paylen,header,crc,invertiq);
		break;
		
		case 2021:
		LR202x_set_packet_params(sync,prelen,paylen,header,crc,invertiq);
		break;
		
		case 3029:
		PAN_set_packet_params(sync,prelen,paylen,header,crc,invertiq);
		break;
    
    default:
    return INVALID_CHIP;
  }
	radio_setopmode(prevopmode);
	return err;
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
			status->snr_pkt = pktstatus.snr_pkt_raw / 4.0;
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
		SX126X_irq_handler();
		break;  

    case 1280:
		SX128X_irq_handler();
		break;
		
    case 1121:
		LR112X_irq_handler();
		
		case 2021:
		LR20xx_irq_handler();
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
			*dbm = rssi - half_dbm_cnt / 2.0f; //to be checked
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
	
	switch(stream)
	{
		case 0:
		default:
		txmode = 0;
		txled_off();
		if(radioconfig.chip == 3029) 
		{
			PAN_StopTxContinuousWave();
		}
		err = radio_rx();	
		break;
		
		case 1: //CW
		err = radio_setopmode(RADIO_OPMODE_TXSTREAMCW);
		if(err == RADIO_OK) 
		{
			txled_on();
			txmode = 1;
		}
		break;
		
		case 2:
		err = radio_setopmode(RADIO_OPMODE_TXSTREAMPRE);
		if(err == RADIO_OK)
		{
			txled_on();
			txmode = 2;
		}
		break;
	}
	return err;
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
