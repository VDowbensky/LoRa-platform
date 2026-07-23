#include "radio_func.h"

//globals
int8_t minpower;
int8_t maxpower;

uint8_t txmode;

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
radioconfig_t radioconfig __attribute__((aligned (4)));
rxpacketstatus_t pktstatus;

//init radio
int8_t radio_initconfig(uint16_t chip,uint8_t tcxo)
{
	radioconfig.chip = 1262;
	radioconfig.id = 0;
	radioconfig.freq = 433125000;
	radioconfig.txpower = 10;
	//modulation
	radioconfig.sf = LORA_SF_11;
	radioconfig.bw_index = 8; //LORA_BW_250;
	radioconfig.cr = LORA_CR_4_5;
	radioconfig.ldropt = 0;
	//packet
	radioconfig.sync = 0x24b4;
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
	writeconfig();
	radio_init();
	return RADIO_OK;
}

int8_t radio_init(void)
{
  currfreq = radioconfig.freq / 1000;
	prevfreq = radioconfig.freq / 1000;

	SX126X_reset();
	SX126X_Wakeup();
	delay_ms(100);
	SX126X_setopmode(RADIO_OPMODE_STBYRC);
	SX126X_SetRegulatorMode(true);
	delay_ms(100);
	//set TCXO here if needed
	if(sx126x_tcxo != 0)
	{
		SX126X_SetDIO3AsTCXOCtrl(sx126x_tcxo_voltage,5000);
		//SX126X_Calibrate(true,true,true,true,true,true,true);
		SX126X_ClearDeviceErrors();
	}
	SX126X_SetPacketType(SX126X_MODEM_LORA);
	SX126X_SetRfFrequency((uint32_t)(radioconfig.freq / SX126X_SYNTH_STEP));
	SX126X_SetBufferBaseAddress(0,0);
	SX126X_SetLoRaModParams(SX126X_bw[radioconfig.bw_index],radioconfig.sf,radioconfig.cr,radioconfig.ldropt);
	SX126X_SetLoRaPacketParams(radioconfig.prelen,radioconfig.header,radioconfig.paylen,radioconfig.crc,radioconfig.invertiq);
	SX126X_writeReg(SX126X_REG_LRSYNC_H,(radioconfig.sync & 0xff00) >> 8);
	SX126X_writeReg(SX126X_REG_LRSYNC_L,radioconfig.sync & 0xff);
	SX126X_setopmode(RADIO_OPMODE_RX);
	SX126X_CalibrateIR();

	SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
	if(sx126x_tcxo == 0) //TCXO off
	{
		//SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
		SX126X_writeReg(SX126X_REG_XTATRIM,sx126x_xtatrim);
		SX126X_writeReg(SX126X_REG_XTBTRIM,sx126x_xtbtrim);
	}
	delay_ms(100); //??? without this delay tx amp not activated
	//SX126X_restart_agc();
	//SX126X_CalibrateIR();
	SX126X_ClearDeviceErrors();
	SX126X_SetTxParams();
	SX126X_LNAboost(true);
	//SX126X_SetDIO2AsRfSwitchCtrl(true); //for E77 not needed!
	SX126X_SetDioIrqParams(SX126X_TXDONE_IRQMSK | SX126X_RPEDET_IRQMSK | SX126X_SYNCDET_IRQMSK | SX126X_RXDONE_IRQMSK | SX126X_CRCERR_IRQMSK, SX126X_TXDONE_IRQMSK | SX126X_RPEDET_IRQMSK | SX126X_SYNCDET_IRQMSK | SX126X_RXDONE_IRQMSK, 0, 0);
	SX126X_setopmode(RADIO_OPMODE_RX);
	//SX126X_restart_agc();
	return RADIO_OK;
}


//set RF frequency
int8_t radio_set_freq(uint32_t khz)
{
  currfreq = khz;
	radioconfig.freq = khz * 1000;
	prevopmode = opmode;
	SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
	SX126X_SetRfFrequency((uint32_t)(radioconfig.freq / SX126X_SYNTH_STEP));
	SX126X_setopmode(prevopmode);
	return RADIO_OK;
}

//set tx power
int8_t radio_set_power(int8_t dbm)
{
	if((dbm < -9) || (dbm > 22)) return RADIO_INVALID_PARAMETER;
	prevopmode = opmode;
	SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
	//SX126X_setopmode(RADIO_OPMODE_STBYRC);
	radioconfig.txpower = dbm;
	SX126X_SetTxParams();
	SX126X_setopmode(prevopmode);
	return RADIO_OK;
}

//set modulation parameters
int8_t radio_setmodparams(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
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
	prevopmode = opmode;
	SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
	SX126X_SetLoRaModParams(sf,SX126X_bw[bw_index],cr,ldropt); //
	SX126X_setopmode(prevopmode);
	return RADIO_OK;
}

int8_t radio_getmodparams(uint16_t *bw_khz,uint8_t *sf,uint8_t *cr,uint8_t *ldropt)
{
	*sf = radioconfig.sf;
	*bw_khz = SX126X_bw_kHz[radioconfig.bw_index];
	*cr = radioconfig.cr;
	*ldropt = radioconfig.ldropt;
	return RADIO_OK;
}

//set packet parameters
int8_t radio_setpktparams(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
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
	SX126X_SetLoRaPacketParams(prelen,paylen,(bool)header,(bool)crc,(bool)invertiq);
	SX126X_writeReg(SX126X_REG_LRSYNC_H,(sync & 0xff00) >> 8);
	SX126X_writeReg(SX126X_REG_LRSYNC_L,sync & 0xff);
	SX126X_setopmode(prevopmode);
	return RADIO_OK;
}

//send one packet
int8_t radio_sendpacket(uint8_t *buf)
{
	SX126X_SetLoRaPacketParams(radioconfig.prelen,txlen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	SX126X_writeBuffer(0,buf,txlen);
	SX126X_setopmode(RADIO_OPMODE_TX);
	return RADIO_OK;
}
//retrieve packet info
int8_t radio_getpktstatus(rxpacketstatus_t *status)
{
	uint8_t dummy;
	uint8_t rssipkt;
	int16_t snrpkt;
	uint8_t signalrssi;
		
	SX126X_GetLoRaPacketStatus(&dummy,&rssipkt,&snrpkt,&signalrssi);
	status->rssi_pkt = -((float)rssipkt/2);
	if(snrpkt < 128) status->snr_pkt = ((float)snrpkt)/4;
	else status->snr_pkt = (float)(snrpkt - 256)/4;
	status->signal_rssi_pkt = -((float)signalrssi/2);
	return RADIO_OK;
}

//receive one packet
int8_t radio_getpacket(uint8_t *buf)
{
	uint8_t dummy;
	uint8_t rxpointer;
	
	SX126X_GetRxBufferStatus(&dummy,&rxlen,&rxpointer);
	SX126X_readBuffer(rxpointer,buf,rxlen);
	return RADIO_OK;
}

//helpers
int8_t radio_getstats(rxstats_t *stats)
{
	uint8_t dummy;
	SX126X_LoRaGetStats(&dummy,&stats->pkt_received,&stats->crc_error,&stats->header_error);
	stats->false_sync = 0;
	return RADIO_OK;
}

int8_t radio_clearstats(void)
{
	SX126X_ResetStats();
	return RADIO_OK;
}

//irq handler
void radio_irq_handler(void)
{
	//read SX126x status
	uint16_t irqstatus = SX126X_GetIrqStatus();
	SX126X_ClearIrqStatus(SX126X_ALL_IRQMSK);
	if(irqstatus & SX126X_TXDONE_IRQMSK) packet_sent = true;
	if(irqstatus & SX126X_RXDONE_IRQMSK) packet_received = true;
	if(irqstatus & SX126X_CRCERR_IRQMSK) crc_error = true;
	//if(irqstatus & SX126X_RPEDET_IRQMSK) {};
	//if(irqstatus & SX126X_SYNCDET_IRQMSK) {};
	//if(irqstatus & SX126X_HEADERDET_IRQMSK) {};
	//if(irqstatus & SX126X_HEADERERR_IRQMSK) {};
	//if(irqstatus & SX126X_CADDONE_IRQMSK) {};
	//if(irqstatus & SX126X_CADDET_IRQMSK) {};
	//if(irqstatus & SX126X_TIMEOUT_IRQMSK) {};
	//if(irqstatus & SX126X_LRFHSSHOP_IRQMSK) {};
}

//working modes
int8_t radio_rx(void)
{
	SX126X_SetLoRaPacketParams(radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	SX126X_setopmode(RADIO_OPMODE_RX);
	return RADIO_OK;
}

int8_t radio_getrssi(float *dbm)
{
	*dbm = SX126X_GetRssiInst()/-2.0f;
	return RADIO_OK;
}

int8_t radio_stream(uint8_t stream)
{
  if(stream == 0) 
	{
		txmode = 0;
		led_off();
		return radio_rx();
	}
	if(stream > 2) return RADIO_INVALID_PARAMETER;
	//if(txmode != 0) return RADIO_INVALID_MODE;
	//prevopmode = opmode;
	if(stream == 1) 
	{
		SX126X_setopmode(RADIO_OPMODE_TXSTREAMCW);
		txmode = 1;
	}
	else 
	{
		SX126X_setopmode(RADIO_OPMODE_TXSTREAMPRE);
		txmode = 2;
	}
	led_on();
	return RADIO_OK;
}

int8_t radio_sleep(uint8_t node)
{
	return RADIO_TODO;
}

int8_t radio_wakeup(uint8_t mode)
{
	return RADIO_TODO;
}

int8_t radio_setxotrim(uint8_t trim)
{
	if(sx126x_tcxo != 0) return FEATURE_NOT_SUPPORTED;
	if(trim > 94) return RADIO_INVALID_PARAMETER;
	sx126x_xtatrim = trim / 2;
	sx126x_xtbtrim = trim - sx126x_xtatrim;
	prevopmode = opmode;
	SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
	SX126X_writeReg(SX126X_REG_XTATRIM,sx126x_xtatrim);
	SX126X_writeReg(SX126X_REG_XTBTRIM,sx126x_xtbtrim);
	SX126X_setopmode(prevopmode);
	return RADIO_OK;
}

int8_t radio_getxotrim(uint8_t *trim)
{
	*trim = SX126X_readReg(SX126X_REG_XTATRIM) + SX126X_readReg(SX126X_REG_XTBTRIM);
	return RADIO_OK;
}

int8_t radio_readreg(uint32_t reg,uint32_t *val)
{
	if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
	*val = SX126X_readReg(reg);
	return RADIO_OK;
}

int8_t radio_writereg(uint32_t reg,uint32_t val)
{
	if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
	if(val > 0xff) return RADIO_INVALID_PARAMETER;
	SX126X_writeReg(reg,val);
	return RADIO_OK;
}

int8_t radio_get_chip_version(uint8_t *hw,uint8_t *use_case,uint8_t *fw_major,uint8_t *fw_minor)
{
	return RADIO_TODO;
}

int8_t radio_get_status(uint8_t *stat1,uint8_t *stat2)
{
	return RADIO_TODO;
}

uint8_t radio_setopmode(uint8_t mode)
{
	SX126X_setopmode(mode);
	return RADIO_OK;
}



