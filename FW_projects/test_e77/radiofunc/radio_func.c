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

//bool pre_detected = false;
bool header_error = false;


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
	
	radioconfig.txstartfreq = 400000;
	radioconfig.txstopfreq = 1000000;
	radioconfig.txstep = 100;
	radioconfig.txinterval = 10;
	radioconfig.txmodulation = TXMOD_CW; //CW
	//scanner parameters
	radioconfig.rxstartfreq = 400000;
	radioconfig.rxstopfreq = 1000000;
	radioconfig.rxstep = 100;
	radioconfig.rxinterval = 100;
	radioconfig.rssitr = -100.0;
	radioconfig.pktformat = PKT_MESHTASTIC; //meshtastic
	radioconfig.txsendinterval = 5000; //5 sec.
	radioconfig.pair_id = 0;
	radioconfig.pkt_count = 10;
	
	
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
		
	writeconfig();
	radio_init();
	return RADIO_OK;
}

int8_t radio_init(void)
{
	int8_t err = (int8_t)SX126x_init();
	if(err != RADIO_OK) return err;
	workmode = WORK_MODE_PACKET;
	radio_rx();
	return RADIO_OK;
}

//Common functions
//set RF frequency
int8_t radio_set_freq(uint32_t khz)
{
	int8_t err = RADIO_OK;
	currfreq = khz;
	prevopmode = opmode;
	radio_setopmode(RADIO_OPMODE_STBYXOSC);
	err = (int8_t)sx126x_set_rf_freq(NULL,khz * 1000);
	radio_setopmode(prevopmode);
	return err;
}

int8_t radio_set_power(int8_t dbm)
{
	int8_t err = RADIO_OK;
	prevopmode = opmode;
	radio_setopmode(RADIO_OPMODE_STBYXOSC);
	if((dbm < -9) || (dbm > 22)) err = RADIO_INVALID_PARAMETER;
	else err = (int8_t)sx126x_set_tx_params(NULL,dbm,SX126X_RAMP_10_US);
	radio_setopmode(prevopmode);
	return err;
}

//set modulation parameters
int8_t radio_setmodparams(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
{
  int8_t err = RADIO_OK;
	prevopmode = opmode;
	radio_setopmode(RADIO_OPMODE_STBYXOSC);
	err = (int8_t)SX126x_set_mod_params(bw_khz,sf,cr,ldropt);
	radio_setopmode(prevopmode);
	return err;
}

//set packet parameters
int8_t radio_setpktparams(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
  int8_t err = RADIO_OK;
	prevopmode = opmode;
	radio_setopmode(RADIO_OPMODE_STBYXOSC);
	err = (int8_t)SX126x_set_packet_params(sync,prelen,paylen,header,crc,invertiq);
	radio_setopmode(prevopmode);
	return err;
}

//send one packet
int8_t radio_sendpacket(uint8_t *buf)
{
  int8_t err = radio_setpktparams(radioconfig.sync,radioconfig.prelen,txlen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	if(err != RADIO_OK) return err;
	err = (int8_t)sx126x_write_buffer(NULL,0,buf,txlen);
	if(err != RADIO_OK) return err;
	return radio_setopmode(RADIO_OPMODE_TX);
}
//retrieve packet info
int8_t radio_getpktstatus(rxpacketstatus_t *status)
{
  int8_t err;
	sx126x_pkt_status_lora_t pktstatus;
	err = (int8_t)sx126x_get_lora_pkt_status(NULL,&pktstatus);
	if(err != RADIO_OK) return err;
	status->rssi_pkt = pktstatus.rssi_pkt_in_dbm;
	status->snr_pkt = pktstatus.snr_pkt_in_db; 
	status->signal_rssi_pkt = pktstatus.signal_rssi_pkt_in_dbm;
	return RADIO_OK;
}

//receive one packet
int8_t radio_getpacket(uint8_t *buf)
{
  int8_t err;
	sx126x_rx_buffer_status_t status;
	err = (int8_t)sx126x_get_rx_buffer_status(NULL,&status);
	if(err != RADIO_OK) return err;
	rxlen = status.pld_len_in_bytes;
	return(int8_t)sx126x_read_buffer(NULL,status.buffer_start_pointer,buf,rxlen);
}

//helpers
int8_t radio_getstats(rxstats_t *stats)
{
  int8_t err;
	sx126x_stats_lora_t lora_stats;
	err = (int8_t)sx126x_get_lora_stats(NULL,&lora_stats);
	if(err != RADIO_OK) return err;
	stats->pkt_received = lora_stats.nb_pkt_received;
	stats->crc_error = lora_stats.nb_pkt_crc_error;
	stats->header_error = lora_stats.nb_pkt_header_error;
	stats->false_sync = 0;
	return RADIO_OK;
}

int8_t radio_clearstats(void)
{
	return(int8_t)sx126x_reset_stats(NULL);
}

//irq handler
void radio_irq_handler(void)
{
	SX126X_irq_handler();
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
	int16_t rssi;
	err = (int8_t)sx126x_get_rssi_inst(NULL,&rssi);
	if(err != RADIO_OK) return err;
	*dbm = rssi;
	return RADIO_OK;
}

int8_t radio_stream(uint8_t stream)
{
  int8_t err;
	
	switch(stream)
	{
		case 0:
		default:
		txmode = 0;
		led_off();
		err = radio_rx();	
		break;
		
		case 1: //CW
		err = radio_setopmode(RADIO_OPMODE_TXSTREAMCW);
		if(err == RADIO_OK) 
		{
			led_on();
			txmode = 1;
		}
		break;
		
		case 2:
		err = radio_setopmode(RADIO_OPMODE_TXSTREAMPRE);
		if(err == RADIO_OK)
		{
			led_on();
			txmode = 2;
		}
		break;
	}
	return err;
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
	if(trim > 94) return RADIO_INVALID_PARAMETER;
	sx126x_xtatrim = trim / 2;
	sx126x_xtbtrim = trim - sx126x_xtatrim;
	prevopmode = opmode;
	SX126X_setopmode(RADIO_OPMODE_STBYXOSC);
	sx126x_set_trimming_capacitor_values(NULL,sx126x_xtatrim,sx126x_xtbtrim);
	SX126X_setopmode(prevopmode);
	return RADIO_OK;
}

int8_t radio_getxotrim(uint8_t *trim)
{
	uint8_t buf[2];
	sx126x_read_register(NULL,SX126X_REG_XTATRIM,buf,2);
	*trim = buf[0] + buf[1];
	return RADIO_OK;
}

int8_t radio_readreg(uint32_t reg,uint32_t *val)
{
	uint8_t regval;
	if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
	sx126x_read_register(NULL,reg,&regval,1);
	*val = regval;
	return RADIO_OK;
}

int8_t radio_writereg(uint32_t reg,uint32_t val)
{
	uint8_t regval = val & 0xff;
	if(reg > 0xffff) return RADIO_INVALID_PARAMETER;
	sx126x_write_register(NULL,reg,&regval,1);
	return RADIO_OK;
}

int8_t radio_get_chip_version(uint8_t *hw,uint8_t *use_case,uint8_t *fw_major,uint8_t *fw_minor)
{
	return RADIO_TODO;
}

int8_t radio_get_status(uint8_t *chip_mode,uint8_t *cmd_status)
{
	sx126x_chip_status_t status;
	sx126x_get_status(NULL,&status);
	*chip_mode = status.chip_mode;
	*cmd_status = status.cmd_status;
	return RADIO_OK;
}

int8_t radio_setopmode(uint8_t mode)
{
	int8_t err = SX126X_setopmode(mode);
	if(err != RADIO_OK) return err;
	opmode = mode;
	return RADIO_OK;
}
