#include "sx127x_proc.h"

const float synthStep = 32000000 / 524288.0;

uint8_t SX127x_calc_bw_value(uint32_t bw_khz);

int8_t SX127x_init(void)
{
	SX127x_reset();
	//set modem
	SX127x_setopmode(OPMODE_SLEEP);
	SX127x_write_reg(REG_OPMODE,OPMODE_LORA);
	//maybe small delay
	delay_ms(1000);
	SX127x_setopmode(RADIO_OPMODE_STBYRC);
	delay_ms(1000);
	SX127x_set_rf_freq(radioconfig.freq,true);
	//set parameters
	SX127x_set_tx_params(radioconfig.txpower,PARAMP_10);
	SX127x_set_mod_params(radioconfig.bw,radioconfig.sf,radioconfig.cr,radioconfig.ldropt);
	SX127x_set_packet_params(radioconfig.sync,radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	//RX IR calibration - ?
	//DIO, IRQ
	return RADIO_OK;
}


int8_t SX127x_set_rf_freq(uint32_t freq,bool rx)
{
	uint32_t freqoffset;
	uint32_t freqval = (uint32_t)(freq / synthStep);
	uint8_t bw_value = SX127x_calc_bw_value(radioconfig.bw);
	if(rx)
	{
		//fix errata
		if(bw_value == LR_BW_500) 
		{
			SX127x_write_reg(REG_LR_DETOPT,SX127x_read_reg(REG_LR_DETOPT) | LR_DETOPT_AUTOIF_ON);
		}
		else
		{
			SX127x_write_reg(REG_LR_DETOPT,SX127x_read_reg(REG_LR_DETOPT) & ~LR_DETOPT_AUTOIF_ON);
			SX127x_write_reg(REG_LR_IFFREQ2,0);
			switch(bw_value)
			{
				case LR_BW_7P8:
				SX127x_write_reg(REG_LR_IFFREQ1,0x48);
				freqoffset = (uint32_t)(7812.5 / synthStep);
				break;
				
				case LR_BW_10P4:
				SX127x_write_reg(REG_LR_IFFREQ1,0x44);
				freqoffset = (uint32_t)(10417.5 / synthStep);
				break;
				
				case LR_BW_15P6:
				SX127x_write_reg(REG_LR_IFFREQ1,0x44);
				freqoffset = (uint32_t)(15625 / synthStep);
				break;
				
				case LR_BW_20P8:
				SX127x_write_reg(REG_LR_IFFREQ1,0x44);
				freqoffset = (uint32_t)(20833 / synthStep);
				break;
				
				case LR_BW_31P25:
				SX127x_write_reg(REG_LR_IFFREQ1,0x44);
				freqoffset = (uint32_t)(31250 / synthStep); 
				break;
				
				case LR_BW_41P7:
				SX127x_write_reg(REG_LR_IFFREQ1,0x44);
				freqoffset = (uint32_t)(41666 / synthStep); 
				break;
				
				case LR_BW_62P5:
				case LR_BW_125:
				case LR_BW_250:	
				default:
				SX127x_write_reg(REG_LR_IFFREQ1,0x40);
				freqoffset = 0;
				break;
			}
			freqval += freqoffset;
		}
	}
	SX127x_write_reg(REG_FRF_MSB,(freqval >> 16) & 0xff);
	SX127x_write_reg(REG_FRF_MID,(freqval >> 8) & 0xff);
	SX127x_write_reg(REG_FRF_LSB,freqval & 0xff);
	return RADIO_OK;
}

uint32_t SX127x_get_rf_freq(void)
{
	uint8_t buf[3];
	SX127x_read_regs(REG_FRF_MSB,buf,3);
	uint32_t tmp = (((uint32_t)buf[0]) << 16) | (((uint32_t)buf[1]) << 8) | buf[2];
	float f = synthStep * tmp;
	return (uint32_t)f;
}

int8_t SX127x_set_tx_params(int8_t power,uint8_t ramptime)
{
	SX127x_write_reg(REG_PACONFIG,PACONFIG_PABOOST | (power - 2));
	SX127x_write_reg(REG_PARAMP,ramptime);
	SX127x_write_reg(REG_OCP,OCP_TRIM_120);
	return RADIO_OK;
}

int8_t SX127x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
{
	uint8_t regval = 0;
	uint8_t bw_value = SX127x_calc_bw_value(bw_khz);
	//modem config 1
	regval = bw_value | (cr << 1);
	
	SX127x_write_reg(REG_LR_MODEMCONFIG1,regval);
	//modem config 2
	regval = radioconfig.sf << 4;
	SX127x_write_reg(REG_LR_MODEMCONFIG2,regval);
	//modem config 3 - set LP optimization
	regval = SX127x_read_reg(REG_LR_MODEMCONFIG3);
	if(ldropt) regval |= LR_MODEMCONFIG3_LDROPT;
	else regval &= ~LR_MODEMCONFIG3_LDROPT;
	SX127x_write_reg(REG_LR_MODEMCONFIG3,regval);
	//detect optimize
	if(radioconfig.sf == 6) 
	{
		SX127x_write_reg(REG_LR_DETOPT,0x05);
		SX127x_write_reg(REG_LR_DET_THR,0x0c);
	}
	else SX127x_write_reg(REG_LR_DETOPT,0x03);
	//HBW optimize
	if(bw_value == LR_BW_500)
	{
		if(radioconfig.freq >= 862000000)
		{
			SX127x_write_reg(REG_LR_HIGHBW_OPT1,0x02);
			SX127x_write_reg(REG_LR_HIGHBW_OPT2,0x64);
		}
		else if((radioconfig.freq >= 410000000) && (radioconfig.freq <= 525000000))
		{
			SX127x_write_reg(REG_LR_HIGHBW_OPT1,0x02);
			SX127x_write_reg(REG_LR_HIGHBW_OPT2,0x7f);
		}
	}
	return RADIO_OK;
}

int8_t SX127x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
	uint8_t regval = 0;
	
	SX127x_write_reg(REG_LR_PRE_MSB, (prelen >> 8) & 0xff);
	SX127x_write_reg(REG_LR_PRE_LSB, prelen & 0xff);
	//paylen
	SX127x_write_reg(REG_LR_PAYLEN,paylen);
	SX127x_write_reg(REG_LR_MAXPAYLEN,0xff); //not necessary
	//sync
	regval = SX127x_read_reg(REG_LR_MODEMCONFIG1);
	if(header == true) regval |= LR_IMPL_HEADER;
	else regval &= ~LR_IMPL_HEADER;
	SX127x_write_reg(REG_LR_MODEMCONFIG1,regval);
	SX127x_write_reg(REG_LR_SYNC,sync);
	//crc
	regval = SX127x_read_reg(REG_LR_MODEMCONFIG2);
	if(radioconfig.crc == true) regval |= LR_CRCON;
	else regval &= ~LR_CRCON;
	SX127x_write_reg(REG_LR_MODEMCONFIG2,regval);
	
	//invert IQ
	if(invertiq == true)
	{
		SX127x_write_reg(REG_LR_INVERTIQ1,LR_INVERTIQ1_INVERTRX | LR_INVERTIQ1_INVERTTX);
		SX127x_write_reg(REG_LR_INVERTIQ2,0x19);
	}
	else SX127x_write_reg(REG_LR_INVERTIQ2,0x1d);
	return RADIO_OK;
}

int8_t SX127x_setopmode(uint8_t mode)
{
	uint8_t regval;
	uint8_t mode127x;

	switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
    opmode = RADIO_OPMODE_SLEEP;
		mode127x = OPMODE_SLEEP;
    break;

    case RADIO_OPMODE_STBYRC:
		case RADIO_OPMODE_STBYXOSC:
    opmode = RADIO_OPMODE_STBYRC;
    mode127x = OPMODE_STBY;
    break;

    case RADIO_OPMODE_FS:
    opmode = RADIO_OPMODE_FS;
    mode127x = OPMODE_FSRX; //OPMODE_FSTX
    break;

    case RADIO_OPMODE_TX:
    opmode = RADIO_OPMODE_TX;
    mode127x = OPMODE_TX;
    break;

    case RADIO_OPMODE_RX:
    opmode = RADIO_OPMODE_RX;
    mode127x = LR_OPMODE_RXCONT;
    break;

    case RADIO_OPMODE_TXSTREAMCW:
    case RADIO_OPMODE_TXSTREAMPRE:
		return RADIO_TODO;
		
		default:
		return FEATURE_NOT_SUPPORTED;
  }
	regval = SX127x_read_reg(REG_OPMODE);
	regval &= ~OPMODE_MSK;
	regval |= mode127x;
	SX127x_write_reg(REG_OPMODE,regval);
	return RADIO_OK;
}

uint8_t SX127x_calc_bw_value(uint32_t bw_khz)
{
	if(bw_khz <= 8) return LR_BW_7P8;
	else if(bw_khz <= 11) return LR_BW_10P4;
	else if(bw_khz <= 16) return LR_BW_15P6;
	else if(bw_khz <= 21) return LR_BW_20P8;
	else if(bw_khz <= 32) return LR_BW_31P25;
	else if(bw_khz <= 42) return LR_BW_41P7;
	else if(bw_khz <= 63) return LR_BW_62P5;
	else if(bw_khz <= 125) return LR_BW_125;
	else if(bw_khz <= 250) return LR_BW_250;
	else return LR_BW_500;
}

int8_t sx127x_prepare_tx(uint8_t *buf,uint8_t len)
{
	SX127x_write_regs(REG_FIFO,buf,len);
	SX127x_set_rf_freq(radioconfig.freq,false);
	//set TX length
	SX127x_write_reg(REG_LR_PAYLEN,len);
	//set DIO0
	uint8_t regval = SX127x_read_reg(REG_DIOMAPPING1);
	regval &= ~DIOMAPPING1_DIO0_MSK;
	regval |= DIO0_TXDONE;
	SX127x_write_reg(REG_DIOMAPPING1,regval);
	return RADIO_OK;
}

int8_t SX127x_start_rx(void)
{
	uint8_t regval;
	
	SX127x_setopmode(OPMODE_STBY);
	regval = SX127x_read_reg(REG_DIOMAPPING1);
	regval &= ~DIOMAPPING1_DIO0_MSK;
	regval |= DIO0_RXDONE;
	SX127x_write_reg(REG_DIOMAPPING1,regval);
	SX127x_set_rf_freq(radioconfig.freq,true);
	//SX127x_write_reg(REG_LR_PAYLEN,255); //???
	SX127x_setopmode(LR_OPMODE_RXCONT);
	return RADIO_OK;
}

uint8_t SX127x_get_rx_len(void)
{
	return SX127x_read_reg(REG_LR_RXBYTES_NB);
}

int8_t SX127x_read_rx_buffer(uint8_t *buf,uint8_t len)
{
	SX127x_read_regs(REG_FIFO,buf,len);
	return RADIO_OK;
}

float SX127x_get_rssi_inst(void)
{
	uint32_t f = SX127x_get_rf_freq();
	if(f >= 800000000) return (-157.0 + (float)SX127x_read_reg(REG_LR_RSSI));
	else return (-164.0 + (float)SX127x_read_reg(REG_LR_RSSI));
}

float SX127x_get_rssi_pkt(void)
{
	uint32_t f = SX127x_get_rf_freq();
	if(f >= 800000000) return (-157.0 + (float)SX127x_read_reg(REG_LR_PKT_RSSI));
	else return (-164 + SX127x_read_reg(REG_LR_PKT_RSSI));
}

float SX127x_get_snr_pkt(void)
{
	return ((float)SX127x_read_reg(REG_LR_PKT_SNR)) / 4;
}

uint16_t sx127x_get_rxpkt_cnt(void)
{
	uint8_t buf[2];
	SX127x_read_regs(REG_LR_RXPKT_CNT_MSB,buf,2);
	return (((uint16_t)buf[0]) << 8) | buf[1];
}


void SX127x_irq_handler(void)
{
	uint8_t irqstatus = SX127x_read_reg(REG_LR_IRQFLAGS);
	SX127x_write_reg(REG_LR_IRQFLAGS,0xff); //clear all flags
//	if(irqstatus & LR_IRQFLAGS_CAD_DET)
//	{
//		
//	}
//	if(irqstatus & LR_IRQFLAGS_FHSS_CHANGECH)
//	{
//		
//	}
//	if(irqstatus & LR_IRQFLAGS_CAD_DONE)
//	{
//		
//	}
	if(irqstatus & LR_IRQFLAGS_TX_DONE) packet_sent = true;
//	if(irqstatus & LR_IRQFLAGS_HEADER_VALID)
//	{
//		
//	}
	if(irqstatus & LR_IRQFLAGS_CRC_ERROR) crc_error = true;
	if(irqstatus & LR_IRQFLAGS_RX_DONE) packet_received = true;
//	if(irqstatus & LR_IRQFLAGS_RX_TO)
//	{
//		
//	}
}


