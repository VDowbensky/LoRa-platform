#include "SX128X_proc.h"
#include "sx128x.h"
#include "radio_proc.h"

sx128x_status_t SX128x_init(void)
{
	sx128x_status_t err;
	err = sx128x_reset(NULL);
	if(err != SX128X_STATUS_OK) return err;
	//err = sx128x_wakeup(NULL);
	//printf("err:%d\r\n",err);
	//if(err != SX128X_STATUS_OK) return err;
	//SX128X_setopmode(SX128X_OPMODE_STBYRC);
	err = sx128x_set_reg_mode(NULL,SX128X_REG_MODE_DCDC);
	err = sx128x_set_pkt_type(NULL,SX128X_PKT_TYPE_LORA);
	if(err != SX128X_STATUS_OK) return err;
	err = SX128x_set_mod_params(radioconfig.bw,radioconfig.sf,radioconfig.cr);
	if(err != SX128X_STATUS_OK) return err;
	if(err != SX128X_STATUS_OK) return err;
	err = SX128x_set_packet_params(radioconfig.sync,radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	if(err != SX128X_STATUS_OK) return err;
	err = sx128x_set_rf_freq(NULL,radioconfig.freq);
	if(err != SX128X_STATUS_OK) return err;
	err = sx128x_set_buffer_base_address(NULL,0,0);
	if(err != SX128X_STATUS_OK) return err;
	err = sx128x_set_tx_params(NULL,radioconfig.txpower,SX128X_RAMP_02_US);
	if(err != SX128X_STATUS_OK) return err;
	
	uint8_t fix[] = SX128X_REG_RSSI_SNR_BUGFIX_BLOB;
	err = (int8_t)sx128x_write_register(NULL, SX128X_REG_RSSI_SNR_BUGFIX_ADDRESS, fix, sizeof( fix ) );
	if(err != SX128X_STATUS_OK) return err;
	err = sx128x_set_lna_settings(NULL,SX128X_LNA_HIGH_SENSITIVITY_MODE); //SX128X_LNA_LOW_POWER_MODE
	if(err != SX128X_STATUS_OK) return err;
	err = sx128x_set_dio_irq_params(NULL,SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE | SX128X_IRQ_CRC_ERROR,SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE,SX128X_IRQ_NONE,SX128X_IRQ_NONE);
	if(err != SX128X_STATUS_OK) return err;
	SX128X_setopmode(RADIO_OPMODE_RX);
	return SX128X_STATUS_OK;
}

sx128x_status_t SX128x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr)
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

sx128x_status_t SX128x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
	sx128x_pkt_params_lora_t pktparams;
	uint8_t m,e;

	SX128X_CalcPreamble(prelen,&m,&e);
	pktparams.preamble_len.mant = m;
	pktparams.preamble_len.exp = e;
	pktparams.pld_len_in_bytes = paylen;
	if(header == false) pktparams.header_type = SX128X_LORA_RANGING_PKT_EXPLICIT;
	else pktparams.header_type = SX128X_LORA_RANGING_PKT_IMPLICIT;
	pktparams.crc_is_on = crc;
	pktparams.invert_iq_is_on = invertiq;
	sx128x_status_t err = sx128x_set_lora_pkt_params(NULL,&pktparams);
	if(err != SX128X_STATUS_OK) return (int8_t)err;
	//set sync
	return (int8_t)sx128x_set_lora_sync_word(NULL,sync);
}

void SX128X_setopmode(uint8_t mode)
{
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
    opmode = RADIO_OPMODE_SLEEP;
		sx128x_set_sleep(NULL,NULL,false);
    break;

    case RADIO_OPMODE_STBYRC:
    opmode = RADIO_OPMODE_STBYRC;
    sx128x_set_standby(NULL,SX128X_STANDBY_CFG_RC);
    break;

    case RADIO_OPMODE_STBYXOSC:
    opmode = RADIO_OPMODE_STBYXOSC;
    sx128x_set_standby(NULL,SX128X_STANDBY_CFG_XOSC);
    break;

    case RADIO_OPMODE_FS:
    opmode = RADIO_OPMODE_FS;
    sx128x_set_fs(NULL);
    break;

    case RADIO_OPMODE_TX:
    opmode = RADIO_OPMODE_TX;
    sx128x_set_tx(NULL,0,0); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
    opmode = RADIO_OPMODE_RX;
    sx128x_set_rx(NULL,0xff,0xffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
    opmode = RADIO_OPMODE_TXSTREAMCW;
    sx128x_set_tx_cw(NULL);
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
    opmode = RADIO_OPMODE_TXSTREAMPRE;
    sx128x_set_tx_infinite_preamble(NULL);
    break;
  }
}

void SX128X_CalcPreamble(uint32_t prelen,uint8_t *m,uint8_t *e)
{
  uint32_t calc_preamble;
	uint8_t preamble_e = 1;
	uint8_t preamble_m = 1;
  // Cap max preamble length
  if (prelen >= 0xF000) prelen = 0xF000;
	// calculate exponent and mantissa values for modem
	while (preamble_e <= 15) 
	{
		while (preamble_m <= 15) 
		{
			calc_preamble = preamble_m * (pow(2,preamble_e));
			if (calc_preamble >= prelen - 4) break;
			preamble_m++;
		}
		if (calc_preamble >= prelen - 4) break;
		preamble_m = 1;
		preamble_e++;
	}
	*m = preamble_m;
	*e = preamble_e;
}
