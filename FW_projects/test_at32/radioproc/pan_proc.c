#include "pan_proc.h"

int8_t PAN3029_init(void)
{
	int8_t err = PAN_Init(radioconfig.freq);
	if(err != PAN_OK) return RADIO_INVALID_MODE; //in STB3 state
	PAN_setpower(radioconfig.txpower);
	PAN_SetFreq(radioconfig.freq);          /* Set the frequency */
	PAN_set_mod_params(radioconfig.bw,radioconfig.sf,radioconfig.cr,radioconfig.ldropt);
	PAN_set_packet_params(radioconfig.sync,radioconfig.prelen,radioconfig.paylen,radioconfig.header,radioconfig.crc,radioconfig.invertiq);
	PAN_SetRegulatorMode(USE_LDO);         /* Set the chip to LDO power mode */
	//PAN_SetChipMode(CHIPMODE_MODE0);
	//PAN_SetChipMode(CHIPMODE_MODE1);       /* Set the chip mode to MODE1 */
	//enable interrupts
	//PAN_SetPageRegBits(0,0x58,PAN_IRQ_TX_DONE | PAN_IRQ_RX_DONE | PAN_IRQ_CRC_ERR);
	PAN_SetPageRegBits(0,0x58,PAN_IRQ_TX_DONE | PAN_IRQ_RX_DONE);
	PAN_setopmode(RADIO_OPMODE_RX);
	return RADIO_OK;
}

//		if(PAN_Init(radioconfig.freq) != PAN_OK) return RADIO_INVALID_MODE; //in STB3 state
//		//PAN_SetTxPower(radioconfig.txpower);                    /* Set the power level */
//		PAN_setpower(radioconfig.txpower);
//		PAN_SetFreq(radioconfig.freq);          /* Set the frequency */
//		PAN_SetBW(PAN_bw[radioconfig.bw_index]);              /* Set the bandwidth */
//		PAN_SetSF(radioconfig.sf);              /* Set the spreading factor */
//		PAN_SetCR(radioconfig.cr);              /* Set the channel coding rate */
//		PAN_SetCRC(radioconfig.crc);            /* Set the CRC check. Disable for regular LoRa compatibility! */
//		PAN_SetLDR(radioconfig.ldropt);            /* Set the low-rate mode */
//		PAN_SetPreamLen(radioconfig.prelen);  /* Set the preamble length */
//		PAN_SetInvertIQ(radioconfig.invertiq); /* Set IQ to non-inverted */
//		PAN_SetSyncWord(radioconfig.sync & 0xff);
//		PAN_SetRegulatorMode(USE_LDO);         /* Set the chip to LDO power mode */
//		//PAN_SetChipMode(CHIPMODE_MODE0);
//		//PAN_SetChipMode(CHIPMODE_MODE1);       /* Set the chip mode to MODE1 */
//		//enable interrupts
//		PAN_SetPageRegBits(0,0x58,PAN_IRQ_TX_DONE | PAN_IRQ_RX_DONE | PAN_IRQ_CRC_ERR);
//		//PAN_SetPageRegBits(0,0x58,0xff);
//		//STB3 now
//		radio_rx();

int8_t PAN_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
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

int8_t PAN_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
	PAN_SetCRC(crc);            /* Set the CRC check. Disable for regular LoRa compatibility! */
	PAN_SetPreamLen(prelen);  /* Set the preamble length */
	PAN_SetInvertIQ(invertiq); /* Set IQ to non-inverted */
	PAN_SetSyncWord(sync);
	return RADIO_OK;
}

int8_t PAN_setopmode(uint8_t mode) 
{
//* - PAN_STATE_DEEPSLEEP
//* - PAN_STATE_SLEEP
//* - PAN_STATE_STB3
//* - PAN_STATE_TX
//* - PAN_STATE_RX
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
		opmode = RADIO_OPMODE_SLEEP;
    PAN_SetRfState(PAN_STATE_SLEEP);
    return RADIO_OK;

    case RADIO_OPMODE_STBYRC:
    return RADIO_OK;

    case RADIO_OPMODE_STBYXOSC:
		opmode = RADIO_OPMODE_STBYXOSC;
		PAN_EnterStandbyState();
    return RADIO_OK;

    case RADIO_OPMODE_TX:
		return RADIO_TODO;

    case RADIO_OPMODE_RX:
		opmode = RADIO_OPMODE_RX;
		PAN_EnterContinousRxState();
    return RADIO_OK;

    case RADIO_OPMODE_TXSTREAMCW:
		opmode = RADIO_OPMODE_TXSTREAMCW;
    PAN_StartTxContinuousWave();
    return RADIO_OK;

		default:
		return FEATURE_NOT_SUPPORTED;
  }
}

const uint8_t PAN_powerlevels[21] = {5,6,   //0,1
																		 7,7,8,   //2,3,4
																		 9,9,10,  //5,6,7
																		 11,11,12, //8,9,10
																		 13,14,15, //11,12,13
																		 17,18,19, //14,15,16
																		 20,21,22,22 //17,18,19,20
																		};

void PAN_setpower(int8_t power)
{
	PAN_SetTxPower(PAN_powerlevels[power]);
}


void PAN_irq_handler(void)
{
  if (CHECK_RF_IRQ()) /* RF interrupt detected, a high level indicates an interrupt */
  {
    uint8_t Flags;

    Flags = PAN_GetIRQFlag();    /* Get interrupt flags */
		printf("Flags:0x%02X\r\n",Flags);
    if (Flags & PAN_IRQ_TX_DONE) /* Transmit complete interrupt */
    {
      PAN_TurnoffPA();                /* Turn off PA after transmission completes */
      PAN_ClrIRQFlag(PAN_IRQ_TX_DONE); /* Clear transmit complete interrupt flag */
      Flags &= ~PAN_IRQ_TX_DONE;
			packet_sent = true;
    }
    if(Flags & PAN_IRQ_RX_DONE) /* Receive complete interrupt */
    {
      g_RfRxPkt.Snr = PAN_GetPktSnr();   /* Get the SNR value of the received packet */
      g_RfRxPkt.Rssi = PAN_GetPktRssi(); /* Get the RSSI value of the received packet */
      /* Get the received data and length */
      g_RfRxPkt.RxLen = PAN_GetRecvPayload((uint8_t *)g_RfRxPkt.RxBuf);
      PAN_ClrIRQFlag(PAN_IRQ_RX_DONE); /* Clear the receive complete interrupt flag */
      Flags &= ~PAN_IRQ_RX_DONE;
			packet_received = true;
    }
    if(Flags & PAN_IRQ_MAPM_DONE) /* MAPM receive complete interrupt */
    {
      uint8_t MapmAddr = PAN_ReadPageReg(0, 0x6E);
      g_RfRxPkt.MapmRxBuf[g_RfRxPkt.MapmRxIndex++] = MapmAddr;
      PAN_ClrIRQFlag(PAN_IRQ_MAPM_DONE); /* Clear the MAMP receive completion interrupt flag */
      Flags &= ~PAN_IRQ_MAPM_DONE;
    }
    if(Flags & PAN_IRQ_CRC_ERR) /* CRC error interrupt */
    {
      PAN_ClrIRQFlag(PAN_IRQ_CRC_ERR); /* Clear the CRC error interrupt flag */
      Flags &= ~PAN_IRQ_CRC_ERR;
			crc_error = true;
    }
    if(Flags & PAN_IRQ_RX_TIMEOUT) /* Receive timeout interrupt */
    {
      /* rf_refresh(); */
      Flags &= ~PAN_IRQ_RX_TIMEOUT;
      PAN_ClrIRQFlag(PAN_IRQ_RX_TIMEOUT); /* Clear the receive timeout interrupt flag */
    }
		//PAN_IRQ_MAPM_DONE 
		//PAN_IRQ_EFUSE_WR
    if(Flags) PAN_ClrIRQFlag(Flags); /* Clear the remaining interrupt flags */
  }
}

