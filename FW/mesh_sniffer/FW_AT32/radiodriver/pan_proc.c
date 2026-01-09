#include "pan_proc.h"

void PAN_setopmode(uint8_t mode) 
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
    PAN_SetOperateState(PAN_STATE_SLEEP);
    break;

    case RADIO_OPMODE_STBYRC:
    break;

    case RADIO_OPMODE_STBYXOSC:
		opmode = RADIO_OPMODE_STBYXOSC;
		PAN_SetOperateState(PAN_STATE_STB3);
    break;

    case RADIO_OPMODE_FS:
    break;

    case RADIO_OPMODE_TX:
		opmode = RADIO_OPMODE_SLEEP;
    PAN_SetOperateState(PAN_STATE_TX);
    break;

    case RADIO_OPMODE_RX:
		opmode = RADIO_OPMODE_RX;
    PAN_SetOperateState(PAN_STATE_RX);
    break;

    case RADIO_OPMODE_TXSTREAMCW:
		opmode = RADIO_OPMODE_TXSTREAMCW;
    PAN_StartTxContinuousWave();
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
    break;
		
		default:
		break;
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
	if(power < 0) power = 0;
	if(power > 20) power = 20;
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

