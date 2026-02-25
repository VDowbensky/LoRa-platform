#include "SX128X_proc.h"
#include "sx128x.h"
#include "radio_proc.h"

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
