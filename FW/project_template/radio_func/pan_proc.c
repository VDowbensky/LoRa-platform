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




