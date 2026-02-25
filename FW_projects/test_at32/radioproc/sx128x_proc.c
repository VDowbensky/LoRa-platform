#include "SX128X_proc.h"
#include "sx128x.h"
#include "radio_proc.h"

void SX128X_setopmode(uint8_t mode)
{
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
    opmode = RADIO_OPMODE_SLEEP;
    SX128X_SetSleep(false);
    break;

    case RADIO_OPMODE_STBYRC:
    opmode = RADIO_OPMODE_STBYRC;
    SX128X_SetStandby(0);
    break;

    case RADIO_OPMODE_STBYXOSC:
    opmode = RADIO_OPMODE_STBYXOSC;
    SX128X_SetStandby(1);
    break;

    case RADIO_OPMODE_FS:
    opmode = RADIO_OPMODE_FS;
    SX128X_SetFs();
    break;

    case RADIO_OPMODE_TX:
    opmode = RADIO_OPMODE_TX;
    SX128X_SetTx(0,0); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
    opmode = RADIO_OPMODE_RX;
    SX128X_SetRx(0xff,0xffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
    opmode = RADIO_OPMODE_TXSTREAMCW;
    SX128X_SetCW();
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
    opmode = RADIO_OPMODE_TXSTREAMPRE;
    SX128X_SetPRE();
    break;
  }
}
