#include "sx126x_proc.h"

//uint8_t opmode = 0;
//uint8_t prevopmode = 0;
//uint16_t irqflags = 0;
//uint8_t rfstatus = 0;

/*
#define OPMODE_SLEEP            0
#define OPMODE_STBYRC           1
#define OPMODE_STBYXOSC         2
#define OPMODE_FS               3
#define OPMODE_TX               4
#define OPMODE_RX               5
#define OPMODE_TXSTREAMCW       6
#define OPMODE_TXSTREAMPRE      7
*/


void SX126X_setopmode(uint8_t mode)
{
  switch(mode)
  {
    case RADIO_OPMODE_SLEEP:
    opmode = RADIO_OPMODE_SLEEP;
    SX126X_SetSleep(0,0);
    break;

    case RADIO_OPMODE_STBYRC:
    opmode = RADIO_OPMODE_STBYRC;
    SX126X_SetStandby(0);
    break;

    case RADIO_OPMODE_STBYXOSC:
    opmode = RADIO_OPMODE_STBYXOSC;
    SX126X_SetStandby(1);
    break;

    case RADIO_OPMODE_FS:
    opmode = RADIO_OPMODE_FS;
    SX126X_SetFs();
    break;

    case RADIO_OPMODE_TX:
    opmode = RADIO_OPMODE_TX;
    SX126X_SetTx(0xffffff); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
    opmode = RADIO_OPMODE_RX;
    SX126X_SetRx(0xffffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
    opmode = RADIO_OPMODE_TXSTREAMCW;
    SX126X_SetCW();
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
    opmode = RADIO_OPMODE_TXSTREAMPRE;
    SX126X_SetTxInfinitePreamble();
    break;
  }
}

//boost LNA
void SX126X_LNAboost(bool boost)
{
  if(boost) SX126X_writeReg(0x08ac,0x96);
  else SX126X_writeReg(0x08ac,0x94);
}

//calibrate IR according to RF ftequency
//The calibration frequencies are computed as follows:
//Calibration freq = CalFreq * 4 MHz where CalFreq1 < CalFreq2
//Example: 0x6B = 428 MHz.
//When CalFreq1 = CalFreq2, the image calibration is done at a single frequency.
//For frequencies between CalFreq1 and CalFreq2, the calibration coefficient is linearly
//interpolated from the values obtained during the image calibration at CalFreq1 and CalFreq2.
//For frequencies < CalFreq1, the coefficient obtained during the image calibration at CalFreq1 is used.
//For frequencies > CalFreq2, the coefficient obtained during the image calibration at CalFreq2 is used.

void SX126X_CalibrateIR(void)
{
  uint8_t f1,f2;
  uint32_t f;
  f = radioconfig.freq / 1000000UL;
  if(f > 1020) f = 1020;
  f1 = (uint8_t)(f / 4);
  f = (f * 3) / 2; //probably 1.5 times
  if(f > 1020) f = 1020;
  f2 = (uint8_t)(f / 4);
  SX126X_CalibrateImage(f1,f2);
  //restore Ctune needed
}




