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
    sx126x_set_sleep(NULL,SX126X_SLEEP_CFG_COLD_START); //SX126X_SLEEP_CFG_WARM_START
    break;

    case RADIO_OPMODE_STBYRC:
    opmode = RADIO_OPMODE_STBYRC;
    sx126x_set_standby(NULL,SX126X_STANDBY_CFG_RC);
    break;

    case RADIO_OPMODE_STBYXOSC:
    opmode = RADIO_OPMODE_STBYXOSC;
    sx126x_set_standby(NULL,SX126X_STANDBY_CFG_XOSC);
    break;

    case RADIO_OPMODE_FS:
    opmode = RADIO_OPMODE_FS;
    sx126x_set_fs(NULL);
    break;

    case RADIO_OPMODE_TX:
    opmode = RADIO_OPMODE_TX;
    sx126x_set_tx_with_timeout_in_rtc_step(NULL,0xffffff); //temp.
    break;

    case RADIO_OPMODE_RX:
    default:
    opmode = RADIO_OPMODE_RX;
    sx126x_set_rx_with_timeout_in_rtc_step(NULL,0xffffff); //temp.
    break;

    case RADIO_OPMODE_TXSTREAMCW:
    opmode = RADIO_OPMODE_TXSTREAMCW;
    sx126x_set_tx_cw(NULL);
    break;

    case RADIO_OPMODE_TXSTREAMPRE:
    opmode = RADIO_OPMODE_TXSTREAMPRE;
    sx126x_set_tx_infinite_preamble(NULL);
    break;
  }
}

//boost LNA
void SX126X_LNAboost(bool boost)
{
	sx126x_cfg_rx_boosted(NULL,boost);
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
  sx126x_cal_img(NULL,f1,f2);
  //restore Ctune needed
}




