#ifndef _RADIO_FUNC_H_
#define _RADIO_FUNC_H_

#include "bsp.h"
#include "userdata_defs.h"

#include "sx126x_proc.h"
#include "sx126x.h"
#include "sx128x_proc.h"
#include "sx128x.h"
#include "lr11xx.h"
#include "lr112x_proc.h"
#include "pan_rf.h"
#include "pan_proc.h"

//global defines

#define RADIO_TXBUF_SIZE							256
#define RADIO_RXBUF_SIZE							256

#define RADIO_OK                      0
#define INVALID_CHIP                  1
#define RADIO_COMM_FAIL               2
#define RADIO_BUSY                    3
#define FEATURE_NOT_SUPPORTED         4
#define RADIO_INVALID_MODE            5
#define RADIO_INVALID_PARAMETER       6
//etc.
#define RADIO_TODO                    127

#define RADIO_OPMODE_SLEEP						0
#define RADIO_OPMODE_STBYRC						1
#define RADIO_OPMODE_STBYXOSC         2
#define RADIO_OPMODE_FS               3
#define RADIO_OPMODE_RX               4
#define RADIO_OPMODE_TX               5
#define RADIO_OPMODE_TXSTREAMCW       6
#define RADIO_OPMODE_TXSTREAMPRE      7

typedef struct radioconfig
{
  uint16_t chip;
  uint32_t id;
  uint32_t freq;
  int8_t txpower;
  //modulation
  uint8_t sf; //spreading factor
  uint8_t bw_index; //bandwidth
  uint8_t cr; //coding rate
  uint8_t ldropt;
  //packet
  uint16_t sync;
  uint16_t prelen;
  uint8_t header; //0 - explicit,1 - implicit
  uint8_t paylen;
  uint8_t crc;
  uint8_t invertiq;
  uint8_t userdata[64]; //maybe different
  //TCXO usage and voltage, XO trim values etc.
}radioconfig_t;

//incoming packet status
typedef struct
{
	float rssi_pkt; //LoRa
	float snr_pkt; //LoRa
	float signal_rssi_pkt; //LoRa
}rxpacketstatus_t;

//statistics
typedef struct rx_stats
{
	uint16_t pkt_received; 
	uint16_t crc_error; 
	uint16_t header_error; 
	uint16_t false_sync;
}rxstats_t;

//global functions
int8_t radio_initconfig(uint16_t chip,uint8_t tcxo);
int8_t radio_init(void);
int8_t radio_set_freq(uint32_t khz);
int8_t radio_set_power(int8_t dbm);
int8_t radio_setmodparams(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt);
int8_t radio_getmodparams(uint16_t *bw_khz,uint8_t *sf,uint8_t *cr,uint8_t *ldropt);
int8_t radio_setpktparams(uint16_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq);
int8_t radio_sendpacket(uint8_t *buf);
int8_t radio_getpktstatus(rxpacketstatus_t *status);
int8_t radio_getpacket(uint8_t *buf);
//helpers
int8_t radio_getstats(rxstats_t *stats);
int8_t radio_clearstats(void);
//irq handler
void radio_irq_handler(void);
//calibrations

//working modes
int8_t radio_rx(void);
int8_t radio_getrssi(float *dbm);
int8_t radio_stream(uint8_t stream);
int8_t radio_sleep(uint8_t mode);
int8_t radio_wakeup(uint8_t mode);

int8_t radio_setxotrim(uint8_t trim);
int8_t radio_getxotrim(uint8_t *trim);

int8_t radio_readreg(uint32_t reg,uint32_t *val);
int8_t radio_writereg(uint32_t reg,uint32_t val);

int8_t radio_get_chip_version(uint8_t *hw,uint8_t *use_case,uint8_t *fw_major,uint8_t *fw_minor);
int8_t radio_get_status(uint8_t *stat1,uint8_t *stat2);

uint8_t radio_setopmode(uint8_t mode);


//global variables
//buffers
extern uint8_t radio_txbuffer[RADIO_TXBUF_SIZE];
extern uint8_t txlen;
extern uint8_t radio_rxbuffer[RADIO_RXBUF_SIZE];
extern uint8_t rxlen;
//radio configuration structure
extern radioconfig_t radioconfig;
extern rxpacketstatus_t pktstatus;

extern bool packet_sent;
extern bool packet_received;
extern bool crc_error; 



#endif
