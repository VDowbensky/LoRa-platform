#ifndef _RADIO_FUNC_H_
#define _RADIO_FUNC_H_

#include "bsp.h"
#include "userdata_defs.h"

#include "sx126x_proc.h"
#include "sx126x_regs.h"
#include "sx126x.h"

#include "sx128x_proc.h"
#include "sx128x_regs.h"
#include "sx128x.h"

#include "lr11xx.h"
#include "lr112x_proc.h"
#include "lr202x_proc.h"

#include "pan_rf.h"
#include "pan_proc.h"

#include "lr20xx.h"

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

#define WORK_MODE_SNIFFER							0
#define WORK_MODE_SCANNER							1
#define WORK_MODE_JAMMER							2

#define TXMOD_CW											0
#define TXMOD_PRE											1
#define TXMOD_ALTERNATING							2 //0101
#define TXMOD_PN9											3

#define PKT_MESHTASTIC								0
#define PKT_MESHCORE									1
#define PKT_KISS											2
#define PKT_TESTING										3 //my own

typedef struct radioconfig
{
  uint16_t chip;
  uint32_t id;
  uint32_t freq;
  int8_t txpower;
  //modulation
  uint8_t sf; //spreading factor
  uint32_t bw; //bandwidth
  uint8_t cr; //coding rate
  uint8_t ldropt;
  //packet
  uint8_t sync;
  uint16_t prelen;
  uint8_t header; //0 - explicit,1 - implicit
  uint8_t paylen;
  uint8_t crc;
  uint8_t invertiq;
	//TCXO usage and voltage, XO trim values etc.
  uint8_t userdata[64]; //maybe different
	//jammer parameters
	uint32_t txstartfreq;
	uint32_t txstopfreq;
	uint32_t txstep;
	uint32_t txinterval;
	uint32_t txmodulation;
	//scanner parameters
	uint32_t rxstartfreq;
	uint32_t rxstopfreq;
	uint32_t rxstep;
	uint32_t rxinterval;
	float rssitr;
	uint8_t workmode;
	uint8_t pktformat;
	uint8_t reserved[2];
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
int8_t radio_get_status(uint8_t *chip_mode,uint8_t *cmd_status);

int8_t radio_setopmode(uint8_t mode);
void radio_irq_handler(void);


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

extern uint32_t currfreq;
extern uint32_t prevfreq;



#endif
