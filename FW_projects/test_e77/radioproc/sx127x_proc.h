#ifndef _SX127X_PROC_H_
#define _SX127X_PROC_H_

#include "bsp.h"
#include "sx1276_regs.h"
#include "sx127x_interface.h"


int8_t SX127x_init(void);
int8_t SX127x_set_rf_freq(uint32_t freq,bool rx);
uint32_t SX127x_get_rf_freq(void);
int8_t SX127x_set_tx_params(int8_t power,uint8_t ramptime);
int8_t SX127x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt);
int8_t SX127x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq);
int8_t SX127x_setopmode(uint8_t mode);
int8_t sx127x_prepare_tx(uint8_t *buf,uint8_t len);
int8_t SX127x_start_rx(void);
float SX127x_get_rssi_inst(void);
float SX127x_get_rssi_pkt(void);
float SX127x_get_snr_pkt(void);
uint8_t SX127x_get_rx_len(void);
int8_t SX127x_read_rx_buffer(uint8_t *buf,uint8_t len);
uint16_t sx127x_get_rxpkt_cnt(void);


void SX127x_irq_handler(void);


extern uint8_t opmode;
extern uint8_t prevopmode;

extern uint16_t irqflags;
extern uint8_t rfstatus;

#endif
