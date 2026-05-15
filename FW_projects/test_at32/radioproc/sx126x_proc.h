#ifndef _SX126X_PROC_H_
#define _SX126X_PROC_H_

#include "bsp.h"
#include "flash.h"
#include "sx126x.h"

sx126x_status_t SX126x_init(void);
sx126x_status_t SX126x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt);
sx126x_status_t SX126x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq);

void SX126X_setopmode(uint8_t mode);
void SX126X_LNAboost(bool boost);
void SX126X_CalibrateIR(void);
void SX126X_irq_handler(void);

extern uint8_t opmode;
extern uint8_t prevopmode;

extern uint16_t irqflags;
extern uint8_t rfstatus;

#endif

