#ifndef _SX127X_PROC_H_
#define _SX127X_PROC_H_

#include "bsp.h"
#include "sx1276.h"

int8_t SX127x_init(void);
int8_t SX127x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt);
int8_t SX127x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq);

void SX127X_setopmode(uint8_t mode);
void SX127X_irq_handler(void);

uint8_t SX1276GetPaSelect(uint32_t channel);
void SX1276AntSwInit(void);
void SX1276AntSwDeInit(void);
void SX1276SetAntSw(uint8_t rxTx);

extern uint8_t opmode;
extern uint8_t prevopmode;

extern uint16_t irqflags;
extern uint8_t rfstatus;

#endif
