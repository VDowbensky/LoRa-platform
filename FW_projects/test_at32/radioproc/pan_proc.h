#ifndef _PAN_PROC_H_
#define _PAN_PROC_H_

#include "bsp.h"
#include "pan_rf.h"
#include "radio_func.h"

int8_t PAN3029_init(void);
int8_t PAN_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt);
int8_t PAN_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq);

void PAN_setopmode(uint8_t mode);
void PAN_setpower(int8_t power);
void PAN_irq_handler(void);

#endif
