#ifndef _SX127X_INTERFACE_H_
#define _SX127X_INTERFACE_H_

#include "bsp.h"

void SX127x_reset(void);
void SX127x_write_regs(uint8_t reg,uint8_t *buf,uint8_t len);
void SX127x_write_reg(uint8_t reg,uint8_t val);
void SX127x_read_regs(uint8_t reg,uint8_t *buf,uint8_t len);
uint8_t SX127x_read_reg(uint8_t reg);




#endif
