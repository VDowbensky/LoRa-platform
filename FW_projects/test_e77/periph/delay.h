#ifndef _DELAY_H_
#define _DELAY_H_

#include "bsp.h"

void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint32_t nms);

#endif
