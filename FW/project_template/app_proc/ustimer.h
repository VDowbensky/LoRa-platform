#ifndef _USTIMER_H_
#define _USTIMER_H_

#include "bsp.h"

void ustimer_init(void);
void ustimer_start(uint8_t timer);
void ustimer_stop(uint8_t timer);
void ustimer_setinterval(uint8_t timer,uint16_t interval);

extern bool sweepflag[];

#endif
