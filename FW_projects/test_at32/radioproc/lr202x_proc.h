#ifndef _LR20XX_PROC_H_
#define _LR20XX_PROC_H_

#include "bsp.h"
#include "lr20xx.h"

#define LR20XX_SEPARATION_FREQ 950000000UL

void LR202x_setopmode(uint8_t mode);
void LR202x_RssiCal(uint32_t freq);

#endif
