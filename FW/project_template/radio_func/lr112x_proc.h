#ifndef _LR112X_CONFIG_H_
#define _LR112X_CONFIG_H_

#include "bsp.h"

#define LR112X_SEPARATION_FREQ 950000000UL

void LR112X_setopmode(uint8_t mode);
void LR112X_RssiCal(uint32_t freq);
void LR112X_printstatus(void);
void LR112X_printerrors(void);

#endif
