#ifndef _SX126X_PROC_H_
#define _SX126X_PROC_H_

#include "bsp.h"
#include "flash.h"
#include "sx126x.h"
#include "radio_func.h"

void SX126X_setopmode(uint8_t mode);
void SX126X_LNAboost(bool boost);

void SX126X_CalibrateIR(void);
void SX126X_irq_proc(void);

extern uint8_t opmode;
extern uint8_t prevopmode;

extern uint16_t irqflags;
extern uint8_t rfstatus;

#endif

