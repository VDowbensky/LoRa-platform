#ifndef _SX128X_CONFIG_H_
#define _SX128X_CONFIG_H_

#include "bsp.h"
#include "sx128x.h"

#define SX128X_SYNTH_STEP ((double)52000000 / 262144)

void SX128X_setopmode(uint8_t mode);
void SX128X_CalcPreamble(uint32_t prelen,uint8_t *m,uint8_t *e);

#endif
