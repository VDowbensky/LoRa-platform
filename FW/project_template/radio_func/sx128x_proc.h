#ifndef _SX128X_CONFIG_H_
#define _SX128X_CONFIG_H_

#include "bsp.h"

#define SX128X_SYNTH_STEP ((double)52000000 / 262144)

void SX128X_setopmode(uint8_t mode);

#endif
