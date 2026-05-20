#ifndef _SX127X_INTERFACE_H_
#define _SX127X_INTERFACE_H_

#include "bsp.h"

void Sx1276SetNSS(uint8_t nss);
uint8_t Sx1276SpiInOut(uint8_t b);

#endif
