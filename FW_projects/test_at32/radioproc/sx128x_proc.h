#ifndef _SX128X_CONFIG_H_
#define _SX128X_CONFIG_H_

#include "bsp.h"
#include "sx128x.h"

#define SX128X_SYNTH_STEP ((double)52000000 / 262144)
	
sx128x_status_t SX128x_init(void);
sx128x_status_t SX128x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr);
sx128x_status_t SX128x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq);

void SX128X_setopmode(uint8_t mode);
void SX128X_CalcPreamble(uint32_t prelen,uint8_t *m,uint8_t *e);

#endif
