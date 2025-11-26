#ifndef _USERDATA_DEFS_H_
#define _USERDATA_DEFS_H_
#include "bsp.h"
#include "sx126x.h"

#define SX126X_SYNTH_STEP ((double)FXO / 33554432)
	
	
//SX126x
#define sx126x_tcxo (radioconfig.userdata[0])
#define sx126x_tcxo_voltage (radioconfig.userdata[1])
#define lr112x_tcxo (radioconfig.userdata[0])
#define lr112x_tcxo_voltage (radioconfig.userdata[1])
#define sx126x_xtatrim (radioconfig.userdata[2])
#define sx126x_xtbtrim (radioconfig.userdata[3])

extern const uint8_t SX126X_bw[];
extern const uint16_t SX126X_bw_kHz[];

#endif
