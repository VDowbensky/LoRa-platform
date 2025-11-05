#ifndef _USERDATA_DEFS_H_
#define _USERDATA_DEFS_H_
#include "bsp.h"
#include "sx126x.h"
#include "sx128x.h"
#include "lr11xx.h"
#define SX126X_SYNTH_STEP ((double)FXO / 33554432)
	
//LR1121 RF switches (LORA1121)
//#define LR112X_RFSW_ENABLE					0x1f
//#define LR112X_RFSW_STBY						0x00
//#define LR112X_RFSW_RX							0x01 
//#define LR112X_RFSW_SUBG_TX					0x03
//#define LR112X_RFSW_SUBG_TX_HP			0x02
//#define LR112X_RFSW_HF_TX						0
//#define LR112X_RFSW_GNSS						0
//#define LR112X_RFSW_HF_RX						0x00

//LR1121 RF switches (E80)
//#define LR112X_RFSW_ENABLE					0x07
//#define LR112X_RFSW_STBY						0x00
//#define LR112X_RFSW_RX							0x01
//#define LR112X_RFSW_SUBG_TX					0x03
//#define LR112X_RFSW_SUBG_TX_HP			0x02
//#define LR112X_RFSW_HF_TX						0x00
//#define LR112X_RFSW_GNSS						0x00
//#define LR112X_RFSW_WIFI						0x00
//#define LR112X_RFSW_HF_RX						0x00
//E80 2nd version
#define LR112X_RFSW_ENABLE					0x1f
#define LR112X_RFSW_STBY						0x00
#define LR112X_RFSW_RX							0x02 
#define LR112X_RFSW_SUBG_TX					0x03
#define LR112X_RFSW_SUBG_TX_HP			0x01
#define LR112X_RFSW_HF_TX						0x00
#define LR112X_RFSW_GNSS						0x04
#define LR112X_RFSW_WIFI						0x08
#define LR112X_RFSW_HF_RX						0x00
	
	
//SX126x
#define sx126x_tcxo (radioconfig.userdata[0])
#define sx126x_tcxo_voltage (radioconfig.userdata[1])
#define lr112x_tcxo (radioconfig.userdata[0])
#define lr112x_tcxo_voltage (radioconfig.userdata[1])
#define sx126x_xtatrim (radioconfig.userdata[2])
#define sx126x_xtbtrim (radioconfig.userdata[3])

extern const uint8_t SX126X_bw[];
extern const uint16_t SX126X_bw_kHz[];
extern const uint8_t SX128X_bw[];
extern const uint16_t SX128X_bw_kHz[];
extern const uint8_t LR112X_bw[];
extern const uint16_t LR112X_bw_kHz[];
extern const uint8_t PAN_bw[];
extern const uint16_t PAN_bw_kHz[];
				
//LR1121

//SX1280



#endif
