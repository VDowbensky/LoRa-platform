#ifndef _SX128X_H_
#define _SX128X_H_

#include "bsp.h"
#include "sx128x_interface.h"

uint8_t SX128X_GetStatus(void);
void SX128X_SetSleep(bool retention);
void SX128X_SetStandby(uint8_t mode);
void SX128X_SetFs(void);
void SX128X_SetTx(uint8_t period_base,uint16_t period_base_cnt);
void SX128X_SetRx(uint8_t period_base,uint16_t period_base_cnt);
void SX128X_SetRxDutyCycle(uint8_t period_base,uint16_t rx_cnt,uint16_t sleep_cnt);
void SX128X_SetCad(void);
void SX128X_SetCW(void);
void SX128X_SetPRE(void);
void SX128X_SetPacketType(uint8_t pkt);
uint8_t SX128X_GetPacketType(void);
void SX128X_SetRfFrequency(uint32_t f);
void SX128X_SetTxParams(int8_t power,uint8_t ramptime);
void SX128X_SetCadParams(uint8_t cadSymbolNum);
void SX128X_SetBufferBaseAddress(uint8_t txBaseAdress,uint8_t rxBaseAdress);
void SX128X_SetModulationParams(uint8_t modParam1,uint8_t modParam2,uint8_t modParam3);
void SX128X_LORA_SetModulationParams(uint8_t sf,uint8_t bw,uint8_t cr);
void SX128X_FSK_SetModulationParams(uint8_t bitrate,uint8_t modindex,uint8_t shaping);
void SX128X_FLRC_SetModulationParams(uint8_t bitrate,uint8_t cr,uint8_t shaping);
void SX128X_SetPacketParams(uint8_t *params);
void SX128X_LORA_SetPacketParams(uint8_t PreambleLength,uint8_t HeaderType,uint8_t PayloadLength,uint8_t crc,uint8_t invertiq);
void SX128X_BLE_SetPacketParams(uint8_t ConnectionState,uint8_t CrcLength,uint8_t BleTestPayload,uint8_t Whitening);
void SX128X_FSK_FLRC_SetPacketParams(uint8_t PreambleLength,uint8_t SyncWordLength,uint8_t SyncWordMatch,uint8_t HeaderType,uint8_t PayloadLength,uint8_t CrcLength,uint8_t Whitening);
void SX128X_GetRxBufferStatus(uint8_t* rxPayloadLength,uint8_t* rxStartBufferPointer);
void SX128X_GetPacketStatus(uint8_t *status);
void SX128X_FSK_BLE_FLRC_GetGetPacketStatus(uint8_t *rssiSync,uint8_t *errors,uint8_t *status,uint8_t *sync);
void SX128X_LORA_GetPacketStatus(uint8_t *rssiSync,int16_t *snrPkt);
uint8_t SX128X_GetRssiInst(void);
void SX128X_SetDioIrqParams(uint16_t irqMask,uint16_t  dio1Mask,uint16_t  dio2Mask,uint16_t  dio3Mask);
uint16_t SX128X_GetIrqStatus(void);
void SX128X_ClearIrqStatus(uint16_t mask);
void SX128X_Calibrate(uint8_t calibParam);
void SX128X_SetRegulatorMode(uint8_t mode);
void SX128X_SetSaveContext(void);
void SX128X_AutoTx(uint16_t time);
void SX128X_AutoFs(bool enable);
void SX128X_SetLongPreamble(bool enable);
void SX128X_SetRangingRole(uint8_t role);

//void SX128X_SetFskSyncWord(uint8_t sx,uint8_t *sw);
//void SX128X_SetFskAddr(sx,config->nodeaddr,config->braddr);
//void SX128X_SetFskCrcWhitening(uint8_t sx,uint32_t crcinit,uint32_t crcpoly,uint32_t whiteinit);



#endif
