#ifndef _LR112X_INTERFACE_H_
#define _LR112X_INTERFACE_H_

#include "bsp.h"
#include "lr11xx.h"
#include "wk_spi.h"

#define LR112x_NOP    0
#define LR112X_BUSY_TIMEOUT 1000000UL

void LR112X_reset(void);
void LR112X_Wakeup(void);
bool LR112X_checkBusy(void);
void LR112X_select(void);
void LR112X_deselect(void);
void LR112X_writeCmd(uint16_t cmd, uint8_t *buffer, uint16_t size);
void LR112X_readCmd(uint16_t cmd, uint8_t *args, uint8_t argslen, uint8_t *buffer, uint16_t size);
void LR112X_WriteRegMem32(uint32_t addr, uint8_t *buffer, uint16_t size);
void LR112X_ReadRegMem32(uint32_t addr, uint8_t *buffer, uint16_t size);
void LR112X_writeRegMemMask32(uint32_t addr, uint32_t mask, uint32_t regdata);
void LR112X_writeBuffer8(uint8_t *data, uint8_t leng);
void LR112X_readBuffer8(uint8_t offset, uint8_t *data, uint8_t len);
uint8_t LR112X_spi_transfer(uint8_t b);

void LR112X_rfsw_tx(void);
void LR112X_rfsw_rx(void);

#endif
