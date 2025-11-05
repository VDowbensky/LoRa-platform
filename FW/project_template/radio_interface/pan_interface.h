#ifndef _PAN_INTERFACE_H_
#define _PAN_INTERFACE_H_

#include "bsp.h"
#include "pan_rf.h"
#include "wk_spi.h"

#define CHECK_RF_IRQ() (gpio_input_data_bit_read(RF_INT_GPIO_PORT,RF_INT_PIN) == SET)
uint8_t __ctz(uint8_t Value);
uint8_t PAN_ReadReg(uint8_t Addr);
int8_t PAN_WriteReg(uint8_t Addr,uint8_t Value);
void PAN_ReadRegs(uint8_t Addr, uint8_t *Buffer, uint8_t Size);
void PAN_WriteRegs(uint8_t Addr,uint8_t *Buffer,uint8_t Size);
void PAN_Reset(void);
//void PAN_DelayUs(uint32_t us);
void PAN_DelayMs(uint32_t ms);

#endif
