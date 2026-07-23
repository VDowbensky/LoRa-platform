#ifndef _LR112X_PROC_H_
#define _LR112X_PROC_H_

#include "bsp.h"
#include "lr11xx.h"

#define LR112X_SEPARATION_FREQ 950000000UL

lr11xx_status_t LR112x_init(void);
lr11xx_status_t LR112X_set_freq(uint32_t Hz);
lr11xx_status_t LR112x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt);
lr11xx_status_t LR112x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq);

int8_t LR112X_setopmode(uint8_t mode);
void LR112X_RssiCal(uint32_t freq);
void LR112X_printstatus(void);
void LR112X_printerrors(void);
void LR112X_irq_handler(void);

extern const lr11xx_radio_pa_cfg_t pa_config_HF;
extern const lr11xx_radio_pa_cfg_t pa_config_subGHz;

#endif
