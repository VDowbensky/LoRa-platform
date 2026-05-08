#ifndef _LR20XX_PROC_H_
#define _LR20XX_PROC_H_

#include "bsp.h"
#include "lr20xx.h"

#define LR20XX_SEPARATION_FREQ 950000000UL

void LR202x_setopmode(uint8_t mode);
void LR202x_RssiCal(uint32_t freq);
void LR20xx_bsp_get_front_end_calibration_cfg(const void* context, lr20xx_radio_common_front_end_calibration_value_t front_end_calibration_structures[3]);
void LR20xx_bsp_get_rx_cfg( const void* context, const uint32_t freq_in_hz, lr20xx_radio_common_rx_path_t* rx_path,lr20xx_radio_common_rx_path_boost_mode_t* boost_mode );

extern const lr20xx_radio_common_pa_cfg_t pa_config_lf;
extern const lr20xx_radio_common_pa_cfg_t pa_config_hf;
#endif
