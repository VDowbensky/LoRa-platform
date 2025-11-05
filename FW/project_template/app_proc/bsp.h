#ifndef _BSP_H_
#define _BSP_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "wk_system.h"
#include "at32f413_wk_config.h"
#include "at32f413_int.h"
#include "usb_app.h"

#define HW_VERSION				1
#define FW_VERSION				0
#define FW_REVISION				1

#define CPU_CLOCK_HZ			96000000UL

void radio_power_on(void);
void radio_power_off(void);
void txled_on(void);
void txled_off(void);
void rxled_on(void);
void rxled_off(void);
void bsp_reset_proc(void);
uint64_t bsp_get_uid(void);

#endif
