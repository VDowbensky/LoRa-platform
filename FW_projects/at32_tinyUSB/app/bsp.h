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
#include "wk_adc.h"
#include "gui.h"
#include "test.h"
#include "wk_spi.h"

#include "radio_proc.h"
#include "keypad.h"
#include "ring_buffer.h"
#include "tusb.h"
#include "usb_task.h"

#define HW_VERSION				1
#define FW_VERSION				0
#define FW_REVISION				1

#define CPU_CLOCK_HZ			96000000UL

#define SSD1306_INTERFACE_HARD_SPI		1
#define OLED_ENABLED									1
#define DISPLAY_OFF_DELAY							120000 //2 min.

void radio_power_on(void);
void radio_power_off(void);
void txled_on(void);
void txled_off(void);
void rxled_on(void);
void rxled_off(void);
void bsp_reset_proc(void);
uint64_t bsp_get_uid(void);

void bsp_timing_irq(void);
void display_deactivate(void);
void display_activate(void);

extern volatile uint32_t adc_ticks;
extern volatile bool SecFlag;
extern volatile bool MinFlag;
extern volatile bool usb_flag;
extern volatile bool display_active;
extern volatile uint32_t display_cnt;

#endif
