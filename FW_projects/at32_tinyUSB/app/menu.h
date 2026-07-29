#ifndef _MENU_H_
#define _MENU_H_

#include "bsp.h"
#include "num_input.h"

void menu_proc(void);
void display_main_screen(void);
void display_status(void);
void display_scan_rssi(uint32_t freq,float rssi);
void display_jam_freq(uint32_t freq);

#endif
