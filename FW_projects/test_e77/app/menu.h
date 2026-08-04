#ifndef _MENU_H_
#define _MENU_H_

#include "bsp.h"
#include "micromenu.h"
//#include "rtc.h"
#include "adc.h"

#define KEY_SCAN_CNT		20 //ms

void menu_init(void);
void menu_proc(void);
void scankeys(void);
void updatescreen(void);

extern bool menu_mode;

#endif
