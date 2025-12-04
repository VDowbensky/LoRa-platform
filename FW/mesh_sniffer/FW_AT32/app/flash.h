#ifndef _FLASH_H_
#define _FLASH_H_

#include "bsp.h"
#include "radio_proc.h"

#define CONFIG_BASE			0x0801F800 //last 2 KB
#define CONFIG_SIZE			1024 //halfwords

void readconfig(void);
int32_t writeconfig(void);

#endif
