#ifndef _FLASH_H_
#define _FLASH_H_

#include "bsp.h"

#include <stdint.h>

#define FLASH_BASE_ADDR   0x08000000UL

//#define FLASH_ACR         (*(volatile uint32_t*)0x40022000)
//#define FLASH_KEYR        (*(volatile uint32_t*)0x40022008)
//#define FLASH_SR          (*(volatile uint32_t*)0x40022010)
//#define FLASH_CR          (*(volatile uint32_t*)0x40022014)
//#define FLASH_ECCR        (*(volatile uint32_t*)0x40022018)

/* FLASH_CR bits */
//#define FLASH_CR_PG       (1U << 0)
//#define FLASH_CR_PER      (1U << 1)
//#define FLASH_CR_MER1     (1U << 2)
//#define FLASH_CR_STRT     (1U << 16)
//#define FLASH_CR_LOCK     (1U << 31)

///* FLASH_SR bits */
//#define FLASH_SR_BSY      (1U << 16)
//#define FLASH_SR_EOP      (1U << 0)

/* Unlock keys */
#define FLASH_KEY1        0x45670123U
#define FLASH_KEY2        0xCDEF89ABU

#define CONFIG_ADDR			0x0801f800 //last page
#define CONFIG_SIZE			0x0800 //2KB

#define MAGIC_NUMBER		0x57575757

//#define FLASH_KEY1			0x45670123
//#define FLASH_KEY2			0xcdef89ab

void readconfig(void);
int32_t writeconfig(void);


#endif
