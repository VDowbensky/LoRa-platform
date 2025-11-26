#include "flash.h"
#include "radio_func.h"

static void flash_unlock(void);
static void flash_lock(void);
static void flash_wait_ready(void);
static void flash_clear_flags(void);
int32_t flash_erase_page(uint32_t addr);
int32_t flash_program_bytes(uint32_t flash_addr,const uint8_t *data,uint32_t size);

//uint8_t flashbuffer[CONFIG_SIZE];

void readconfig(void)
{
	memcpy((void *)&radioconfig,(uint8_t *)(CONFIG_ADDR),sizeof(radioconfig));
}

int32_t writeconfig(void)
{
	//memcpy((void *)(&flashbuffer[0]),(uint8_t*)&radioconfig,sizeof(radioconfig));
	flash_erase_page(CONFIG_ADDR);
	flash_program_bytes(CONFIG_ADDR, (uint8_t*)&radioconfig, CONFIG_SIZE);
	return RADIO_OK;
}

static void flash_unlock(void)
{
	if (FLASH->CR & FLASH_CR_LOCK)
	{
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}
}

static void flash_lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}


/* ----------------------------------------------- */
/* Wait until Flash is not busy */
/* ----------------------------------------------- */
static void flash_wait_ready(void)
{
	while (FLASH->SR & FLASH_SR_BSY);
}


/* ----------------------------------------------- */
/* Clear EOP + error flags */
/* ----------------------------------------------- */
static void flash_clear_flags(void)
{
	FLASH->SR |= FLASH_SR_EOP;         // writing 1 clears flag
	FLASH->ECCR |= (1U << 24);         // clear ECC errors
}


/* ----------------------------------------------- */
/* Erase ONE Flash page */
/* ----------------------------------------------- */
/* addr must be a valid page start address */
int32_t flash_erase_page(uint32_t addr)
{
	uint32_t pnb = (addr & 0x07ffffff) >> 11;
	
	flash_unlock();
	flash_wait_ready();
	flash_clear_flags();
	FLASH->CR = FLASH_CR_PER;      // Page erase
	//FLASH->CR |= addr;             // Set page address
	FLASH->CR |= pnb << FLASH_CR_PNB_Pos; // Set page address
	FLASH->CR |= FLASH_CR_STRT;    // Start erase
	flash_wait_ready();
	FLASH->CR &= ~FLASH_CR_PER;    // Disable erase mode
	flash_lock();
	return 0;
}

//#define FLASH_CR_PNB_Pos                    (3U)
//#define FLASH_CR_PNB_Msk                    (0x7FUL << FLASH_CR_PNB_Pos)       /*!< 0x000003F8 */
//#define FLASH_CR_PNB                        FLASH_CR_PNB_Msk                   /*!< Page number selection mask                          */

/* ----------------------------------------------- */
/* Program bytes into flash (programming unit = 64 bits) */
/* Data MUST be <= 1 page and aligned to flash rules */
/* ----------------------------------------------- */
int32_t flash_program_bytes(uint32_t flash_addr,const uint8_t *data,uint32_t size)
{
	flash_unlock();
	flash_wait_ready();
	flash_clear_flags();
	while (size > 0)
	{
		uint64_t dword = 0;
		/* Assemble 64-bit block */
		for (int i = 0; i < 8; i++)
		{
			uint8_t b = (size > 0) ? *data++ : 0xFF;  // pad with 0xFF
			dword |= ((uint64_t)b << (8 * i));
			if (size > 0) size--;
		}
		/* Enable programming */
		FLASH->CR |= FLASH_CR_PG;
		/* Write double-word */
		*(volatile uint32_t*)(flash_addr)       = (uint32_t)(dword & 0xFFFFFFFF);
		*(volatile uint32_t*)(flash_addr + 4)   = (uint32_t)(dword >> 32);
		flash_wait_ready();
		FLASH->CR &= ~FLASH_CR_PG;
		flash_addr += 8;
	}
	flash_lock();
	return 0;
}

