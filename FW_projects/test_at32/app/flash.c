#include "flash.h"


uint16_t config_buffer[CONFIG_SIZE];

void readconfig(void)
{
	memcpy((void *)&radioconfig,(uint8_t*)(CONFIG_BASE),sizeof(radioconfig));
}

int32_t writeconfig(void)
{
	uint16_t i;
	flash_status_type status = FLASH_OPERATE_DONE; 
	uint32_t write_addr = CONFIG_BASE;
	memcpy((void *)(&config_buffer[0]),(uint8_t*)&radioconfig,sizeof(radioconfig));
	flash_unlock();
	flash_sector_erase(CONFIG_BASE);
  for(i = 0; i < CONFIG_SIZE; i++)
  {
    status = flash_halfword_program(write_addr, config_buffer[i]);
    if(status != FLASH_OPERATE_DONE) 
		{
			flash_lock();
			return -1;
		}
    write_addr += 2;
  }
	flash_lock();
	return 0;
}

