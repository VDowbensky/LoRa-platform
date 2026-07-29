#include "usb_task.h"

static void cdc_task(void);

uint8_t tud_rx_buffer[TUD_RXBUFSIZE];
volatile uint32_t tud_rxcount = 0;

void tud_mount_cb(void) 
{
  //blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) 
{
  //blink_interval_ms = BLINK_NOT_MOUNTED;
}

static void cdc_task(void) 
{
// connected() check for DTR bit
// Most but not all terminal client set this when making connection
// if(!tud_cdc_n_connected(0)) return;
	uint8_t buf[64];
	//cnt = (tud_cdc_n_available(0)); 
	//tud_cdc_n_read(0,buf,cnt);
	if(tud_cdc_n_available(0))
	{
		uint32_t cnt = tud_cdc_n_read(0,buf,64);
		//tud_cdc_n_read_flush(0);
		for(uint32_t i = 0; i < cnt; i++) ringbuf_push(&cdc_rx_ring,buf[i]);
	}
	
	//send pending TX data
	uint16_t cnt = 0;
	while(1)
	{
		int32_t v = ringbuf_pop(&cdc_tx_ring);
		if(v == -1) break;
		tud_cdc_n_write(0,&v,1);
		cnt++;
	}
	if(cnt != 0) tud_cdc_n_write_flush(0);
	return;
}

void usb_task(void)
{
	tud_task();
	cdc_task();
}

void cdc_print(uint8_t itf,char* str)
{
	while(*str != 0)
	{
		tud_cdc_n_write_char(itf,*str);
		str++;
	}
	tud_cdc_n_write_flush(itf);
}
