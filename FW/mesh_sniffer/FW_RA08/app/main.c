#include "bsp.h"
#include "radio_proc.h"
#include "app_cli.h"

int main(void)
{
	init_power_clk();
	init_peripherals();
	delay_ms(100);
	printf("\r\nMeshtastic sniffer\r\n");
	printf("HW=%d,FW=%d.%d\r\n",HW_VERSION,FW_VERSION,FW_REVISION);
	led_on();
	//radio_initconfig(1121,0);
	readconfig();
	radio_init();
	delay_ms(100);
	led_off();
	cli_init();
	
	while(1)
	{
		radio_proc();
		cli_proc();
	}
}

