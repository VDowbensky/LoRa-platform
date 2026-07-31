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
	SSD1306_Init();
	SSD1306_Clear(0);
	init_radio_specific();
	led_on();
	//radio_initconfig(1262,1);
	readconfig();
	radio_init();
	delay_ms(100);
	led_off();
	cli_init();
	GUI_ShowString(0,16,"E77 TEST PLATFORM",16,1);
	GUI_ShowString(0,32,"EXT. RADIO: NONE",16,1);
	while(1)
	{
		radio_proc();
		cli_proc();
	}
}

