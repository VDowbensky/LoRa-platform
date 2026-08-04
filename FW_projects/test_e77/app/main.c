#include "bsp.h"
#include "radio_proc.h"
#include "app_cli.h"
#include "menu.h"

int main(void)
{
	init_power_clk();
	init_peripherals();
	delay_ms(100);
	printf("\r\nMeshtastic sniffer\r\n");
	printf("HW=%d,FW=%d.%d\r\n",HW_VERSION,FW_VERSION,FW_REVISION);
	init_radio_specific();
	led_on();
	//radio_initconfig(1262,1);
	readconfig();
	radio_init();
	delay_ms(100);
	led_off();
	cli_init();
	SSD1306_Init();
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"E77 TEST ",16,1);
	GUI_ShowString(0,32,"EXT.RADIO:NONE",16,1);
	while(1)
	{
		//handle_keys();
		radio_proc();
		cli_proc();
		if (Key == K_ENTER)
		{
			Key = K_NONE;
			//radio_stopscan();
			menu_proc();
		}
	}
}

