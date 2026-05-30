#include "menu.h"

volatile uint8_t menu_level = 0;

void display_main_menu(void);
void mode_settings(void);
void freq_settings(void);
void mod_settings(void);
void add_settings(void);

char key;

void display_main_screen(void)
{
	SSD1306_Clear(0);
	display_status();
	sprintf(strbuffer,"Chip:%d",radioconfig.chip);
	GUI_ShowString(0,16,strbuffer,16,1);
	sprintf(strbuffer,"Freq:%d",radioconfig.freq/1000);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void menu_proc(void)
{
	display_main_menu();
	while(1)
	{
		key = Keypad_GetKey();
		if(key == '#') 
		{
			display_main_screen();
			break;
		}
		switch(key)
		{
			case '1':
			mode_settings();
			break;
		
			case '2':
			freq_settings();
			break;
		
			case '3':
			mod_settings();
			break;
		
			case '4':
			add_settings();
			break;
			
			default:
			break;
		}
	}
}

void display_main_menu(void)
{
	SSD1306_Clear(0);
	GUI_ShowString(0,0,"1 - Mode",16,1);
	//sprintf(strbuffer,"Chip:%d",radioconfig.chip);
	GUI_ShowString(0,16,"2 - Frequency",16,1);
	//sprintf(strbuffer,"Freq:%d",radioconfig.freq/1000);
	GUI_ShowString(0,32,"3 - Modulation",16,1);
	GUI_ShowString(0,48,"4 - Additional",16,1);
}

void mode_settings(void)
{
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Mode: sniffer",16,1);
	//
	while(1)
	{
		if (Keypad_GetKey() == '*') 
		{
			display_main_menu();
			return;
		}
	}
}

void freq_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Frequency:",16,1);
//		if (Keypad_GetKey() == '*') 
//		{
//			display_main_menu();
//			return;
//		}
	NumberInput_Start(radioconfig.freq / 1000);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	//set frequency
	radioconfig.freq = value * 1000;
	writeconfig();
	display_main_menu();
}

void mod_settings(void)
{
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Settings:",16,1);
	//
	while(1)
	{
		if (Keypad_GetKey() == '*') 
		{
			display_main_menu();
			return;
		}
	}
}

void add_settings(void)
{
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Additional:",16,1);
	//
	while(1)
	{
		if (Keypad_GetKey() == '*') 
		{
			display_main_menu();
			return;
		}
	}
}


void display_status(void)
{
	sprintf(strbuffer,"BATT:%.2fV",Vbatt);
	GUI_ShowString(64,0,strbuffer,8,1);
}

void display_scan_rssi(uint32_t freq,float rssi)
{
	char strbuffer[64];

	GUI_ShowString(0,32,"                ",16,1);
	GUI_ShowString(0,48,"                ",16,1);
	sprintf(strbuffer,"FREQ: %d kHz",freq);
	GUI_ShowString(0,32,strbuffer,16,1);
	sprintf(strbuffer,"RSSI: %.1f dBm",rssi);
	GUI_ShowString(0,48,strbuffer,16,1);
}

void display_jam_freq(uint32_t freq)
{
	GUI_ShowString(0,32,"                ",16,1);
	sprintf(strbuffer,"FREQ: %d kHz",freq);
	GUI_ShowString(0,32,strbuffer,16,1);
}


