#include "menu.h"
#include "app_cli.h"

//volatile uint8_t menu_level = 0;

void display_main_menu(void);

void sniffer_settings(void);
void display_sniffer_menu(void);
void freq_settings(void);
void bw_settings(void);
void sf_settings(void);
void cr_settings(void);
void ldro_settings(void);
void pkt_fmt_settings(void);

void scanner_settings(void);
void display_scanner_menu(void);
void scan_start_settings(void);
void scan_stop__settings(void);
void scan_step_settings(void);
void scan_time_settings(void);
void scan_rssi_settings(void);

void jammer_settings(void);
void display_jammer_menu(void);
void jam_start_settings(void);
void jam_stop__settings(void);
void jam_step_settings(void);
void jam_time_settings(void);
void jam_mod_settings(void);

void add_settings(void);
void display_add_menu(void);
void add_own_id_settings(void);
void add_pair_id_settings(void);
void add_pkt_cnt_settings(void);
void add_interval_settings(void);
void add_power_settings(void);

char key;

void display_main_screen(void)
{
	SSD1306_Clear(0);
	display_status();
	sprintf(strbuffer,"Chip:%d",radioconfig.chip);
	GUI_ShowString(0,8,strbuffer,8,1);
	sprintf(strbuffer,"Freq:%d",radioconfig.freq/1000);
	GUI_ShowString(0,16,strbuffer,8,1);
	GUI_ShowString(0,24,"A - Send one packet",8,1);
	GUI_ShowString(0,32,"B - Send burst",8,1);
	GUI_ShowString(0,40,"C - RX scan",8,1);
	GUI_ShowString(0,48,"D - TX sweep",8,1);
	GUI_ShowString(0,56,"# - Settings",8,1);
}

void menu_proc(void)
{
	display_main_menu();
	while(1)
	{
		key = Keypad_GetKey();
		if(key == '*') 
		{
			display_main_screen();
			writeconfig();
			break;
		}
		if(key == '#') 
		{
			display_main_screen();
			break;
		}
		switch(key)
		{
			//sniffer parameters settinga
			case '1':
			sniffer_settings();
			break;
		
			case '2':
			scanner_settings();
			break;
		
			case '3':
			jammer_settings();
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
	GUI_ShowString(0,0,"1 Sniff settings",8,1);
	GUI_ShowString(0,8,"2 Scan settings",8,1);
	GUI_ShowString(0,16,"3 Jam settings",8,1);
	GUI_ShowString(0,24,"4 Test settings",8,1);
	GUI_ShowString(0,32,"* Save and exit",8,1);
	GUI_ShowString(0,40,"# Exit without saving",8,1);
}

void display_sniffer_menu(void)
{
	SSD1306_Clear(0);
	GUI_ShowString(0,0,"1 Frequency",8,1);
	GUI_ShowString(0,8,"2 BW",8,1);
	GUI_ShowString(0,16,"3 CR",8,1);
	GUI_ShowString(0,24,"4 SF",8,1);
	GUI_ShowString(0,32,"5 LDRO",8,1);
	GUI_ShowString(0,40,"6 Packet format",8,1);
}

void sniffer_settings(void)
{
	display_sniffer_menu();
	while(1)
	{
		key = Keypad_GetKey();
		if(key == '#') 
		{
			display_main_menu();
			break;
		}
		switch(key)
		{
			case '1':
			freq_settings();
			break;
			
			case '2':
			bw_settings();
			break;
			
			case '3':
			sf_settings();
			break;
			
			case '4':
			cr_settings();
			break;
			
			case '5':
			ldro_settings();
			break;
			
			case '6':
			pkt_fmt_settings();
			break;
			
			default:
			break;
		}
	}
}

void freq_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Frequency",16,1);
	NumberInput_Start(radioconfig.freq / 1000);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,30000,3000000);
	radioconfig.freq = value * 1000;
	//writeconfig();
	display_sniffer_menu();
}

void bw_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"BW",16,1);
	NumberInput_Start(radioconfig.bw);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,7,1600);
	radioconfig.bw = value;
	//writeconfig();
	display_sniffer_menu();
}

void sf_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"SF",16,1);
	NumberInput_Start(radioconfig.sf);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,5,12);
	radioconfig.sf = value;
	//writeconfig();
	display_sniffer_menu();
}

void cr_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"CR",16,1);
	NumberInput_Start(radioconfig.cr);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,0,4);
	radioconfig.cr = value;
	//writeconfig();
	display_sniffer_menu();
}

void ldro_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"LDRO",16,1);
	NumberInput_Start(radioconfig.ldropt);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,0,1);
	radioconfig.ldropt = value;
	//writeconfig();
	display_sniffer_menu();
}

void pkt_fmt_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"PACKET FORMAT",16,1);
	NumberInput_Start(radioconfig.pktformat);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,0,3); //to be changed
	radioconfig.pktformat = value;
	//writeconfig();
	display_sniffer_menu();
}

void display_scanner_menu(void)
{
	SSD1306_Clear(0);
	GUI_ShowString(0,0,"1 Start frequency",8,1);
	GUI_ShowString(0,8,"2 Stop frequency",8,1);
	GUI_ShowString(0,16,"3 Frequency step",8,1);
	GUI_ShowString(0,24,"4 Scan time",8,1);
	GUI_ShowString(0,32,"5 RSSI threshold",8,1);
}


void scanner_settings(void)
{
	display_scanner_menu();
	while(1)
	{
		key = Keypad_GetKey();
		if (key == '#') 
		{
			display_main_menu();
			break;
		}
		switch(key)
		{
			case '1':
			scan_start_settings();
			break;
			
			case '2':
			scan_stop__settings();
			break;
			
			case '3':
			scan_step_settings();
			break;
			
			case '4':
			scan_time_settings();
			break;
			
			case '5':
			scan_rssi_settings();
			break;
			
			default:
			break;
		}
	}
}

void scan_start_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Start, kHz",16,1);
	NumberInput_Start(radioconfig.rxstartfreq);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,30000,3000000);
	radioconfig.rxstartfreq = value;
  //writeconfig();
	display_scanner_menu();
}

void scan_stop__settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Stop, kHz",16,1);
	NumberInput_Start(radioconfig.rxstopfreq);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,30000,3000000);
	radioconfig.rxstopfreq = value;
	//writeconfig();
	display_scanner_menu();
}

void scan_step_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Step, kHz",16,1);
	NumberInput_Start(radioconfig.rxstep);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,1,1000000);
	radioconfig.rxstep = value;
	//writeconfig();
	display_scanner_menu();
}

void scan_time_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Scan time, ms",16,1);
	NumberInput_Start(radioconfig.rxinterval);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,1,60000);
	radioconfig.rxinterval = value;
	//writeconfig();
	display_scanner_menu();
}

void scan_rssi_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"RSSI, dBm",16,1);
	NumberInput_Start(radioconfig.rssitr);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,-127,-10);
	radioconfig.rssitr = value;
	//writeconfig();
	display_scanner_menu();
}


void display_jammer_menu(void)
{
	SSD1306_Clear(0);
	GUI_ShowString(0,0,"1 Start frequency",8,1);
	GUI_ShowString(0,8,"2 Stop frequency",8,1);
	GUI_ShowString(0,16,"3 Frequency step",8,1);
	GUI_ShowString(0,24,"4 Jam time",8,1);
	GUI_ShowString(0,32,"5 Modulation",8,1);
}

void jammer_settings(void)
{
	display_jammer_menu();
	while(1)
	{
		key = Keypad_GetKey();
		if (key == '#') 
		{
			display_main_menu();
			break;
		}
		switch(key)
		{
			case '1':
			jam_start_settings();
			break;
			
			case '2':
			jam_stop__settings();
			break;
			
			case '3':
			jam_step_settings();
			break;
			
			case '4':
			jam_time_settings();
			break;
			
			case '5':
			jam_mod_settings ();
			break;
			
			default:
			break;
		}
	}
}

void jam_start_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Start, kHz",16,1);
	NumberInput_Start(radioconfig.txstartfreq);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,30000,3000000);
	radioconfig.txstartfreq = value;
	//writeconfig();
	display_jammer_menu();
}

void jam_stop__settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Stop, kHz",16,1);
	NumberInput_Start(radioconfig.txstopfreq);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,30000,3000000);
	radioconfig.txstopfreq = value;
	//writeconfig();
	display_jammer_menu();
}

void jam_step_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Step, kHz",16,1);
	NumberInput_Start(radioconfig.txstep);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,1,1000000);
	radioconfig.txstep = value;
	//writeconfig();
	display_jammer_menu();
}

void jam_time_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Time, ms",16,1);
	NumberInput_Start(radioconfig.txinterval);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,1,60000);
	radioconfig.txinterval = value;
	//writeconfig();
	display_jammer_menu();
}

void jam_mod_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Modulation",16,1);
	NumberInput_Start(radioconfig.txmodulation);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	value = check_param(value,0,2);
	radioconfig.txmodulation = value;
	//writeconfig();
	display_jammer_menu();
}


void display_add_menu(void)
{
	SSD1306_Clear(0);
	GUI_ShowString(0,0,"1 Own ID",8,1);
	GUI_ShowString(0,8,"2 Pair ID",8,1);
	GUI_ShowString(0,16,"3 Packet count",8,1);
	GUI_ShowString(0,24,"4 Sending interval",8,1);
	GUI_ShowString(0,32,"5 TX power",8,1);
}

void add_settings(void)
{
	display_add_menu();
	while(1)
	{
		key = Keypad_GetKey();
		if (key == '#') 
		{
			display_main_menu();
			break;
		}
		switch(key)
		{
			case '1':
			add_own_id_settings();
			break;
			
			case '2':
			add_pair_id_settings();
			break;
			
			case '3':
			add_pkt_cnt_settings();
			break;
			
			case '4':
			add_interval_settings();
			break;
			
			case '5':
			add_power_settings();
			break;
			
			default:
			break;
		}
	}
}

void add_own_id_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Own ID",16,1);
	NumberInput_Start(radioconfig.id);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	if(value < 0) value = 0;
	radioconfig.id = value;
	//writeconfig();
	display_add_menu();
}

void add_pair_id_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Pair ID",16,1);
	NumberInput_Start(radioconfig.pair_id);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	if(value < 0) value = 0;
	if(value == radioconfig.id) value = radioconfig.id + 1;
	radioconfig.pair_id = value;
	//writeconfig();
	display_add_menu();
}

void add_pkt_cnt_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Packet count",16,1);
	NumberInput_Start(radioconfig.pkt_count);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	if(value < 1) value = 1;
	radioconfig.pkt_count = value;
	//writeconfig();
	display_add_menu();
}

void add_interval_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"Sending interval",16,1);
	NumberInput_Start(radioconfig.txsendinterval);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	if(value < 100) value = 100;
	radioconfig.txsendinterval = value;
	//writeconfig();
	display_add_menu();
}

void add_power_settings(void)
{
	int32_t value;
	SSD1306_Clear(0);
	GUI_ShowString(0,16,"TX power, dBm",16,1);
	NumberInput_Start(radioconfig.txpower);
	while(1)
	{
		int8_t r = NumberInput_Run(&value);
		if(r == 1) break;
		if(r == -1) break; /* canceled */ 
	}
	switch(radioconfig.chip)
	{
		case 1276:
		value = check_param(value,2,17);
		break;
		
		case 1262:
		value = check_param(value,-9,22);
		break;
		
		case 1280:
		value = check_param(value,-18,13);
		break;
		
		case 1121:
		value = check_param(value,-18,22);
		break;
		
		case 2021:
		value = check_param(value,-19,22);
		break;
		
		case 3029:
		value = check_param(value,0,20);
		break;
		
		default:
		break;
	}
	radioconfig.txpower = value;
	//writeconfig();
	display_add_menu();
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


