#include "menu.h"
#include "gui.h"
//#include "beeper.h"
#include "radio.h"
#include "flash.h"
#include "app_cli.h"

#define BLANK_STRING_16		("                ")

void Generic_Write(const char* Text);
void cls(void);
void clear_line16(uint8_t pos);

void menu_display(Menu_Item_t* menu,uint8_t mode);
void menu_exit(void);




void menu_proc(void)
{
	uint8_t key_tmp;
	
	//create menu
	menu_init();
	
	while(menu_mode)
	{
	if (Key != K_NONE)
		{
			key_tmp = Key;
			Key = K_NONE;
			switch(key_tmp)
			{
				case K_LEFT:
				if(CurrentMenuItem->Previous != &NULL_MENU)
				{
					//beep(3000,50);
					menu_display(CurrentMenuItem,1);
					Menu_Navigate(MENU_PREVIOUS);
				}
				break;
			
				case K_RIGHT:
				if(CurrentMenuItem->Next != &NULL_MENU)
				{				
					//beep(3000,50);
					menu_display(CurrentMenuItem,1);
					Menu_Navigate(MENU_NEXT);
				}
				break;
			
				case K_ENTER:
				//beep(3000,100);
				Menu_EnterCurrentItem();
				break;
			
				default:
				break;
			}
		} 
		cli_proc();
	}
}

// Menus function
void display_main_menu(void);
void send_packet(void);
void send_burst(void);
void scan_mode(void);
void jam_mode(void);
void menu_pkt(void);
void settings_enter(void);


	
bool menu_mode = false;
uint8_t temp_key = K_NONE;
	
// Menus  Name 							| Next 							| Prev 									| Parent 						| Child 				| SelectFunction | EnterFunction 						| Text
MENU_ITEM(m_sendpacket,		 	m_sendburst,		 		NULL_MENU,							NULL_MENU, 					NULL_MENU,			NULL,							send_packet,						"\1 Send 1 packet  ");
MENU_ITEM(m_sendburst,			m_scan, 						m_sendpacket,						NULL_MENU, 					NULL_MENU,			NULL, 						send_burst,							"\2 Send burst		 ");
MENU_ITEM(m_scan,				 		m_jam, 							m_sendburst,  					NULL_MENU, 					NULL_MENU, 			NULL, 						scan_mode,							"\3 Scanning mode  ");
MENU_ITEM(m_jam,				 		m_settings,					m_scan,  								NULL_MENU, 					NULL_MENU, 			NULL, 						jam_mode,								"\4 Jamming mode   ");
MENU_ITEM(m_settings,		 		m_exit,							m_jam,  								NULL_MENU, 					m_pktsettings,	NULL, 						settings_enter,					"\5 Settings       ");
MENU_ITEM(m_exit,				 		NULL_MENU,					m_settings,  						NULL_MENU, 					NULL_MENU, 			NULL, 						menu_pkt, 							"\6 Return to pkt mode");

void menu_init(void)
{
	Menu_SetGenericWriteCallback(Generic_Write);
	display_main_menu();
	Menu_Navigate(&m_sendpacket);
	menu_mode = true;
}

void display_main_menu(void)
{
	cls();
	menu_display(&m_sendpacket,0);
	menu_display(&m_sendburst,1);
	menu_display(&m_scan,1);
	menu_display(&m_jam,1);
	menu_display(&m_settings,1);
	menu_display(&m_exit,1);
}

void menu_exit(void)
{
	//cls();
	menu_mode = false;
}

//send one packet
void send_packet(void)
{
	//send packet and return
	cls();
	GUI_ShowString(0,0,"SEND 1 PACKET   ",16,1);
	tx_request = true;
	menu_exit();
}

void send_burst(void)
{
	//set burst mode and return
	txpacketcount = radioconfig.pkt_count;
	inter_packet_delay = radioconfig.txsendinterval;
	cls();
	GUI_ShowString(0,0,"SEND BURST      ",16,1);
	radio_startburst();
	menu_exit();
}

void scan_mode(void)
{
	//set scan mode and return
	int8_t err = radio_rxscan(radioconfig.rxstartfreq,radioconfig.rxstopfreq,radioconfig.rxstep,radioconfig.rxinterval,radioconfig.rssitr);
	if(err == RADIO_OK)
	{
		led_on();
		workmode = WORK_MODE_SCANNER;
		cls();
		GUI_ShowString(0,0,"SCANNER MODE    ",16,1);
	}
	menu_exit();
}

void jam_mode(void)
{
	//set jam mode and return
	int8_t err = radio_txsweep(radioconfig.txstartfreq,radioconfig.txstopfreq,radioconfig.txstep,radioconfig.txinterval,radioconfig.txmodulation);
	if(err == RADIO_OK)
	{
		led_on();
		workmode = WORK_MODE_JAMMER;
		cls();
		GUI_ShowString(0,0,"JAMMER MODE     ",16,1);
	}
	menu_exit();
}

void menu_pkt(void)
{
	led_off();
	workmode = WORK_MODE_PACKET;
	cls();
	GUI_ShowString(0,0,"PACKET MODE     ",16,1);
	menu_exit();
}

//Settings
void settings_enter(void);
void display_settings(void);
void settings_pkt(void);
void settings_scan(void);
void settings_jam(void);
void settings_other(void);
void settings_save(void);
void save_and_reset(void);
void settings_exit(void);
// Menus  Name 							| Next 							| Prev 									| Parent 						| Child 				| SelectFunction | EnterFunction 						| Text
MENU_ITEM(m_pktsettings,	 	m_scansettings,			NULL_MENU,							NULL_MENU,					m_pktfreq,			NULL,							settings_pkt,						"\1 Packet settings");
MENU_ITEM(m_scansettings,	 	m_jamsettings, 			m_pktsettings,					NULL_MENU,					m_scanstart,		NULL,							settings_scan,					"\2 Scanner settings");
MENU_ITEM(m_jamsettings,	 	m_othersettings,		m_scansettings,					NULL_MENU,					m_jamstart,			NULL,							settings_jam, 					"\3 Jammer settings");
MENU_ITEM(m_othersettings, 	m_save_settings,		m_jamsettings,					NULL_MENU,					m_own_id,				NULL,							settings_other,					"\4 Other settings ");
MENU_ITEM(m_save_settings,	m_save_reset,				m_othersettings,				m_sendpacket,				NULL_MENU, 			NULL, 						settings_save,					"\5 Save settings  ");
MENU_ITEM(m_save_reset,			m_exit_settings,		m_save_settings,				NULL_MENU,					NULL_MENU, 			NULL, 						save_and_reset,					"\6 Save and reset");
MENU_ITEM(m_exit_settings,	NULL_MENU,					m_save_reset,						m_sendpacket,				NULL_MENU, 			NULL, 						settings_exit,					"\7 Exit w/o saving");

void settings_enter(void)
{
	Menu_Navigate(MENU_CHILD);
	display_settings();
}

void display_settings(void)
{
	cls();
	menu_display(&m_pktsettings,0);
	menu_display(&m_scansettings,1);
	menu_display(&m_jamsettings,1);
	menu_display(&m_othersettings,1);
	menu_display(&m_save_settings,1);
	menu_display(&m_save_reset,1);
	menu_display(&m_exit_settings,1);
}

//Packet settings
//Frequency, BW,SF,LDRO,pkt.format,sync
void display_pkt_settings(void);
void pkt_freq_set(void);
void pkt_bw_set(void);
void pkt_sf_set(void);
void pkt_cr_set(void);
void pkt_ldro_set(void);
void pkt_fmt_set(void);
void pkt_sync_set(void);
void pkt_exit(void);

void display_pkt_freq(uint32_t freq);
void display_pkt_bw(uint32_t bw);
void display_pkt_sf(uint8_t sf);
void display_pkt_cr(uint8_t cr);
void display_pkt_ldro(uint8_t ldro);
void display_pkt_fmt(uint8_t fmt);
void display_pkt_sync(uint8_t sync);

// Menus  Name 							| Next 							| Prev 									| Parent 						| Child 				| SelectFunction | EnterFunction 						| Text
MENU_ITEM(m_pktfreq,	 			m_pktbw,						NULL_MENU,							m_pktfreq,					NULL_MENU,			NULL,							pkt_freq_set,						"\1 Frequency");
MENU_ITEM(m_pktbw,				 	m_pktsf,			 			m_pktfreq,							m_pktfreq,					NULL_MENU,			NULL,							pkt_bw_set,							"\2 BW");
MENU_ITEM(m_pktsf,	 				m_pktcr,						m_pktbw,								m_pktfreq,					NULL_MENU,			NULL,							pkt_sf_set,		 					"\3 SF");
MENU_ITEM(m_pktcr,	 				m_pktldro,					m_pktsf,								m_pktfreq,					NULL_MENU,			NULL,							pkt_cr_set,		 					"\4 CR");
MENU_ITEM(m_pktldro, 				m_pktformat,				m_pktcr,								m_pktfreq,					NULL_MENU,			NULL,							pkt_ldro_set,						"\5 LDRO ");
MENU_ITEM(m_pktformat,			m_pktsync,					m_pktldro,  						m_pktfreq,					NULL_MENU, 			NULL, 						pkt_fmt_set,						"\6 Packet format ");
MENU_ITEM(m_pktsync,				m_pktexit,					m_pktformat, 						m_pktfreq,					NULL_MENU, 			NULL, 						pkt_sync_set,						"\7 Sync word       ");
MENU_ITEM(m_pktexit,				NULL_MENU,					m_pktsync,							m_pktsettings,			NULL_MENU, 			NULL, 						pkt_exit,								"\b Exit            ");

void settings_pkt(void)
{
	Menu_Navigate(MENU_CHILD);
	display_pkt_settings();
}

void display_pkt_settings(void)
{
	cls();
	menu_display(&m_pktfreq,0);
	menu_display(&m_pktbw,1);
	menu_display(&m_pktsf,1);
	menu_display(&m_pktcr,1);
	menu_display(&m_pktldro,1);
	menu_display(&m_pktformat,1);
	menu_display(&m_pktsync,1);
	menu_display(&m_pktexit,1);
}

void pkt_freq_set(void)
{
	uint32_t freq = radioconfig.freq/1000;
	cls();
	display_pkt_freq(freq);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			freq += 25; //temporary
			display_pkt_freq(freq);
		}
		else if(temp_key == K_LEFT)
		{
			freq -= 25;
			display_pkt_freq(freq);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.freq = freq * 1000;
	Menu_Navigate(MENU_PARENT);
	display_pkt_settings();
}

void display_pkt_freq(uint32_t freq)
{
	clear_line16(32);
	sprintf(strbuffer,"FREQ:%dkHz",freq);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void pkt_bw_set(void)
{
	int8_t bw_index;
	uint16_t bw_val[10]= {7,10,15,20,31,41,62,125,250,500};
	uint32_t bw_khz = radioconfig.bw;
	if(bw_khz <= 8) bw_index = 0;
	else if(bw_khz <= 11) bw_index = 1;
	else if(bw_khz <= 16) bw_index = 2;
	else if(bw_khz <= 21) bw_index = 3;
	else if(bw_khz <= 32) bw_index = 4;
	else if(bw_khz <= 42) bw_index = 5;
	else if(bw_khz <= 63) bw_index = 6;
	else if(bw_khz <= 125) bw_index = 7;
	else if(bw_khz <= 250) bw_index = 8;
	else bw_index = 9;
	cls();
	display_pkt_bw(bw_khz);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			bw_index++;
			if(bw_index > 9) bw_index = 9;
			display_pkt_bw(bw_val[bw_index]);
		}
		else if(temp_key == K_LEFT)
		{
			bw_index--;
			if(bw_index < 0) bw_index = 0;
			display_pkt_bw(bw_val[bw_index]);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.bw = bw_val[bw_index];
	Menu_Navigate(MENU_PARENT);
	display_pkt_settings();
}

void display_pkt_bw(uint32_t bw)
{
	clear_line16(32);
	sprintf(strbuffer,"BW:%dkHz",bw);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void pkt_sf_set(void)
{
	uint8_t sf = radioconfig.sf;
	cls();
	display_pkt_sf(sf);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			sf++;
			if(sf > 12) sf = 12;
			display_pkt_sf(sf);
		}
		else if(temp_key == K_LEFT)
		{
			sf--;
			if(sf < 6) sf = 6;
			display_pkt_sf(sf);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.sf = sf;
	Menu_Navigate(MENU_PARENT);
	display_pkt_settings();
}

void display_pkt_sf(uint8_t sf)
{
	clear_line16(32);
	sprintf(strbuffer,"SF:%d",sf);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void pkt_cr_set(void)
{
	int8_t cr = radioconfig.cr;
	cls();
	display_pkt_cr(cr);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			cr++;
			if(cr > 4) cr = 4;
			display_pkt_cr(cr);
		}
		else if(temp_key == K_LEFT)
		{
			cr--;
			if(cr < 0) cr = 0;
			display_pkt_cr(cr);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.cr = cr;
	Menu_Navigate(MENU_PARENT);
	display_pkt_settings();
}

void display_pkt_cr(uint8_t cr)
{
	clear_line16(32);
	switch(cr)
	{
		case 0:
		GUI_ShowString(0,32,"CR: OFF",16,1);
		break;
		
		case 1:
		GUI_ShowString(0,32,"CR: 4/5",16,1);
		break;
		
		case 2:
		GUI_ShowString(0,32,"CR: 4/6",16,1);
		break;
		
		case 3:
		GUI_ShowString(0,32,"CR: 4/7",16,1);
		break;
		
		case 4:
		GUI_ShowString(0,32,"CR: 4/8",16,1);
		break;
		
		default:
		GUI_ShowString(0,32,"CR: INVALID",16,1);
		break;
	}
}

void pkt_ldro_set(void)
{
	int8_t ldro = radioconfig.ldropt;
	cls();
	display_pkt_ldro(ldro);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			ldro++; 
			if(ldro > 1) ldro = 1;
			display_pkt_ldro(ldro);
		}
		else if(temp_key == K_LEFT)
		{
			ldro--;
			if(ldro < 0) ldro = 0;
			display_pkt_ldro(ldro);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.ldropt = ldro;
	Menu_Navigate(MENU_PARENT);
	display_pkt_settings();
}

void display_pkt_ldro(uint8_t ldro)
{
	clear_line16(32);
	if(ldro == 0) GUI_ShowString(0,32,"LDROPT: OFF",16,1);
	else GUI_ShowString(0,32,"LDROPT: ON",16,1);
}

void pkt_fmt_set(void)
{
	int8_t fmt = radioconfig.pktformat;
	cls();
	display_pkt_fmt(fmt);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			fmt++; //temporary
			if(fmt > 3) fmt = 3;
			display_pkt_fmt(fmt);
		}
		else if(temp_key == K_LEFT)
		{
			fmt--;
			if(fmt < 0) fmt = 0;
			display_pkt_fmt(fmt);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.pktformat = fmt;
	Menu_Navigate(MENU_PARENT);
	display_pkt_settings();
}

void display_pkt_fmt(uint8_t fmt)
{
	clear_line16(32);
	switch(fmt)
	{
		case 0:
		GUI_ShowString(0,32,"MESHTASTIC",16,1);
		break;
		
		case 1:
		GUI_ShowString(0,32,"MESHCORE",16,1);
		break;
		
		case 2:
		GUI_ShowString(0,32,"RETICULUM",16,1);
		break;
		
		case 3:
		GUI_ShowString(0,32,"CUSTOM",16,1);
		break;
		
		default:
		GUI_ShowString(0,32,"INVALID",16,1);
		break;
	}
}

void pkt_sync_set(void)
{
	uint8_t sync = radioconfig.sync;
	cls();
	display_pkt_sync(sync);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			sync++; //temporary
			display_pkt_sync(sync);
		}
		else if(temp_key == K_LEFT)
		{
			sync--;
			display_pkt_sync(sync);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.sync = sync;
	Menu_Navigate(MENU_PARENT);
	display_pkt_settings();
}

void display_pkt_sync(uint8_t sync)
{
	sprintf(strbuffer,"SYNC:%d",sync);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void pkt_exit(void)
{
	Menu_Navigate(MENU_PARENT);
	display_settings();
}

//start,stop,step,time,rssi
void display_scan_settings(void);
void scan_start_set(void);
void scan_stop_set(void);
void scan_step_set(void);
void scan_time_set(void);
void scan_rssi_set(void);
void scan_exit(void);
void display_start_freq(uint32_t freq);
void display_stop_freq(uint32_t freq);
void display_freq_step(uint32_t freq);
void display_freq_time(uint32_t ms);
void display_rssi_tr(float rssi);


// Menus  Name 							| Next 							| Prev 									| Parent 						| Child 				| SelectFunction | EnterFunction 						| Text
MENU_ITEM(m_scanstart, 			m_scanstop,					NULL_MENU,							m_scanstart,				NULL_MENU,			NULL,							scan_start_set,					"\1 Start frequency");
MENU_ITEM(m_scanstop,			 	m_scanstep,		 			m_scanstart,						m_scanstart,				NULL_MENU,			NULL,							scan_stop_set,					"\2 Stop frequency");
MENU_ITEM(m_scanstep,				m_scantime,					m_scanstop,							m_scanstart,				NULL_MENU,			NULL,							scan_step_set, 					"\3 Frequency step");
MENU_ITEM(m_scantime, 			m_scanrssi,					m_scanstep,							m_scanstart,				NULL_MENU,			NULL,							scan_time_set,					"\4 Scan time");
MENU_ITEM(m_scanrssi,				m_scanexit,					m_scantime,  						m_scanstart,				NULL_MENU, 			NULL, 						scan_rssi_set,					"\5 RSSI threshold");
MENU_ITEM(m_scanexit,				NULL_MENU,					m_scanrssi,							m_pktsettings,			NULL_MENU, 			NULL, 						scan_exit,							"\6 Exit           ");

void settings_scan(void)
{
	Menu_Navigate(MENU_CHILD);
	display_scan_settings();
}

void display_scan_settings(void)
{
	cls();
	menu_display(&m_scanstart,0);
	menu_display(&m_scanstop,1);
	menu_display(&m_scanstep,1);
	menu_display(&m_scantime,1);
	menu_display(&m_scanrssi,1);
	menu_display(&m_scanexit,1);
}

void scan_start_set(void)
{
	uint32_t freq = radioconfig.rxstartfreq;
	cls();
	display_start_freq(freq);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			freq += radioconfig.rxstep; //add restrictions!
			display_start_freq(freq);
		}
		else if(temp_key == K_LEFT)
		{
			freq -= radioconfig.rxstep; //add restrictions!
			display_start_freq(freq);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.rxstartfreq = freq;
	Menu_Navigate(MENU_PARENT);
	display_scan_settings();
}

void display_start_freq(uint32_t freq)
{
	clear_line16(32);
	sprintf(strbuffer,"START:%dkHz",freq);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void scan_stop_set(void)
{
	uint32_t freq = radioconfig.rxstopfreq;
	cls();
	display_stop_freq(freq);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			freq += radioconfig.rxstep; //add restrictions!
			display_stop_freq(freq);
		}
		else if(temp_key == K_LEFT)
		{
			freq -= radioconfig.rxstep; //add restrictions!
			display_stop_freq(freq);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.rxstopfreq = freq;
	Menu_Navigate(MENU_PARENT);
	display_scan_settings();
}

void display_stop_freq(uint32_t freq)
{
	clear_line16(32);
	sprintf(strbuffer,"STOP:%dkHz",freq);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void scan_step_set(void)
{
	uint32_t step = radioconfig.rxstep;
	cls();
	display_freq_step(step);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			step += 25; //temp.
			display_freq_step(step);
		}
		else if(temp_key == K_LEFT)
		{
			step -= 25; //temp.
			display_freq_step(step);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.rxstep = step;
	Menu_Navigate(MENU_PARENT);
	display_scan_settings();
}

void display_freq_step(uint32_t freq)
{
	clear_line16(32);
	sprintf(strbuffer,"STEP:%dkHz",freq);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void scan_time_set(void)
{
	uint32_t time = radioconfig.rxinterval;
	cls();
	display_freq_time(time);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			time += 10; //temp.
			display_freq_time(time);
		}
		else if(temp_key == K_LEFT)
		{
			time -= 10; //temp.
			if(time < 10) time = 10;
			display_freq_time(time);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.rxinterval = time;
	Menu_Navigate(MENU_PARENT);
	display_scan_settings();
}

void display_freq_time(uint32_t ms)
{
	clear_line16(32);
	sprintf(strbuffer,"TIME:%dms",ms);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void scan_rssi_set(void)
{
	float rssi = radioconfig.rssitr;
	cls();
	display_rssi_tr(rssi);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			rssi += 0.5; //temp.
			if(rssi > -20.0) rssi = -20.0;
			display_rssi_tr(rssi);
		}
		else if(temp_key == K_LEFT)
		{
			rssi -= 0.5; //temp.
			if(rssi < -127.5) rssi = -127.5;
			display_rssi_tr(rssi);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.rssitr = rssi;
	Menu_Navigate(MENU_PARENT);
	display_scan_settings();
}

void display_rssi_tr(float rssi)
{
	clear_line16(32);
	sprintf(strbuffer,"RSSI:%.1fdBm",rssi);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void scan_exit(void)
{
	Menu_Navigate(MENU_PARENT);
	display_settings();
}


//start,stop,step,time,modulation
void display_jam_settings(void);
void jam_start_set(void);
void jam_stop_set(void);
void jam_step_set(void);
void jam_time_set(void);
void jam_mod_set(void);
void jam_exit(void);
void display_jam_mod(uint8_t mod);

// Menus  Name 							| Next 							| Prev 									| Parent 						| Child 				| SelectFunction | EnterFunction 						| Text
MENU_ITEM(m_jamstart, 			m_jamstop,					NULL_MENU,							m_jamstart,					NULL_MENU,			NULL,							jam_start_set,						"\1 Start frequency");
MENU_ITEM(m_jamstop,			 	m_jamstep,		 			m_jamstart,							m_jamstart,					NULL_MENU,			NULL,							jam_stop_set,							"\2 Stop frequency");
MENU_ITEM(m_jamstep,				m_jamtime,					m_jamstop,							m_jamstart,					NULL_MENU,			NULL,							jam_step_set,		 					"\3 Frequency step");
MENU_ITEM(m_jamtime,	 			m_jammod,						m_jamstep,							m_jamstart,					NULL_MENU,			NULL,							jam_time_set,							"\4 Jam time");
MENU_ITEM(m_jammod,					m_jamexit,					m_jamtime,  						m_jamstart,					NULL_MENU, 			NULL, 						jam_mod_set,							"\5 Modulation");
MENU_ITEM(m_jamexit,				NULL_MENU,					m_jammod,								m_pktsettings,			NULL_MENU, 			NULL, 						jam_exit,									"\6 Exit           ");

void settings_jam(void)
{
	Menu_Navigate(MENU_CHILD);
	display_jam_settings();
}

void display_jam_settings(void)
{
	cls();
	menu_display(&m_jamstart,0);
	menu_display(&m_jamstop,1);
	menu_display(&m_jamstep,1);
	menu_display(&m_jamtime,1);
	menu_display(&m_jammod,1);
	menu_display(&m_jamexit,1);
}

void jam_start_set(void)
{
	uint32_t freq = radioconfig.txstartfreq;
	cls();
	display_start_freq(freq);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			freq += radioconfig.txstep; //add restrictions!
			display_start_freq(freq);
		}
		else if(temp_key == K_LEFT)
		{
			freq -= radioconfig.txstep; //add restrictions!
			display_start_freq(freq);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.txstartfreq = freq;
	Menu_Navigate(MENU_PARENT);
	display_jam_settings();
}

void jam_stop_set(void)
{
	uint32_t freq = radioconfig.txstopfreq;
	cls();
	display_stop_freq(freq);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			freq += radioconfig.txstep; //add restrictions!
			display_stop_freq(freq);
		}
		else if(temp_key == K_LEFT)
		{
			freq -= radioconfig.txstep; //add restrictions!
			display_stop_freq(freq);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.txstopfreq = freq;
	Menu_Navigate(MENU_PARENT);
	display_jam_settings();
}

void jam_step_set(void)
{
	uint32_t step = radioconfig.txstep;
	cls();
	display_freq_step(step);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			step += 25; //temp.
			display_freq_step(step);
		}
		else if(temp_key == K_LEFT)
		{
			step -= 25; //temp.
			display_freq_step(step);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.txstep = step;
	Menu_Navigate(MENU_PARENT);
	display_jam_settings();
}

void jam_time_set(void)
{
	uint32_t time = radioconfig.txinterval;
	cls();
	display_freq_time(time);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			time++; //temp.
			display_freq_time(time);
		}
		else if(temp_key == K_LEFT)
		{
			time--; //temp.
			if(time < 1) time = 1;
			display_freq_time(time);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.txinterval = time;
	Menu_Navigate(MENU_PARENT);
	display_jam_settings();
}

void jam_mod_set(void)
{
	uint8_t mod = radioconfig.txmodulation;
	cls();
	display_jam_mod(mod);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			mod++; //temp.
			if(mod > 2) mod = 2;
			display_jam_mod(mod);
		}
		else if(temp_key == K_LEFT)
		{
			mod--; //temp.
			if(mod < 1) mod = 1;
			display_jam_mod(mod);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.txmodulation = mod;
	Menu_Navigate(MENU_PARENT);
	display_jam_settings();
}

void 	display_jam_mod(uint8_t mod)
{
	clear_line16(32);
	switch(mod)
	{
		case 1:
		GUI_ShowString(0,32,"MOD: CW",16,1);	
		break;
		
		case 2:
		GUI_ShowString(0,32,"MOD: PREAMBLE",16,1);	
		break;
		
		default:
		GUI_ShowString(0,32,"MOD: INVALID",16,1);	
		break;
	}
}

void jam_exit(void)
{
	Menu_Navigate(MENU_PARENT);
	display_settings();
}


//own id, pair id, packet count, interval, tx power
void display_other_settings(void);
void other_own_id(void);
void other_pair_id(void);
void other_pkt_cnt(void);
void other_pkt_interval(void);
void other_tx_power(void);
void other_exit(void);

void display_own_id(uint32_t id);
void display_pair_id(uint32_t id);
void display_pkt_cnt(uint32_t cnt);
void display_pkt_interval(uint32_t interval);
void display_tx_power(int8_t power);

// Menus  Name 							| Next 							| Prev 									| Parent 						| Child 				| SelectFunction | EnterFunction 						| Text
MENU_ITEM(m_own_id,		 			m_pair_id,					NULL_MENU,							m_own_id,						NULL_MENU,			NULL,							other_own_id,							"\1 Own ID");
MENU_ITEM(m_pair_id,			 	m_pkt_cnt,		 			m_own_id,								m_own_id,						NULL_MENU,			NULL,							other_pair_id,						"\2 Pair ID");
MENU_ITEM(m_pkt_cnt,				m_pkt_interval,			m_pair_id,							m_own_id,						NULL_MENU,			NULL,							other_pkt_cnt,	 					"\3 Packet count");
MENU_ITEM(m_pkt_interval,		m_txpower,					m_pkt_cnt,							m_own_id,						NULL_MENU,			NULL,							other_pkt_interval,				"\4 Packet interval");
MENU_ITEM(m_txpower,				m_other_exit,				m_pkt_interval,					m_own_id,						NULL_MENU, 			NULL, 						other_tx_power,						"\5 TX power");
MENU_ITEM(m_other_exit,			NULL_MENU,					m_txpower,							m_pktsettings,			NULL_MENU, 			NULL, 						other_exit,								"\6 Exit           ");

void settings_other(void)
{
	Menu_Navigate(MENU_CHILD);
	display_other_settings();
}

void display_other_settings(void)
{
	cls();
	menu_display(&m_own_id,0);
	menu_display(&m_pair_id,1);
	menu_display(&m_pkt_cnt,1);
	menu_display(&m_pkt_interval,1);
	menu_display(&m_txpower,1);
	menu_display(&m_other_exit,1);
}

void other_own_id(void)
{
	int32_t id = radioconfig.id;
	cls();
	display_own_id(id);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			id++;
			display_own_id(id);
		}
		else if(temp_key == K_LEFT)
		{
			id--;
			if(id < 0) id = 0;
			display_own_id(id);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.id = id;
	Menu_Navigate(MENU_PARENT);
	display_other_settings();
}

void display_own_id(uint32_t id)
{
	clear_line16(32);
	sprintf(strbuffer,"OWN ID:%d",id);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void other_pair_id(void)
{
	int32_t id = radioconfig.pair_id;
	cls();
	display_pair_id(id);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			id++;
			display_pair_id(id);
		}
		else if(temp_key == K_LEFT)
		{
			id--;
			if(id < 0) id = 0;
			display_pair_id(id);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.pair_id = id;
	Menu_Navigate(MENU_PARENT);
	display_other_settings();
}

void display_pair_id(uint32_t id)
{
	clear_line16(32);
	sprintf(strbuffer,"PAIR ID:%d",id);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void other_pkt_cnt(void)
{
	int32_t count = radioconfig.pkt_count;
	cls();
	display_pkt_cnt(count);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			count++;
			display_pkt_cnt(count);
		}
		else if(temp_key == K_LEFT)
		{
			count--;
			if(count < 0) count = 0;
			display_pkt_cnt(count);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.pkt_count= count;
	Menu_Navigate(MENU_PARENT);
	display_other_settings();
}

void display_pkt_cnt(uint32_t cnt)
{
	clear_line16(32);
	sprintf(strbuffer,"COUNT:%d",cnt);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void other_pkt_interval(void)
{
	int32_t interval = radioconfig.txsendinterval;
	cls();
	display_pkt_interval(interval);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			interval += 100;
			display_pkt_interval(interval);
		}
		else if(temp_key == K_LEFT)
		{
			interval -= 100;
			if(interval < 0) interval = 0;
			display_pkt_interval(interval);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.txsendinterval= interval;
	Menu_Navigate(MENU_PARENT);
	display_other_settings();
}

void display_pkt_interval(uint32_t interval)
{
	clear_line16(32);
	sprintf(strbuffer,"INTERVAL:%d",interval);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void other_tx_power(void)
{
	int8_t power = radioconfig.txpower;
	cls();
	display_tx_power(power);
	while(1)
	{
		scankey();
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_RIGHT)
		{
			power++;
			if(power > 22) power = 22;
			display_tx_power(power);
		}
		else if(temp_key == K_LEFT)
		{
			power--;
			if(power < -9) power = -9;
			display_tx_power(power);
		}
		else if(temp_key == K_ENTER) break;
	}
	radioconfig.txpower = power;
	Menu_Navigate(MENU_PARENT);
	display_other_settings();
}

void display_tx_power(int8_t power)
{
	clear_line16(32);
	sprintf(strbuffer,"POWER:%d",power);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void other_exit(void)
{
	Menu_Navigate(MENU_PARENT);
	display_settings();
}


void settings_save(void)
{
	writeconfig();
	menu_display(CurrentMenuItem,1);
	Menu_Navigate(MENU_PARENT);
	display_main_menu();
}

void save_and_reset(void)
{
	writeconfig();
	cls();
	GUI_ShowString(0,16,"RESET DEVICE",8,1);
	delay_ms(500);
	bsp_reset_proc();
}

void settings_exit(void)
{
	menu_display(CurrentMenuItem,1);
	Menu_Navigate(MENU_PARENT);
	display_main_menu();
}

void Generic_Write(const char* Text)
{
	if(Text)	
	{
		menu_display(CurrentMenuItem,0);
	}
}

static void Level1Item3_Enter(void)
{
	
}

void updatescreen(void)
{
	//SSD1306_Clear(0);
	if(!menu_mode)
	{
//		sprintf(strbuffer, "%02d:%02d:%02d   %.2fV",timestamp.hour,timestamp.minute,timestamp.second,Vcc);
//		GUI_ShowString(0,0,strbuffer,16,1);
	}
}

void cls(void)
{
	SSD1306_Clear(0);
}

void clear_line16(uint8_t pos)
{
	GUI_ShowString(0,pos,BLANK_STRING_16,16,1);
}

void menu_display(Menu_Item_t* menu,uint8_t mode)
{
	GUI_ShowString(0,8*(menu->Text[0]-1),(char*)menu->Text+1,8,mode);
//	
//	switch(menu->Text[0])
//	{
//		case '\1':
//		default:
//		GUI_ShowString(0,0,(char*)menu->Text+1,16,mode);
//		break;
//			
//		case '\2':
//		GUI_ShowString(0,16,(char*)menu->Text+1,16,mode);
//		break;
//			
//		case '\3':
//		GUI_ShowString(0,32,(char*)menu->Text+1,16,mode);
//		break;
//			
//		case '\4':
//		GUI_ShowString(0,48,(char*)menu->Text+1,16,mode);
//		break;
//	}
}



