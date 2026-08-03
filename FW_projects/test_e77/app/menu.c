#include "menu.h"
#include "gui.h"
//#include "beeper.h"
#include "radio.h"
#include "flash.h"
#include "app_cli.h"

void Generic_Write(const char* Text);
void cls(void);

void menu_display(Menu_Item_t* menu,uint8_t mode);
void menu_exit(void);

// Menus function
void radio_settings_enter(void);
void radio_settings_exit(void);

void b1_enter(void);
void b1_startfreq(void);
void b1_stopfreq(void);
void b1_freqstep(void);
void b1_rssi(void);
void b1_act(void);
void b1_exit(void);


void b2_enter(void);
void b2_startfreq(void);
void b2_stopfreq(void);
void b2_freqstep(void);
void b2_rssi(void);
void b2_act(void);
void b2_exit(void);

void display_start_freq(uint32_t f);
void display_stop_freq(uint32_t f);
void display_freq_step(uint32_t step);
void display_rssi_tr(int8_t tr);
void display_activation(bool act);

//bool flagPressed = false;
uint8_t Key = K_NONE;
bool keypress[KEY_COUNT] = {false,false,false};
int32_t key_cnt[3] = {0,0,0};
	
bool menu_mode = false;
	
GPIO_TypeDef* KEY_PORT[KEY_COUNT] = {K_UP_PORT,K_DOWN_PORT,K_ENTER_PORT};
const uint32_t KEY_PIN[KEY_COUNT] = {K_UP_PIN,K_DOWN_PIN,K_ENTER_PIN};

// Menus  Name 							| Next 							| Prev 									| Parent 						| Child 				| SelectFunction | EnterFunction 						| Text
MENU_ITEM(m_radiosettings, 	m_timesettings, 		NULL_MENU,							NULL_MENU, 					m_band1,	 			NULL,							radio_settings_enter,			"\1 Radio settings ");
MENU_ITEM(m_timesettings,		m_additional, 			m_radiosettings, 				NULL_MENU, 					NULL_MENU,			NULL, 						NULL,											"\2 Time setting   ");
MENU_ITEM(m_additional, 		m_exit, 						m_timesettings, 				NULL_MENU, 					NULL_MENU, 			NULL, 						NULL, 										"\3 Additional     ");
MENU_ITEM(m_exit, 					NULL_MENU,			 		m_additional, 					NULL_MENU, 					NULL_MENU, 			NULL, 						menu_exit, 								"\4 Exit           ");

//Radio settings
MENU_ITEM(m_band1, 					m_band2, 						NULL_MENU,		 					NULL_MENU,					m1_start, 			NULL, 						b1_enter,									"\1 Band 1 settings");
MENU_ITEM(m_band2, 					m_radioexit, 				m_band1,	 							NULL_MENU,					m2_start, 			NULL, 						b2_enter,				 					"\2 Band 2 settings");
MENU_ITEM(m_radioexit,			NULL_MENU,					m_band2,	 							m_radiosettings,		NULL_MENU, 			NULL, 						radio_settings_exit,			"\3 Save and exit  ");

//Radio settings - childs
MENU_ITEM(m1_start, 				m1_stop, 						NULL_MENU, 							NULL_MENU,					NULL_MENU, 			NULL, 						b1_startfreq,							"\1 Start Frequency");
MENU_ITEM(m1_stop, 					m1_step, 						m1_start,	 							NULL_MENU,					NULL_MENU, 			NULL, 						b1_stopfreq,		 					"\2 Stop Frequency ");
MENU_ITEM(m1_step, 					m1_rssi,						m1_stop,	 							NULL_MENU,					NULL_MENU, 			NULL, 						b1_freqstep,		 					"\3 Frequency Step ");
MENU_ITEM(m1_rssi, 					m1_act,							m1_step,	 							NULL_MENU,					NULL_MENU, 			NULL, 						b1_rssi,				 					"\4 RSSI threshold ");
MENU_ITEM(m1_act, 					m1_exit,						m1_rssi,	 							NULL_MENU,					NULL_MENU, 			NULL, 						b1_act,	 				 					"\4 Activation     ");
MENU_ITEM(m1_exit, 					NULL_MENU,					m1_act,		 							m_band1,						NULL_MENU, 			NULL, 						b1_exit,				 					"\4 Exit           ");

MENU_ITEM(m2_start, 				m2_stop, 						NULL_MENU, 							NULL_MENU,					NULL_MENU, 			NULL, 						b2_startfreq,							"\1 Start Frequency");
MENU_ITEM(m2_stop, 					m2_step, 						m2_start,	 							NULL_MENU,					NULL_MENU, 			NULL, 						b2_stopfreq,		 					"\2 Stop Frequency ");
MENU_ITEM(m2_step, 					m2_rssi,						m2_stop,	 							NULL_MENU,					NULL_MENU, 			NULL, 						b2_freqstep,		 					"\3 Frequency Step ");
MENU_ITEM(m2_rssi, 					m2_act,							m2_step,	 							NULL_MENU,					NULL_MENU, 			NULL, 						b2_rssi,				 					"\4 RSSI threshold ");
MENU_ITEM(m2_act, 					m2_exit,						m2_rssi,	 							NULL_MENU,					NULL_MENU, 			NULL, 						b2_act,	 				 					"\4 Activation     ");
MENU_ITEM(m2_exit, 					NULL_MENU,					m2_act,		 							m_band2,						NULL_MENU, 			NULL, 						b2_exit,				 					"\4 Exit           ");

//Time setting

void menu_init(void)
{
	//clear screen
	cls();
	Menu_SetGenericWriteCallback(Generic_Write);
	menu_display(&m_radiosettings,0);
	menu_display(&m_timesettings,1);
	menu_display(&m_additional,1);
	menu_display(&m_exit,1);
	
	Menu_Navigate(&m_radiosettings);
	menu_mode = true;
}

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
				case K_UP:
				if(CurrentMenuItem->Previous != &NULL_MENU)
				{
					//beep(3000,50);
					menu_display(CurrentMenuItem,1);
					Menu_Navigate(MENU_PREVIOUS);
				}
				break;
			
				case K_DOWN:
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

void menu_exit(void)
{
	cls();
	//delay_ms(200);
	menu_mode = false;
	radio_startscan();
}

//Radio settings
void radio_settings_enter(void)
{
	cls();
	Menu_Navigate(MENU_CHILD);
	menu_display(&m_band1,0);
	menu_display(&m_band2,1);
	menu_display(&m_radioexit,1);
}

void radio_settings_exit(void)
{
	writeconfig();
	menu_display(CurrentMenuItem,1);
	Menu_Navigate(MENU_PARENT);
	cls();
	menu_display(&m_radiosettings,0);
	menu_display(&m_timesettings,1);
	menu_display(&m_additional,1);
	menu_display(&m_exit,1);
}

//Radio settings - childs
//band 1
void b1_enter(void)
{
	Menu_Navigate(MENU_CHILD);
	menu_display(&m1_start,0);
	menu_display(&m1_stop,1);
	menu_display(&m1_step,1);
	menu_display(&m1_rssi,1);
}

void b1_exit(void)
{
	cls();
	Menu_Navigate(MENU_PARENT);
	menu_display(&m_band1,0);
	menu_display(&m_band2,1);
	menu_display(&m_radioexit,1);
}

void b1_startfreq(void)
{
	uint8_t temp_key;
	//read start, freq. from radioconfig
	uint32_t freq = globalrfconfig.LBconfig.startfreq / 1000;
	cls();
	GUI_ShowString(0,16,"BAND 1",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
	//key_up + 1 freq.step
		if(temp_key == K_UP)
		{
			freq += globalrfconfig.LBconfig.freqstep / 1000; //temporary
			if(freq >= globalrfconfig.LBconfig.stopfreq /1000) freq = globalrfconfig.LBconfig.stopfreq - 2 * (globalrfconfig.LBconfig.freqstep / 1000);
			if(freq > LB_FREQ_MAX / 1000) freq = LB_FREQ_MAX / 1000;
		}
	//key_down -1 freq.step
		if(temp_key == K_DOWN)
		{
			freq -= globalrfconfig.LBconfig.freqstep / 1000;
			if(freq < LB_FREQ_MIN / 1000) freq = LB_FREQ_MIN / 1000;
		}
	//call freq. display
		display_start_freq(freq);
	}
	Key = K_NONE;
	globalrfconfig.LBconfig.startfreq = freq * 1000;
	menu_display(&m1_start,0);
	menu_display(&m1_stop,1);
	menu_display(&m1_step,1);
	menu_display(&m1_rssi,1);
	Menu_Navigate(&m1_start);
}

void b1_stopfreq(void)
{
	uint8_t temp_key;
	//read stop freq. from radioconfig
	uint32_t freq = globalrfconfig.LBconfig.stopfreq / 1000;
	cls();
	GUI_ShowString(0,16,"BAND 1",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
	//key_up + 1 freq.step
		if(temp_key == K_UP)
		{
			freq += globalrfconfig.LBconfig.freqstep / 1000; //temporary
			if(freq > LB_FREQ_MAX / 1000) freq = LB_FREQ_MAX / 1000;
		}
	//key_down -1 freq.step
		if(temp_key == K_DOWN)
		{
			freq -= globalrfconfig.LBconfig.freqstep / 1000;
			if(freq <= globalrfconfig.LBconfig.startfreq /1000) freq = globalrfconfig.LBconfig.startfreq + 2 * (globalrfconfig.LBconfig.freqstep / 1000);
			if(freq <= LB_FREQ_MIN / 1000) freq = LB_FREQ_MIN / 1000 + globalrfconfig.LBconfig.freqstep / 1000;
		}
	//call freq. display
		display_stop_freq(freq);
	}
	Key = K_NONE;
	globalrfconfig.LBconfig.stopfreq = freq * 1000;
	menu_display(&m1_start,1);
	menu_display(&m1_stop,0);
	menu_display(&m1_step,1);
	menu_display(&m1_rssi,1);
	Menu_Navigate(&m1_stop);
}

void b1_freqstep(void)
{
	uint8_t temp_key;
	//read freq. stepfrom radioconfig
	uint32_t step = globalrfconfig.LBconfig.freqstep / 1000;
	cls();
	GUI_ShowString(0,16,"BAND 1",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
	//key_up + 1 freq.step
		if(temp_key == K_UP)
		{
			step += 10; //kHz 
			if(step > FREQ_STEP_MAX / 1000) step = FREQ_STEP_MAX / 1000;
		}
	//key_down -1 freq.step
		if(temp_key == K_DOWN)
		{
			step -= 10;;
			if(step <= FREQ_STEP_MIN / 1000) step = FREQ_STEP_MIN / 1000;
		}
	//call step display
		display_freq_step(step);
	}
	Key = K_NONE;
	globalrfconfig.LBconfig.freqstep = step * 1000;
	menu_display(&m1_start,1);
	menu_display(&m1_stop,1);
	menu_display(&m1_step,0);
	menu_display(&m1_rssi,1);
	Menu_Navigate(&m1_step);
}

void b1_rssi(void)
{
	uint8_t temp_key;
	//read rssi from radioconfig
	int8_t tr = globalrfconfig.LBconfig.rssithreshold;
	cls();
	GUI_ShowString(0,16,"BAND 1",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
	//key_up + 1 dB
		if(temp_key == K_UP)
		{
			tr++; 
			if(tr > RSSI_TR_MAX) tr = RSSI_TR_MAX;
		}
	//key_down -1 dB
		if(temp_key == K_DOWN)
		{
			tr--;;
			if(tr < RSSI_TR_MIN) tr = RSSI_TR_MIN;
		}
	//call rssi display
		display_rssi_tr(tr);
	}
	Key = K_NONE;
	globalrfconfig.LBconfig.rssithreshold = tr;
	menu_display(&m1_start,1);
	menu_display(&m1_stop,1);
	menu_display(&m1_step,1);
	menu_display(&m1_rssi,0);
	Menu_Navigate(&m1_rssi);
}

void b1_act(void)
{
	uint8_t temp_key;
	//read from radioconfig
	bool act = globalrfconfig.LBconfig.scan_active;
	cls();
	GUI_ShowString(0,16,"BAND 1",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_UP) act = true;
		if(temp_key == K_DOWN) act = false;
		display_activation(act);
	}
	Key = K_NONE;
	globalrfconfig.LBconfig.scan_active = act;
	menu_display(&m1_start,1);
	menu_display(&m1_stop,1);
	menu_display(&m1_step,1);
	menu_display(&m1_rssi,0);
	Menu_Navigate(&m1_act);
}

//band 2
void b2_enter(void)
{
	Menu_Navigate(MENU_CHILD);
	menu_display(&m2_start,0);
	menu_display(&m2_stop,1);
	menu_display(&m2_step,1);
	menu_display(&m2_rssi,1);
}

void b2_exit(void)
{
	cls();
	Menu_Navigate(MENU_PARENT);
	menu_display(&m_band1,1);
	menu_display(&m_band2,0);
	menu_display(&m_radioexit,1);
}

void b2_startfreq(void)
{
	uint8_t temp_key;
	//read start freq. from radioconfig
	uint32_t freq = globalrfconfig.HBconfig.startfreq / 1000;
	cls();
	GUI_ShowString(0,16,"BAND 2",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
	//key_up + 1 freq.step
		if(temp_key == K_UP)
		{
			freq += globalrfconfig.HBconfig.freqstep / 1000; //temporary
			if(freq >= globalrfconfig.HBconfig.stopfreq /1000) freq = globalrfconfig.HBconfig.stopfreq - 2 * (globalrfconfig.HBconfig.freqstep / 1000);
			if(freq > HB_FREQ_MAX / 1000) freq = HB_FREQ_MAX / 1000;
		}
	//key_down -1 freq.step
		if(temp_key == K_DOWN)
		{
			freq -= globalrfconfig.HBconfig.freqstep / 1000;
			if(freq < HB_FREQ_MIN / 1000) freq = HB_FREQ_MIN / 1000;
		}
	//call freq. display
		display_start_freq(freq);
	}
	Key = K_NONE;
	globalrfconfig.HBconfig.startfreq = freq * 1000;
	menu_display(&m2_start,0);
	menu_display(&m2_stop,1);
	menu_display(&m2_step,1);
	menu_display(&m2_rssi,1);
	Menu_Navigate(&m2_start);
}

void b2_stopfreq(void)
{
	uint8_t temp_key;
	//read stop freq. from radioconfig
	uint32_t freq = globalrfconfig.HBconfig.stopfreq / 1000;
	cls();
	GUI_ShowString(0,16,"BAND 2",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
	//key_up + 1 freq.step
		if(temp_key == K_UP)
		{
			freq += globalrfconfig.HBconfig.freqstep / 1000; //temporary
			if(freq > HB_FREQ_MAX / 1000) freq = HB_FREQ_MAX / 1000;
		}
	//key_down -1 freq.step
		if(temp_key == K_DOWN)
		{
			freq -= globalrfconfig.HBconfig.freqstep / 1000;
			if(freq <= globalrfconfig.HBconfig.startfreq /1000) freq = globalrfconfig.HBconfig.startfreq + 2 * (globalrfconfig.HBconfig.freqstep / 1000);
			if(freq <= HB_FREQ_MIN / 1000) freq = HB_FREQ_MIN / 1000 + globalrfconfig.HBconfig.freqstep / 1000;
		}
	//call freq. display
		display_stop_freq(freq);
	}
	Key = K_NONE;
	globalrfconfig.HBconfig.stopfreq = freq * 1000;
	menu_display(&m2_start,1);
	menu_display(&m2_stop,0);
	menu_display(&m2_step,1);
	menu_display(&m2_rssi,1);
	Menu_Navigate(&m2_stop);
}

void b2_freqstep(void)
{
	uint8_t temp_key;
	//read step from radioconfig
	uint32_t step = globalrfconfig.HBconfig.freqstep / 1000;
	cls();
	GUI_ShowString(0,16,"BAND 2",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
	//key_up + 1 freq.step
		if(temp_key == K_UP)
		{
			step += 10; //kHz 
			if(step > FREQ_STEP_MAX / 1000) step = FREQ_STEP_MAX / 1000;
		}
	//key_down -1 freq.step
		if(temp_key == K_DOWN)
		{
			step -= 10;;
			if(step <= FREQ_STEP_MIN / 1000) step = FREQ_STEP_MIN / 1000;
		}
	//call step display
		display_freq_step(step);
	}
	Key = K_NONE;
	globalrfconfig.HBconfig.freqstep = step * 1000;
	menu_display(&m2_start,1);
	menu_display(&m2_stop,1);
	menu_display(&m2_step,0);
	menu_display(&m2_rssi,1);
	Menu_Navigate(&m2_step);
}

void b2_rssi(void)
{
	uint8_t temp_key;
	//read rssi from radioconfig
	int8_t tr = globalrfconfig.HBconfig.rssithreshold;
	cls();
	GUI_ShowString(0,16,"BAND 2",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
	//key_up + 1 dB
		if(temp_key == K_UP)
		{
			tr++; 
			if(tr > RSSI_TR_MAX) tr = RSSI_TR_MAX;
		}
	//key_down -1 dB
		if(temp_key == K_DOWN)
		{
			tr--;;
			if(tr < RSSI_TR_MIN) tr = RSSI_TR_MIN;
		}
	//call step display
		display_rssi_tr(tr);
	}
	Key = K_NONE;
	globalrfconfig.HBconfig.rssithreshold = tr;
	menu_display(&m2_start,1);
	menu_display(&m2_stop,1);
	menu_display(&m2_step,1);
	menu_display(&m2_rssi,0);
	Menu_Navigate(&m2_rssi);
}

void b2_act(void)
{
	uint8_t temp_key;
	//read from radioconfig
	bool act = globalrfconfig.HBconfig.scan_active;
	cls();
	GUI_ShowString(0,16,"BAND 2",16,1);
	GUI_ShowString(0,48,"ENTER to save",16,1);
	while(Key != K_ENTER)
	{
		temp_key = Key;
		Key = K_NONE;
		if(temp_key == K_UP) act = true;
		if(temp_key == K_DOWN) act = false;
		display_activation(act);
	}
	Key = K_NONE;
	globalrfconfig.HBconfig.scan_active = act;
	menu_display(&m2_start,1);
	menu_display(&m2_stop,1);
	menu_display(&m2_step,1);
	menu_display(&m2_rssi,0);
	Menu_Navigate(&m2_act);
}


//Date/Time


void Generic_Write(const char* Text)
{
	if(Text)	
	{
		menu_display(CurrentMenuItem,0);
	}
}

//uint8_t getPressKey(void)
//{
//  for(uint8_t i = 0; i < KEY_COUNT; i++)
//  {
//		if((gpio_read(KEY_PORT[i],KEY_PIN[i]) == GPIO_LEVEL_LOW) && !flagButton[i])
//		{
//		 flagButton[i] = true;
//		 key_pressed = i;
//		}
//		if((gpio_read(KEY_PORT[i],KEY_PIN[i]) == GPIO_LEVEL_HIGH) && flagButton[i])
//	  {
//		  flagButton[i] = false;
//		  key_pressed = K_NONE;
//	  }
//  }
//  return key_pressed;
//}

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

void menu_display(Menu_Item_t* menu,uint8_t mode)
{
		switch(menu->Text[0])
		{
			case '\1':
			default:
			GUI_ShowString(0,0,(char*)menu->Text+1,16,mode);
			break;
			
			case '\2':
			GUI_ShowString(0,16,(char*)menu->Text+1,16,mode);
			break;
			
			case '\3':
			GUI_ShowString(0,32,(char*)menu->Text+1,16,mode);
			break;
			
			case '\4':
			GUI_ShowString(0,48,(char*)menu->Text+1,16,mode);
			break;
		}
}

void scankeys(void) //called every 1 ms by systick
{
	for(uint8_t i = 0; i < KEY_COUNT; i++)
	{
		//if(gpio_read(KEY_PORT[i],KEY_PIN[i]) == GPIO_LEVEL_LOW)
		if(LL_GPIO_IsInputPinSet(KEY_PORT[i],KEY_PIN[i]) == 0)
		{
			key_cnt[i]++;
			if(key_cnt[i] >= KEY_SCAN_CNT) 
			{
				key_cnt[i] = KEY_SCAN_CNT;
				keypress[i] = true;
			}
		}
		else
		{
			key_cnt[i]--;
			if((key_cnt[i]) < 0) 
			{
				key_cnt[i] = 0;
				if(keypress[i]) 
				{
					keypress[i] = false;
					Key = i;
				}
			}
		}
	}
}

void display_start_freq(uint32_t f)
{
	sprintf(strbuffer,"START:%dkHz",f);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void display_stop_freq(uint32_t f)
{
	sprintf(strbuffer,"STOP:%d kHz",f);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void display_freq_step(uint32_t step)
{
	sprintf(strbuffer,"STEP:%d kHz",step);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void display_rssi_tr(int8_t tr)
{
	sprintf(strbuffer,"THR:%d dBm",tr);
	GUI_ShowString(0,32,strbuffer,16,1);
}

void display_activation(bool act)
{
	if(act == true) sprintf(strbuffer,"ACTIVATED   ");
	else sprintf(strbuffer,"DEACTIVATED ");
	GUI_ShowString(0,32,strbuffer,16,1);
}

