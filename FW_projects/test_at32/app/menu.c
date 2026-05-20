#include "menu.h"

volatile uint8_t menu_level = 0;

void menu_proc(void)
{
	char key;
	
	key = Keypad_GetKey();
	if(key != KEY_NONE)
	{
		printf("Key pressed: %c\r\n",key); 
	}
	switch(key)
	{
		case '0':
		break;
		
		case '1':
		break;
		
		case '2':
		break;
		
		case '3':
		break;
		
		case '4':
		break;
		
		case '5':
		break;
		
		case '6':
		break;
		
		case '7':
		break;
		
		case '8':
		break;
		
		case '9':
		break;
		
		case '*':
		break;
		
		case '#':
		GUI_ShowString(0,0,"0 - RF settings",16,1);
		//sprintf(strbuffer,"Chip:%d",radioconfig.chip);
		GUI_ShowString(0,16,"1 - RF settings",16,1);
		//sprintf(strbuffer,"Freq:%d",radioconfig.freq/1000);
		GUI_ShowString(0,32,"1 - RF settings",16,1);
		GUI_ShowString(0,48,"# - Exit",16,1);
		break;
		
		case 'A':
		break;
		
		case 'B':
		break;
		
		case 'C':
		break;
		
		case 'D':
		break;
		
		default:
		break;
	}
}