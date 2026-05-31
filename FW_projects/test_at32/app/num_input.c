#include "num_input.h"
#include "keypad.h"
#include "ssd1306.h"

#define MAX_DIGITS 9

static char input_buf[MAX_DIGITS + 2];
static uint8_t input_len;
static uint8_t negative;
/* digit storage area */
static char digits[MAX_DIGITS];

/* ======================================================= */

static void Display_Input(void)
{
	GUI_ShowString(0,32,"                ",16,1);
	GUI_ShowString(0,32,input_buf,16,1);
	//GUI_ShowString(0,32,"test",16,1);
}

/* ======================================================= */

static void Rebuild_String(void)
{
	uint8_t idx = 0;

	if(negative) input_buf[idx++] = '-';
	if(input_len == 0) input_buf[idx++] = '0';
	else
	{
		for(uint8_t i=0;i<input_len;i++) input_buf[idx++] = digits[i];
	}
	input_buf[idx] = 0;
}

/* ======================================================= */

static void Append_Digit(char d)
{
	if(input_len >= MAX_DIGITS) return;
	/* remove leading zero */
	if(input_len == 1 && digits[0] == '0') digits[0] = d;
	else digits[input_len++] = d;
	if(input_len == 0) input_len = 1;
	Rebuild_String();
}

/* ======================================================= */

static void Remove_Last(void)
{
	if(input_len == 0) return;
	input_len--;
	if(input_len == 0)
	{
		digits[0] = '0';
		input_len = 1;
	}
	Rebuild_String();
}

/* ======================================================= */

static void Toggle_Sign(void)
{
    negative ^= 1;
    Rebuild_String();
}

/* ======================================================= */

static int8_t Key_To_Digit(char key,char* d)
{
  if((key < '0') || (key > '9')) return 0;
	else
	{
		*d = key;
		return 1;
	}
}

/* ======================================================= */

static int32_t Convert_To_Int32(void)
{
	int32_t v = 0;
	for(uint8_t i=0;i<input_len;i++)
	{
		v *= 10;
		v += digits[i] - '0';
	}
	if(negative) v = -v;
	return v;
}

/* ======================================================= */

void NumberInput_Start(int32_t initial)
{
	negative = 0;
	if(initial < 0)
	{
		negative = 1;
		initial = -initial;
	}
	input_len = 0;
	if(initial == 0)
	{
		digits[0] = '0';
		input_len = 1;
	}
	else
	{
		char tmp[10];
		uint8_t cnt = 0;
		while(initial)
		{
			tmp[cnt++] = '0' + (initial % 10);
			initial /= 10;
		}
		for(uint8_t i=0;i<cnt;i++) digits[i] = tmp[cnt-1-i];
		input_len = cnt;
	}
	Rebuild_String();
	Display_Input();
}

/* ======================================================= */

int8_t NumberInput_Run(int32_t* value)
{
	char  key;
	char d;
	key = Keypad_GetKey();
	if(key == KEY_NONE) return 0;
	if(Key_To_Digit(key,&d))
	{
		Append_Digit(d);
		Display_Input();
		return 0;
	}
	switch(key)
	{
		case '*':
		Toggle_Sign();
		Display_Input();
		break;
		
		case 'D':
		Remove_Last();
		Display_Input();
		break;

		case '#':
		*value = Convert_To_Int32();
		return 1;
		
		case 'C':
		return -1;

		default:
		break;
	}
	return 0;
}
////////////////////////////////////////////////////
