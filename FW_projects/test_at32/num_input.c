#include "num_input.h"
#include "keypad.h"
#include "ssd1306.h"

#define MAX_DIGITS 9

static char input_buf[MAX_DIGITS + 2];

static uint8_t input_len;

static uint8_t negative;

/* ======================================================= */

static void Display_Input(void)
{
    SSD1306_Clear();

    SSD1306_GotoXY(0,0);
    SSD1306_Puts("Enter Value", &Font_7x10, White);

    SSD1306_GotoXY(0,20);

    SSD1306_Puts(input_buf,
                 &Font_11x18,
                 White);

    SSD1306_GotoXY(0,50);

    SSD1306_Puts(
        "A:+/- *=DEL #=OK",
        &Font_6x8,
        White);

    SSD1306_UpdateScreen();
}

/* ======================================================= */

static void Rebuild_String(void)
{
    uint8_t idx = 0;

    if(negative)
        input_buf[idx++] = '-';

    if(input_len == 0)
    {
        input_buf[idx++] = '0';
    }
    else
    {
        for(uint8_t i=0;i<input_len;i++)
            input_buf[idx++] = input_buf[MAX_DIGITS+1+i];
    }

    input_buf[idx] = 0;
}

/* ======================================================= */

/* digit storage area */
static char digits[MAX_DIGITS];

/* ======================================================= */

static void Append_Digit(char d)
{
    if(input_len >= MAX_DIGITS)
        return;

    /* remove leading zero */

    if(input_len == 1 &&
       digits[0] == '0')
    {
        digits[0] = d;
    }
    else
    {
        digits[input_len++] = d;
    }

    if(input_len == 0)
        input_len = 1;

    Rebuild_String();
}

/* ======================================================= */

static void Remove_Last(void)
{
    if(input_len == 0)
        return;

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

static int8_t Key_To_Digit(
    keypad_key_t key,
    char* d)
{
    switch(key)
    {
        case KEY_0: *d='0'; return 1;
        case KEY_1: *d='1'; return 1;
        case KEY_2: *d='2'; return 1;
        case KEY_3: *d='3'; return 1;
        case KEY_4: *d='4'; return 1;
        case KEY_5: *d='5'; return 1;
        case KEY_6: *d='6'; return 1;
        case KEY_7: *d='7'; return 1;
        case KEY_8: *d='8'; return 1;
        case KEY_9: *d='9'; return 1;

        default:
            return 0;
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

    if(negative)
        v = -v;

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
            tmp[cnt++] =
                '0' + (initial % 10);

            initial /= 10;
        }

        for(uint8_t i=0;i<cnt;i++)
        {
            digits[i] =
                tmp[cnt-1-i];
        }

        input_len = cnt;
    }

    Rebuild_String();

    Display_Input();
}

/* ======================================================= */

int8_t NumberInput_Run(int32_t* value)
{
    keypad_key_t key;

    char d;

    key = Keypad_GetKey();

    if(key == KEY_NONE)
        return 0;

    if(Key_To_Digit(key,&d))
    {
        Append_Digit(d);

        Display_Input();

        return 0;
    }

    switch(key)
    {
        case KEY_A:

            Toggle_Sign();

            Display_Input();

            break;

        case KEY_STAR:

            Remove_Last();

            Display_Input();

            break;

        case KEY_HASH:

            *value = Convert_To_Int32();

            return 1;

        case KEY_D:

            return -1;

        default:
            break;
    }

    return 0;
}
////////////////////////////////////////////////////
int32_t value;

NumberInput_Start(-123);

while(1)
{
    int8_t r =
        NumberInput_Run(&value);

    if(r == 1)
    {
        /* OK */

        break;
    }

    if(r == -1)
    {
        /* canceled */

        break;
    }
}