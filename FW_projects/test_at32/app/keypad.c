#include "keypad.h"

//from AI
#include "keypad.h"

#define DEBOUNCE_TICKS 3

static const char KeyCodes[] = {0,'1','2','3','A','4','5','6','B','7','8','9','C','*','0','#','D'};

static const keypad_key_t keymap[4][4] =
{
	{KEY_1, KEY_2, KEY_3, KEY_A},
	{KEY_4, KEY_5, KEY_6, KEY_B},
	{KEY_7, KEY_8, KEY_9, KEY_C},
	{KEY_STAR, KEY_0, KEY_HASH, KEY_D}
};

static volatile keypad_key_t key_ready = KEY_NONE;

static uint8_t current_row = 0;

static uint8_t debounce_cnt[4][4];
static uint8_t stable_state[4][4];

/* ========================================================= */

static void Rows_AllHigh(void)
{
	gpio_bits_set(R0_GPIO_PORT, R0_PIN);
	gpio_bits_set(R1_GPIO_PORT, R1_PIN);
	gpio_bits_set(R2_GPIO_PORT, R2_PIN);
	gpio_bits_set(R3_GPIO_PORT, R3_PIN);
}

/* ========================================================= */

void Keypad_Init(void)
{
	gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
	gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;//GPIO_OUTPUT_OPEN_DRAIN;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_pins = R0_PIN;
	gpio_init(R0_GPIO_PORT, &gpio_init_struct);
	gpio_init_struct.gpio_pins = R1_PIN;
	gpio_init(R1_GPIO_PORT, &gpio_init_struct);
	gpio_init_struct.gpio_pins = R2_PIN;
	gpio_init(R2_GPIO_PORT, &gpio_init_struct);
	gpio_init_struct.gpio_pins = R3_PIN;
	gpio_init(R3_GPIO_PORT, &gpio_init_struct);
	Rows_AllHigh();

	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init_struct.gpio_pull = GPIO_PULL_UP;
	gpio_init_struct.gpio_pins = C0_PIN;
	gpio_init(C0_GPIO_PORT, &gpio_init_struct);
	gpio_init_struct.gpio_pins = C1_PIN;
	gpio_init(C1_GPIO_PORT, &gpio_init_struct);
	gpio_init_struct.gpio_pins = C2_PIN;
	gpio_init(C2_GPIO_PORT, &gpio_init_struct);
	gpio_init_struct.gpio_pins = C3_PIN;
	gpio_init(C3_GPIO_PORT, &gpio_init_struct);
}

/* ========================================================= */

static uint8_t Read_Column(uint8_t col)
{
	switch(col)
	{
		case 0: return !gpio_input_data_bit_read(C0_GPIO_PORT, C0_PIN);
		case 1: return !gpio_input_data_bit_read(C1_GPIO_PORT, C1_PIN);
		case 2: return !gpio_input_data_bit_read(C2_GPIO_PORT, C2_PIN);
		case 3: return !gpio_input_data_bit_read(C3_GPIO_PORT, C3_PIN);
	}
	return 0;
}

/* ========================================================= */

void Keypad_Scan_ISR(void)
{
	Rows_AllHigh();
	switch(current_row)
	{
		case 0:	gpio_bits_reset(R0_GPIO_PORT, R0_PIN); break;
		case 1:	gpio_bits_reset(R1_GPIO_PORT, R1_PIN); break;
		case 2:	gpio_bits_reset(R2_GPIO_PORT, R2_PIN); break;
		case 3:	gpio_bits_reset(R3_GPIO_PORT, R3_PIN); break;
	}
	for(uint8_t col = 0; col < 4; col++)
	{
		uint8_t pressed = Read_Column(col);
		if(pressed != stable_state[current_row][col])
		{
			debounce_cnt[current_row][col]++;
			if(debounce_cnt[current_row][col] >= DEBOUNCE_TICKS)
			{
				debounce_cnt[current_row][col] = 0;
				stable_state[current_row][col] = pressed;
				if(pressed) key_ready = keymap[current_row][col];
			}
		}
		else debounce_cnt[current_row][col] = 0;
	}
	current_row++;
	if(current_row >= 4) current_row = 0;
}

/* ========================================================= */

char Keypad_GetKey(void)
{
	keypad_key_t key = KEY_NONE;
	__disable_irq();
	key = key_ready;
	key_ready = KEY_NONE;
	__enable_irq();
	return KeyCodes[key];
}
