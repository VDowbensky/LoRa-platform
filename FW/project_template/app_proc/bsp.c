#include "bsp.h"

void radio_power_on(void)
{
	gpio_bits_set(RF_EN_GPIO_PORT,RF_EN_PIN); //turn on DCDC or LDO
}

void radio_power_off(void)
{
	gpio_bits_reset(RF_EN_GPIO_PORT,RF_EN_PIN); //turn off DCDC or LDO
}

void txled_on(void)
{
	gpio_bits_set(RED_GPIO_PORT,RED_PIN);
}

void txled_off(void)
{
	gpio_bits_reset(RED_GPIO_PORT,RED_PIN);
}

void rxled_on(void)
{
	gpio_bits_set(GREEN_GPIO_PORT,GREEN_PIN);
}

void rxled_off(void)
{
	gpio_bits_reset(GREEN_GPIO_PORT,GREEN_PIN);
}

//void delay_ms(uint32_t ms)
//{
//	uint32_t start = ms_ticks;
//	while ((ms_ticks - start) < ms);
//}

void bsp_reset_proc(void)
{
	//wk_usb_app_task();
	delay_ms(100);
	NVIC_SystemReset();
}

uint64_t bsp_get_uid(void)
{
	return *(uint64_t*)0x4002002c;
}

