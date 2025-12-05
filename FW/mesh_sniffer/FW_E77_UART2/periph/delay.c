#include "delay.h"

//static uint8_t fac_us  = 0;
//static uint16_t fac_ms = 0;


#define TICK_RATE		1000UL
#define CLK_FREQ		32000000UL
#define RELOAD			(CLK_FREQ / TICK_RATE)
#define FAC_US			(CLK_FREQ / 1000000UL)

/**
 * @brief Init the registers used in delay function
 * @param None
 * @retval None
 */
void delay_init(void)
{
	//uint32_t reload;
	//uint32_t tick_rate = 1000;
	//uint32_t clk_freq = 48000000UL;
	//if (clk_freq < 1000000) return;
	//fac_us = clk_freq / 1000000;
	//reload = clk_freq / tick_rate;
	//fac_ms = 1000 / tick_rate;
	SysTick_Config(RELOAD);
}

/**
 * @brief Delay some microseconds
 * @param nus The delay in microsecond 
 * @retval None
 */
void delay_us(uint32_t nus)
{
	uint32_t ticks;
	uint32_t tpre, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD;

	//if (!fac_us) return;
	ticks = nus * FAC_US;
	tpre  = SysTick->VAL;
	while (1) 
	{
		tnow = SysTick->VAL;
		if (tnow != tpre) 
		{
			if (tnow < tpre) tcnt += tpre - tnow;
			else tcnt += reload - tnow + tpre;
			tpre = tnow;
			if (tcnt >= ticks) break;
		}
	}
}

/**
 * @brief Delay some milliseconds
 * @param nms The delay in millisecond 
 * @retval None
 */
void delay_ms(uint32_t nms)
{
	delay_us(nms * 1000);
}