#include "ustimer.h"

#include "tremo_rcc.h"
#include "tremo_timer.h"

bool sweepflag[2] = {false,false};

void ustimer_init(void)
{
	timer_init_t timerx_init;

	rcc_enable_peripheral_clk(RCC_PERIPHERAL_TIMER0, true);
	rcc_enable_peripheral_clk(RCC_PERIPHERAL_TIMER1, true);
	
	timerx_init.prescaler          = 31;  //sysclock 32MHz, is divided by (prescaler + 1) to 1MHz
	timerx_init.counter_mode       = TIMER_COUNTERMODE_UP;
	timerx_init.period             = 1000;   //1 ms as default
	timerx_init.clock_division     = TIMER_CKD_FPCLK_DIV1;
	//timerx_init.autoreload_preload = false;
	timerx_init.autoreload_preload = true;
	
	timer_init(TIMER0, &timerx_init);
	timer_config_interrupt(TIMER0, TIMER_DIER_UIE, ENABLE);
	timer_generate_event(TIMER0, TIMER_EGR_UG, ENABLE);
	timer_clear_status(TIMER0, TIMER_SR_UIF);
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);
	
	timer_init(TIMER1, &timerx_init);
	timer_config_interrupt(TIMER1, TIMER_DIER_UIE, ENABLE);
	timer_generate_event(TIMER1, TIMER_EGR_UG, ENABLE);
	timer_clear_status(TIMER1, TIMER_SR_UIF);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

void ustimer_start(uint8_t timer)
{
	switch(timer)
	{
		case 0:
		timer_cmd(TIMER0, true);
		break;
		
		case 1:
		timer_cmd(TIMER1, true);
		break;
	
		default:
		break;
	}
}

void ustimer_stop(uint8_t timer)
{
	switch(timer)
	{
		case 0:
		timer_cmd(TIMER0, false);
		sweepflag[0] = false;
		break;
		
		case 1:
		timer_cmd(TIMER1, false);
		sweepflag[1] = false;
		break;
		
		default:
		break;
	}
}

void ustimer_setinterval(uint8_t timer,uint16_t interval)
{
	switch(timer)
	{
		case 0:
		TIMER0->ARR = (uint32_t)interval;
		break;
		
		case 1:
		TIMER1->ARR = (uint32_t)interval;
		break;
	
		default:
		break;
	}
}

void TIMER0_IRQHandler(void)
{
	bool state;
	
	timer_get_status(TIMER0, TIMER_SR_UIF, &state);
	if(state)
	{
		timer_clear_status(TIMER0, TIMER_SR_UIF);
		sweepflag[0] = true;
	}
}

void TIMER1_IRQHandler(void)
{
	bool state;
	
	timer_get_status(TIMER1, TIMER_SR_UIF, &state);
	if(state)
	{
		timer_clear_status(TIMER1, TIMER_SR_UIF);
		sweepflag[1] = true;
	}
}



