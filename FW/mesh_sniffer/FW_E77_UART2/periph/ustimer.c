#include "ustimer.h"

//volatile bool sweepflag = false;

void ustimer_init(void)
{
	//use TIM16 & TIM17
	LL_TIM_InitTypeDef initstruct;
	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);
	
	//LL_TIM_StructInit(&initstruct);
	initstruct.Autoreload = 0xFFFFFFFFU;
	initstruct.CounterMode = LL_TIM_COUNTERMODE_DOWN;
	initstruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	initstruct.Prescaler = 48; //1 us
	initstruct.RepetitionCounter = 0;
	
	LL_TIM_Init(TIM16,&initstruct);
	LL_TIM_Init(TIM17,&initstruct);
	
	//enable/disable timers
	
	//config interrupts
	LL_TIM_EnableIT_UPDATE(TIM16);
	LL_TIM_EnableIT_UPDATE(TIM17);
	NVIC_ClearPendingIRQ(TIM16_IRQn);
	NVIC_ClearPendingIRQ(TIM17_IRQn);
	NVIC_EnableIRQ(TIM16_IRQn);
	NVIC_EnableIRQ(TIM17_IRQn);
	//LL_TIM_GenerateEvent_UPDATE(TIM16);
	//LL_TIM_GenerateEvent_UPDATE(TIM17);
}

void ustimer_start(uint8_t timer)
{
	switch(timer)
	{
		case 0:
		LL_TIM_EnableCounter(TIM16);
		break;
		
		case 1:
		LL_TIM_EnableCounter(TIM17);
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
		LL_TIM_DisableCounter(TIM16);
		sweepflag[0] = false;
		break;
		
		case 1:
		LL_TIM_DisableCounter(TIM17);
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
		TIM16->ARR = (uint32_t)interval;
		break;
		
		case 1:
		TIM17->ARR = (uint32_t)interval;
		break;
	
		default:
		break;
	}
}

void TIM16_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM16) == 1)
	{
		LL_TIM_ClearFlag_UPDATE(TIM16);
		sweepflag[0] = true;
	}
}

void TIM17_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM17) == 1)
	{
		LL_TIM_ClearFlag_UPDATE(TIM17);
		sweepflag[1] = true;
	}
}



