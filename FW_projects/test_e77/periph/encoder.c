#include "encoder.h"

static int32_t EncoderPosition = 0;
static uint32_t LastCounter;
static encoder_event_t LastEvent;
//static volatile encoder_event_t fifo[ENC_FIFO_SIZE];
//static volatile uint8_t wr;
//static volatile uint8_t rd;
static int16_t acc = 0;

static void Encoder_GPIO_Init(void)
{
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_GPIO_InitTypeDef gpio;

	gpio.Pin = ENCODER_A_PIN | ENCODER_B_PIN;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_UP;
	gpio.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(ENCODER_PORT, &gpio);
}

static void Encoder_TIM_Init(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	LL_TIM_DisableCounter(TIM2);
	LL_TIM_SetPrescaler(TIM2, 0);
	LL_TIM_SetAutoReload(TIM2, 0xFFFFFFFF);
	LL_TIM_SetCounterMode(TIM2,LL_TIM_COUNTERMODE_UP);
	LL_TIM_ENCODER_InitTypeDef enc;
	LL_TIM_ENCODER_StructInit(&enc);
	enc.EncoderMode = LL_TIM_ENCODERMODE_X4_TI12;
	enc.IC1Polarity = LL_TIM_IC_POLARITY_RISING;
	enc.IC2Polarity = LL_TIM_IC_POLARITY_RISING;
	enc.IC1Filter = LL_TIM_IC_FILTER_FDIV32_N8;
	enc.IC2Filter = LL_TIM_IC_FILTER_FDIV32_N8;
	LL_TIM_ENCODER_Init(TIM2, &enc);
	LL_TIM_SetCounter(TIM2, 0);
	LastCounter = 0;
	LL_TIM_EnableCounter(TIM2);
}

	
void Encoder_Init(void)
{
	Encoder_GPIO_Init();
	Encoder_TIM_Init();
}

int32_t Encoder_GetDelta(void)
{
	uint32_t cnt;
	int32_t delta;
	cnt = LL_TIM_GetCounter(TIM2);
	delta = (int32_t)(cnt - LastCounter);
	LastCounter = cnt;
	EncoderPosition += delta;
	return delta;
}

int32_t Encoder_GetPosition(void)
{
	return EncoderPosition;
}

void Encoder_SetPosition(int32_t pos)
{
	EncoderPosition = pos;
	LL_TIM_SetCounter(TIM2, 0);
	LastCounter = 0;
}

static int32_t Encoder_GetStep(void)
{
	acc += Encoder_GetDelta();
	if(acc >= 4)
	{
		acc = 0;
		return +1;
	}
	if(acc <= -4)
	{
		acc = 0;
		return -1;
	}
	return 0;
}

void encoder_proc(void)
{
	int32_t step = Encoder_GetStep();
	if(step > 0) 
	{
		Key = K_LEFT;
		LastEvent = ENC_EVENT_CW;
	}
	else if(step < 0) 
	{
		Key = K_RIGHT;
		LastEvent = ENC_EVENT_CCW;
	}
}