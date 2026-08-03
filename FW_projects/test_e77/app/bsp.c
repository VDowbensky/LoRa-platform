#include "bsp.h"

#include "gpio.h"
#include "uart.h"
#include "adc.h"
//#include "i2c.h"
#include "ustimer.h"
#include "retarget.h"
#include "radio_proc.h"
#include "flash.h"
#include "stm32wlxx_it.h"


void TIM2_Encoder_Init(void);
void Encoder_Periodic_1ms_Check(void);

volatile Encoder_Direction_t global_encoder_dir = ENCODER_NO_MOVING;
static uint32_t last_counter_val = 0;
volatile uint32_t encoder_debounce_ticks = 0;
volatile uint32_t encoder_idle_ticks = 0;

volatile uint8_t Key = K_NONE;

volatile bool RxRestartFlag = false;
volatile uint32_t rxtestartcounter = RX_RESTART_INTERVAL;

void init_power_clk(void)
{
	//HSE 32 MHz
	
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_HSEM);
  /* HSEM_IRQn interrupt configuration */
  NVIC_SetPriority(HSEM_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(HSEM_IRQn);
	
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_EnableTcxo();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1);
  LL_RCC_LSI_Enable();
   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
  /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);
	
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHB3Prescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
	
	delay_init();
	SystemCoreClockUpdate();
	
}

void init_peripherals(void)
{
	mygpio_init();
	//myadc_init();
	RETARGET_SerialInit();
	TIM2_Encoder_Init();
	//ustimer_init();
}

void init_radio_specific(void)
{
	subghz_interface_init(); //main
	delay_ms(10);
//	readconfig();
//	if(radioconfig.chip == 0xff) 
//	{
//		radio_initconfig(1262,1);
//		printf("INIT CONFIG: OK\r\n");
//	}
//	radio_init();
}

void bsp_reset_proc(void)
{
	delay_ms(100);
	NVIC_SystemReset();
}

void led_on(void)
{
	LL_GPIO_SetOutputPin(LED_PORT,LED_PIN);
}

void led_off(void)
{
	LL_GPIO_ResetOutputPin(LED_PORT,LED_PIN);
}

void timing_irq_process(void)
{
	if(master)
	{
	 pkt_timecnt++;
   if((pkt_timecnt >= inter_packet_delay) && (master == true))
   {
     pkt_timecnt = 0;
     txpacketnumber++;
     //if((txpacketnumber <= txpacketcount) || (contTX)) tx_needed = true;
		 if(txpacketnumber <= txpacketcount) tx_request = true;
     else
     {
       tx_request = false;
       master = false;
       printf("TX: DONE\r\n");
     }
   }
	}
	if(sweep)
	{
		sweepcnt--;
		if(sweepcnt == 0) 
		{
			sweepcnt = sweeptime;
			sweepflag = true;
		}
	}
	rxtestartcounter--;
	if(rxtestartcounter == 0) 
	{
		RxRestartFlag = true;
		rxtestartcounter = RX_RESTART_INTERVAL;
	}
	Encoder_Periodic_1ms_Check();
}

void TIM2_Encoder_Init(void)
{
	// 1. Enable Peripheral Clocks
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	// 2. Configure GPIO PA0 (CH1) and PA1 (CH2)
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP; // Required internal pull-ups
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1; // AF1 maps TIM2_CH1 and TIM2_CH2
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// 3. Configure TIM2 Encoder Mode
	// X4 Mode: Counts on both rising and falling edges of TI1 and TI2
	LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
	// Configure Input Capture Filters to prevent noise (at 32 MHz clock)
	LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1_N8);
	LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1_N8);
	// Set Polarity (Rising edge means non-inverted signal mapping)
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_FALLING);
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);
	// 4. Configure Interrupts on Motion
	// Enable Update Interrupt: triggered every time the counter changes value
	LL_TIM_EnableIT_UPDATE(TIM2);
	// Setup NVIC for TIM2
	NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(TIM2_IRQn);
	// 5. Enable the Counter
	LL_TIM_SetCounter(TIM2, 0);
	LL_TIM_EnableCounter(TIM2);
}

void tim2_irq_process(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM2) && LL_TIM_IsEnabledIT_UPDATE(TIM2))
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);
		// Software Debounce Lockout Check
		if (encoder_debounce_ticks > 0) return; // Drop this interrupt event; it falls within the bounce stabilization window
		uint32_t current_counter = LL_TIM_GetCounter(TIM2);
		if (current_counter != last_counter_val)
		{
			// Reset the timeout tracking variables
			encoder_debounce_ticks = DEBOUNCE_LOCKOUT_MS; 
			encoder_idle_ticks = 0; 
			// Read the direction hardware bit
			if (LL_TIM_GetDirection(TIM2) == LL_TIM_COUNTERDIRECTION_UP) 
			{
				global_encoder_dir = ENCODER_CLOCKWISE; // Add your Clockwise event execution hook here
			}
			else 
			{
				global_encoder_dir = ENCODER_COUNTERCLOCKWISE;// Add your Counterclockwise event execution hook here
			}
			last_counter_val = current_counter;
		}
	}
}

// Process this within your 1ms SysTick routine
void Encoder_Periodic_1ms_Check(void)
{
	// Countdown the debounce safety window
	if (encoder_debounce_ticks > 0) encoder_debounce_ticks--;
	// Process the idle state timeout
	if (global_encoder_dir != ENCODER_NO_MOVING)
	{
		encoder_idle_ticks++;
		if (encoder_idle_ticks >= IDLE_TIMEOUT_MS)
		{
			if(global_encoder_dir == ENCODER_CLOCKWISE) Key = K_DOWN;
			else Key = K_UP;
			global_encoder_dir = ENCODER_NO_MOVING;
			encoder_idle_ticks = 0;
			// Add your "No Moving" notification or callback action here
		}
	}
}

void handle_keys(void) 
{
	if(Key == K_UP) printf("Up.\r\n");
	else if(Key == K_DOWN) printf("Down.\r\n");
	Key = K_NONE;
}







