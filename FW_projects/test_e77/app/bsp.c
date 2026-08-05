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
#include "encoder.h"


//void TIM2_Encoder_Init(void);
//void Encoder_Periodic_1ms_Check(void);

//volatile Encoder_Direction_t global_encoder_dir = ENCODER_NO_MOVING;
//static int32_t last_counter_val = 0;
//volatile uint32_t encoder_debounce_ticks = 0;
//volatile uint32_t encoder_idle_ticks = 0;

volatile uint16_t enc_cnt = ENC_COUNT;
volatile uint8_t Key = K_NONE;
volatile int16_t key_cnt = 0;
volatile bool keypress = false;

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
	Encoder_Init();
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
	enc_cnt--;
	if(enc_cnt == 0)
	{
		encoder_proc();
		enc_cnt = ENC_COUNT;
	}
	scankey();
}

void tim2_irq_process(void)
{
//	if (LL_TIM_IsActiveFlag_UPDATE(TIM2) && LL_TIM_IsEnabledIT_UPDATE(TIM2))
//	{
//		LL_TIM_ClearFlag_UPDATE(TIM2);
//		// Software Debounce Lockout Check
//		if (encoder_debounce_ticks > 0) return; // Drop this interrupt event; it falls within the bounce stabilization window
//		int32_t current_counter = LL_TIM_GetCounter(TIM2);
//		if (current_counter != last_counter_val)
//		{
//			// Reset the timeout tracking variables
//			encoder_debounce_ticks = DEBOUNCE_LOCKOUT_MS; 
//			encoder_idle_ticks = 0; 
//			// Read the direction hardware bit
//			if (LL_TIM_GetDirection(TIM2) == LL_TIM_COUNTERDIRECTION_UP) 
//			{
//				global_encoder_dir = ENCODER_CLOCKWISE; // Add your Clockwise event execution hook here
//			}
//			else 
//			{
//				global_encoder_dir = ENCODER_COUNTERCLOCKWISE;// Add your Counterclockwise event execution hook here
//			}
//			last_counter_val = current_counter;
//		}
//	}
}

void scankey(void) //called every 1 ms by systick
{
		//if(gpio_read(KEY_PORT[i],KEY_PIN[i]) == GPIO_LEVEL_LOW)
		if(LL_GPIO_IsInputPinSet(ENCODER_PUSH_PORT,ENCODER_PUSH_PIN) == 0)
		{
			key_cnt++;
			if(key_cnt >= KEY_SCAN_CNT) 
			{
				key_cnt = KEY_SCAN_CNT;
				keypress = true;
			}
		}
		else
		{
			key_cnt--;
			if((key_cnt) < 0) 
			{
				key_cnt = 0;
				if(keypress) 
				{
					keypress = false;
					Key = K_ENTER;
				}
			}
		}
}


void handle_keys(void) 
{
	if(Key == K_LEFT) printf("Up.\r\n");
	else if(Key == K_RIGHT) printf("Down.\r\n");
	else if(Key == K_ENTER) printf("Enter.\r\n");
	Key = K_NONE;
}







