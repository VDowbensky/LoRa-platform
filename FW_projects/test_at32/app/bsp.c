#include "bsp.h"

volatile uint32_t adc_ticks = VBAT_MEAS_TIME;

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
	return *(uint64_t*)0x1ffff7e8;
}

void bsp_timing_irq(void)
{
	wk_usb_app_task();
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
	if(sweeptx || sweeprx)
	{
		sweepcnt--;
		if(sweepcnt == 0) 
		{
			sweepcnt = sweeptime;
			sweepflag = true;
		}
	}
	adc_ticks--;
	if(adc_ticks == VBAT_SAMPLE_TIME) gpio_bits_set(BATT_MEAS_GPIO_PORT,BATT_MEAS_PIN);
	if(adc_ticks == 0)
	{
		kick_adc();
		adc_ticks = VBAT_MEAS_TIME;
	}
	Keypad_Scan_ISR();
}


