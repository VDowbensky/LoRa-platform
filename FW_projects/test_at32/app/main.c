/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32f413_wk_config.h"
#include "wk_adc.h"
#include "wk_debug.h"
#include "wk_exint.h"
#include "wk_rtc.h"
#include "wk_spi.h"
#include "wk_usart.h"
#include "wk_usbfs.h"
#include "wk_wdt.h"
#include "wk_gpio.h"
#include "usb_app.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "app_cli.h"
#include "radio_proc.h"
#include "flash.h"
#include "bsp.h"
#include "menu.h"

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* init debug function. */
  wk_debug_config();

  /* nvic config. */
  wk_nvic_config();

  /* timebase config. */
  wk_timebase_init();

  /* init gpio function. */
  wk_gpio_config();

  /* init adc1 function. */
  wk_adc1_init();

  /* init usart1 function. */
  wk_usart1_init();

  /* init spi1 function. */
  wk_spi1_init();

  /* init rtc function. */
  wk_rtc_init();

  /* init exint function. */
  wk_exint_config();

  /* init usbfs function. */
  wk_usbfs_init();

  /* init wdt function. */
  wk_wdt_init();

  /* init usb app function. */
  wk_usb_app_init();

  /* add user code begin 2 */
	RETARGET_Init();
	cli_init();
	txled_on();
	rxled_on();
	delay_ms(3000); //for USB enumeration
	Keypad_Init(); 
#if OLED_ENABLED
	SSD1306_Init();
	SSD1306_Clear(0);
#endif
	printf("\r\nRF test board\r\n");
	printf("HW=%d,FW=%d.%d\r\n", HW_VERSION, FW_VERSION, FW_REVISION);
	readconfig();
	if(radioconfig.chip == 0xff) 
	{
		printf("Config missing. Init...\r\n");
		if(radio_initconfig(1262,0) == RADIO_OK) 
		{
			writeconfig();
			printf("Init config OK.\r\n");
		}
		else printf("Init config error.\r\n");
	}
	txled_off();
	radio_power_on();
	delay_ms(100);
	int8_t err = radio_init();
	printf("Radio init: ");
	if(err == RADIO_OK) printf("OK\r\n");
	else printerror(err);
	printf("Radio chip: %d\r\n",radioconfig.chip);
	rxled_off();
	//radio_rx();
  /* add user code end 2 */
#if OLED_ENABLED
	display_main_screen();
#endif
  while(1)
  {
    /* add user code begin 3 */
		radio_proc();
		cli_proc();
		if(SecFlag)
		{
			SecFlag = false;
			display_status();
			if(workmode == WORK_MODE_JAMMER) display_jam_freq(currfreq);
		}
		char key = Keypad_GetKey();
		//if(key != KEY_NONE) printf("%c\r\n",key);
		//if(key == '#') menu_proc();
		switch(key)
		{
			case '#':
			{
				if(workmode == WORK_MODE_PACKET)
				{
					menu_proc();
				}
			}
			break;
			
			case 'A': //send one packet
			case '@':
			{
				if(workmode == WORK_MODE_PACKET)
				{
					tx_request = true;
				}
				break;
			}
			
			case 'B': //send burst
			//case '@':
			{
				if(workmode == WORK_MODE_PACKET)
				{
					txpacketcount = radioconfig.pkt_count;
					inter_packet_delay = radioconfig.txsendinterval;
					//slave_id = 0xffffffff; //temp.
					radio_startburst();
				}
				break;
			}

			case 'C': //scanner
			{
				int8_t err;
				if(workmode == WORK_MODE_PACKET)
				{
					err = radio_rxscan(radioconfig.rxstartfreq,radioconfig.rxstopfreq,radioconfig.rxstep,radioconfig.rxinterval,radioconfig.rssitr);
					if(err == RADIO_OK)
					{
						rxled_on();
						workmode = WORK_MODE_SCANNER;
						//printf("START\r\n");
					}
				}
				break;
			}
			
			case 'D': //jammer
			{
				int8_t err;
				if(workmode == WORK_MODE_PACKET)
				{
					err = radio_txsweep(radioconfig.txstartfreq,radioconfig.txstopfreq,radioconfig.txstep,radioconfig.txinterval,radioconfig.txmodulation);
					if(err == RADIO_OK)
					{
						txled_on();
						workmode = WORK_MODE_JAMMER;
						//printf("START\r\n");
					}
				}
				break;
			}
			
			case '*':
			{
				if(workmode == WORK_MODE_SCANNER)
				{
					err = radio_rxscan(radioconfig.rxstartfreq,radioconfig.rxstopfreq,radioconfig.rxstep,radioconfig.rxinterval,0.0);
					if(err == RADIO_OK)
					{
						rxled_off();
						workmode = WORK_MODE_PACKET;
						//printf("STOP\r\n");
					}
				}
				if(workmode == WORK_MODE_JAMMER)
				{
					err = radio_txsweep(radioconfig.txstartfreq,radioconfig.txstopfreq,radioconfig.txstep,radioconfig.txinterval,0);
					if(err == RADIO_OK)
					{
						rxled_off();
						workmode = WORK_MODE_PACKET;
						//printf("STOP\r\n");
					}
				}
				if(workmode == WORK_MODE_PACKET)	
				{
					if(master)
					{
						master = false;
						tx_request = false;
					}
				}					
			}
			
			default:
			break;
		}
    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

/* add user code end 4 */
