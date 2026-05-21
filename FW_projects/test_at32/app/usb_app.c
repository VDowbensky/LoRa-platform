/* add user code begin Header */
/**
  **************************************************************************
  * @file     usbd_app.c
  * @brief    usb device app
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

#include "usb_conf.h"
#include "wk_system.h"

#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include <string.h>
#include "retarget.h"
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

extern volatile int rxCount;
extern volatile int txCount;

uint16_t ov_cnt = 0;


/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

usbd_core_type usb_core_dev;
uint32_t usbd_app_buffer_fs1[1024];



/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  usb application initialization
  * @param  none
  * @retval none
  */
void wk_usb_app_init(void)
{
  /* add user code begin usb_app_init 0 */
	usbd_disconnect(&usb_core_dev);
  /* add user code end usb_app_init 0 */

  /*fs1 device cdc*/
  usbd_core_init(&usb_core_dev, USB, &cdc_class_handler, &cdc_desc_handler, 0);

  /* enable usb pull-up */
  usbd_connect(&usb_core_dev);

  /* add user code begin usb_app_init 1 */

  /* add user code end usb_app_init 1 */
}

/**
  * @brief  usb application task
  * @param  none
  * @retval none
  */
void wk_usb_app_task(void)
{
  /* add user code begin usb_app_task 0 */

  /* add user code end usb_app_task 0 */

  uint32_t length = 0;
	uint32_t timeout;
  static uint8_t send_zero_packet = 0;

  /* add user code begin usb_app_task 1 */

  /* add user code end usb_app_task 1 */

  /* fs1 device cdc */
  length = usb_vcp_get_rxdata(&usb_core_dev, (uint8_t *)usbd_app_buffer_fs1);
	if(length > 0) 
	{
		//send_zero_packet = 1;
		memcpy((void*)rxBuffer,(void*)usbd_app_buffer_fs1,length);
		rxCount = length;
	}
	
	/* if data in buffer present,usb send data to host */
  if(usb_tx_read_index == usb_tx_write_index) 
	{
		txCount = 0;
//		if (send_zero_packet == 1)
//		{
//			//SetEPTxCount(ENDP1, 0);
//			//SetEPTxValid(ENDP1);
//			send_zero_packet = 0;
//		}
//		return;
	}
  else
  {
    /* whether to process the fifo overflow or not */
    if(usb_tx_write_index > usb_tx_read_index) txCount = usb_tx_write_index - usb_tx_read_index;
    else if(usb_tx_write_index == 0 && usb_tx_write_index != usb_tx_read_index) txCount = usb_tx_write_index - usb_tx_read_index;
    else txCount = (usb_tx_write_index-1) + usb_tx_write_index - usb_tx_read_index;
  }
	if(txCount > 0 || send_zero_packet == 1)
	{
		if(txCount > 0) send_zero_packet = 1;
		if(txCount == 0) 
		{
			send_zero_packet = 0;
			usb_tx_write_index = 0;
			usb_tx_read_index = 0;
		}
		timeout = 50000;
		if((usb_tx_read_index + txCount) < TXBUFSIZE)
		{
			do
			{
				/* send data to host */
				if(usb_vcp_send_data(&usb_core_dev, &txBuffer[usb_tx_read_index], txCount) == SUCCESS)
				{
					usb_tx_read_index = usb_tx_read_index + txCount;
					if(send_zero_packet == 1)	usb_vcp_send_data(&usb_core_dev, NULL, 0);
					break;
				}
			}while(timeout --);
		}
		/* process the fifo overflow */
		else
		{
			do
			{
				/* send data to host */
				if(usb_vcp_send_data(&usb_core_dev, &txBuffer[usb_tx_read_index], TXBUFSIZE - usb_tx_read_index) == SUCCESS)
				{
					/* get fifo overflow data count */
					ov_cnt = txCount - (TXBUFSIZE - usb_tx_read_index);
					usb_tx_read_index = 0;
					if(send_zero_packet == 1) usb_vcp_send_data(&usb_core_dev, NULL, 0);
					break;
				}
			}while(timeout --);
			timeout = 50000;
			do
			{
				/* send data to host */
				if(usb_vcp_send_data(&usb_core_dev, &txBuffer[usb_tx_read_index], ov_cnt) == SUCCESS)
				{
					usb_tx_read_index = ov_cnt;
					if(send_zero_packet == 1)	usb_vcp_send_data(&usb_core_dev, NULL, 0);
					break;
				}
			}while(timeout --);
		}
	}
  /* add user code begin usb_app_task 2 */

  /* add user code end usb_app_task 2 */
}

/**
  * @brief  usb interrupt handler
  * @param  none
  * @retval none
  */
void wk_usbfs_irq_handler(void)
{
  /* add user code begin otgfs1_irq_handler 0 */

  /* add user code end otgfs1_irq_handler 0 */

  usbd_irq_handler(&usb_core_dev);

  /* add user code begin otgfs1_irq_handler 1 */
	wk_usb_app_task();
  /* add user code end otgfs1_irq_handler 1 */
}

/**
  * @brief  usb delay function
  * @param  ms: delay number of milliseconds.
  * @retval none
  */
void usb_delay_ms(uint32_t ms)
{
  /* add user code begin delay_ms 0 */

  /* add user code end delay_ms 0 */

  wk_delay_ms(ms);

  /* add user code begin delay_ms 1 */

  /* add user code end delay_ms 1*/
}

/* add user code begin 1 */

/* add user code end 1 */
