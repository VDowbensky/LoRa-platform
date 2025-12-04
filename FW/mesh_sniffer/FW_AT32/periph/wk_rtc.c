/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_rtc.c
  * @brief    work bench config program
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
#include "wk_rtc.h"

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  init rtc function.
  * @param  none
  * @retval none
  */
void wk_rtc_init(void)
{
  /* add user code begin rtc_init 0 */
  
  /* add user code end rtc_init 0 */

  calendar_type time_struct;

  /* add user code begin rtc_init 1 */

  /* add user code end rtc_init 1 */

  pwc_battery_powered_domain_access(TRUE);

  crm_rtc_clock_select(CRM_RTC_CLOCK_LEXT);
  crm_rtc_clock_enable(TRUE);
  rtc_wait_update_finish();
  rtc_wait_config_finish();
  rtc_divider_set(32767);
  rtc_wait_config_finish();

  time_struct.year  = 2025;
  time_struct.month = 9;
  time_struct.date  = 20;
  time_struct.hour  = 0;
  time_struct.min   = 0;
  time_struct.sec   = 0;
  rtc_time_set(&time_struct);

  bpr_rtc_output_select(BPR_RTC_OUTPUT_NONE);

  /* add user code begin rtc_init 2 */

  /* add user code end rtc_init 2 */
}

/* add user code begin 1 */

/* add user code end 1 */
