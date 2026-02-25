/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_gpio.c
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
#include "wk_gpio.h"

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  init gpio_input/gpio_output/gpio_analog/eventout function.
  * @param  none
  * @retval none
  */
void wk_gpio_config(void)
{
  /* add user code begin gpio_config 0 */

  /* add user code end gpio_config 0 */

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin gpio_config 1 */

  /* add user code end gpio_config 1 */

  /* gpio input config */
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = RF_BUSY_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_init(RF_BUSY_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = C3_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(C3_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = C2_PIN | C1_PIN | C0_PIN | PTT_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(GPIOB, &gpio_init_struct);

  /* gpio output config */
  gpio_bits_set(RF_CS_GPIO_PORT, RF_CS_PIN);
  gpio_bits_reset(GPIOB, R3_PIN | R1_PIN | R0_PIN | OLED_DC_PIN | RED_PIN | 
                  GREEN_PIN);
  gpio_bits_set(GPIOB, R2_PIN | RF_RST_PIN | OLED_CS_PIN | OLED_RST_PIN);
  gpio_bits_reset(GPIOA, RF_EN_PIN | BATT_MEAS_PIN);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = RF_CS_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(RF_CS_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = R3_PIN | R2_PIN | R1_PIN | R0_PIN | RF_RST_PIN | 
                               OLED_CS_PIN | OLED_DC_PIN | OLED_RST_PIN | RED_PIN | GREEN_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = RF_EN_PIN | BATT_MEAS_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* add user code begin gpio_config 2 */

  /* add user code end gpio_config 2 */
}

/* add user code begin 1 */

/* add user code end 1 */
