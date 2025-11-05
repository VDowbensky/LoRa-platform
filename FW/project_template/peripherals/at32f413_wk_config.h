/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f413_wk_config.h
  * @brief    header file of work bench config
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

/* define to prevent recursive inclusion -----------------------------------*/
#ifndef __AT32F413_WK_CONFIG_H
#define __AT32F413_WK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------------*/
#include "stdio.h"
#include "at32f413.h"

/* private includes -------------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* exported types -------------------------------------------------------------*/
/* add user code begin exported types */

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* add user code begin dma define */
/* user can only modify the dma define value */
//#define DMA1_CHANNEL1_BUFFER_SIZE   0
//#define DMA1_CHANNEL1_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL1_PERIPHERAL_BASE_ADDR  0

//#define DMA1_CHANNEL2_BUFFER_SIZE   0
//#define DMA1_CHANNEL2_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL2_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL3_BUFFER_SIZE   0
//#define DMA1_CHANNEL3_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL3_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL4_BUFFER_SIZE   0
//#define DMA1_CHANNEL4_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL4_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL5_BUFFER_SIZE   0
//#define DMA1_CHANNEL5_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL5_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL6_BUFFER_SIZE   0
//#define DMA1_CHANNEL6_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL6_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL7_BUFFER_SIZE   0
//#define DMA1_CHANNEL7_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL7_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL1_BUFFER_SIZE   0
//#define DMA2_CHANNEL1_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL1_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL2_BUFFER_SIZE   0
//#define DMA2_CHANNEL2_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL2_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL3_BUFFER_SIZE   0
//#define DMA2_CHANNEL3_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL3_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL4_BUFFER_SIZE   0
//#define DMA2_CHANNEL4_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL4_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL5_BUFFER_SIZE   0
//#define DMA2_CHANNEL5_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL5_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL6_BUFFER_SIZE   0
//#define DMA2_CHANNEL6_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL6_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL7_BUFFER_SIZE   0
//#define DMA2_CHANNEL7_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL7_PERIPHERAL_BASE_ADDR   0
/* add user code end dma define */

/* Private defines -------------------------------------------------------------*/
#define VBATT_PIN    GPIO_PINS_0
#define VBATT_GPIO_PORT    GPIOA
#define RF_BUSY_PIN    GPIO_PINS_2
#define RF_BUSY_GPIO_PORT    GPIOA
#define RF_CS_PIN    GPIO_PINS_3
#define RF_CS_GPIO_PORT    GPIOA
#define SCK1_PIN    GPIO_PINS_5
#define SCK1_GPIO_PORT    GPIOA
#define MISO1_PIN    GPIO_PINS_6
#define MISO1_GPIO_PORT    GPIOA
#define MOSI1_PIN    GPIO_PINS_7
#define MOSI1_GPIO_PORT    GPIOA
#define RF_INT_PIN    GPIO_PINS_0
#define RF_INT_GPIO_PORT    GPIOB
#define C3_PIN    GPIO_PINS_1
#define C3_GPIO_PORT    GPIOB
#define C2_PIN    GPIO_PINS_2
#define C2_GPIO_PORT    GPIOB
#define C1_PIN    GPIO_PINS_10
#define C1_GPIO_PORT    GPIOB
#define C0_PIN    GPIO_PINS_11
#define C0_GPIO_PORT    GPIOB
#define R3_PIN    GPIO_PINS_12
#define R3_GPIO_PORT    GPIOB
#define R2_PIN    GPIO_PINS_13
#define R2_GPIO_PORT    GPIOB
#define R1_PIN    GPIO_PINS_14
#define R1_GPIO_PORT    GPIOB
#define R0_PIN    GPIO_PINS_15
#define R0_GPIO_PORT    GPIOB
#define RF_EN_PIN    GPIO_PINS_8
#define RF_EN_GPIO_PORT    GPIOA
#define TXD1_PIN    GPIO_PINS_9
#define TXD1_GPIO_PORT    GPIOA
#define RXD1_PIN    GPIO_PINS_10
#define RXD1_GPIO_PORT    GPIOA
#define BATT_MEAS_PIN    GPIO_PINS_15
#define BATT_MEAS_GPIO_PORT    GPIOA
#define PTT_PIN    GPIO_PINS_3
#define PTT_GPIO_PORT    GPIOB
#define RF_RST_PIN    GPIO_PINS_4
#define RF_RST_GPIO_PORT    GPIOB
#define OLED_CS_PIN    GPIO_PINS_5
#define OLED_CS_GPIO_PORT    GPIOB
#define OLED_DC_PIN    GPIO_PINS_6
#define OLED_DC_GPIO_PORT    GPIOB
#define OLED_RST_PIN    GPIO_PINS_7
#define OLED_RST_GPIO_PORT    GPIOB
#define RED_PIN    GPIO_PINS_9
#define RED_GPIO_PORT    GPIOB
#define GREEN_PIN    GPIO_PINS_8
#define GREEN_GPIO_PORT    GPIOB

/* exported functions ------------------------------------------------------- */
  /* system clock config. */
  void wk_system_clock_config(void);

  /* config periph clock. */
  void wk_periph_clock_config(void);

  /* nvic config. */
  void wk_nvic_config(void);

/* add user code begin exported functions */

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
