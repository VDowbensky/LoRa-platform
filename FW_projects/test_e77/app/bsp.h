#ifndef _BSP_H_
#define _BSP_H_

#include <stdio.h>
#include <string.h>
#include "system_stm32wlxx.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_system.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_ll_exti.h"
#include "stm32wlxx_ll_usart.h"
#include "stm32wlxx_ll_spi.h"
#include "stm32wlxx_ll_adc.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_tim.h"
#include "stm32wlxx_ll_utils.h"
#include "retarget.h"
#include "delay.h"
#include "subghz.h"
#include "radio_proc.h"

#define HW_VERSION    1
#define FW_VERSION		0
#define FW_REVISION		1

#define WORK_UART  		2

#define TXD0_PORT						GPIOA
#define TXD0_PIN						LL_GPIO_PIN_9
#define RXD0_PORT						GPIOA
#define RXD0_PIN						LL_GPIO_PIN_10

#define TXD1_PORT						GPIOA
#define TXD1_PIN						LL_GPIO_PIN_2
#define RXD1_PORT						GPIOA
#define RXD1_PIN						LL_GPIO_PIN_3

//#define SCL0_PORT						GPIOA
//#define SCL0_PIN						LL_GPIO_PIN_14
//#define SDA0_PORT						GPIOA
//#define SDA0_PIN						LL_GPIO_PIN_15


//Keys
//#define K_UP_PORT						GPIOB
//#define K_UP_PIN						LL_GPIO_PIN_2
//#define K_DOWN_PORT					GPIOB
//#define K_DOWN_PIN					LL_GPIO_PIN_12
//#define K_ENTER_PORT				GPIOC
//#define K_ENTER_PIN					LL_GPIO_PIN_13

//#define K_NONE							255
//#define K_UP								0
//#define K_DOWN							1
//#define K_ENTER							2
//#define KEY_COUNT	          3

//BATT measuring switch
#define VBATT_MEAS_PORT			GPIOA
#define VBATT_MEAS_PIN			LL_GPIO_PIN_2
#define VBATT_PORT					GPIOA
#define VBATT_PIN						LL_GPIO_PIN_15
#define VBATT_ADC_CHANNEL		ADC_SAMPLE_CHAN_1//PA11

//LED
#define LED_PORT						GPIOB
#define LED_PIN							LL_GPIO_PIN_4

//Buzzer
//#define BUZZER_PORT					GPIOB
//#define BUZZER_PIN					LL_GPIO_PIN_8

//Main RF switch
#define RFSW_TX_GPIO 				LL_AHB2_GRP1_PERIPH_GPIOA //to BSP.h
#define RFSW_RX_GPIO 				LL_AHB2_GRP1_PERIPH_GPIOA
#define RFSW_RX_PORT				GPIOA //RX
#define RFSW_RX_PIN					LL_GPIO_PIN_7
#define RFSW_TX_PORT				GPIOA //TX
#define RFSW_TX_PIN					LL_GPIO_PIN_6

#define E77M915M30					0

//#define RX_RESTART_INTERVAL	120000 //2 min
#define RX_RESTART_INTERVAL	60000 //1 min

//AUX RF power
//#define EXTRF_POWER_PORT		GPIOA
//#define EXTRF_POWER_PIN			LL_GPIO_PIN_0


//LR1121
//#define LR112X_SPI					AUX_SPI
//#define LR112X_NSS_PORT			GPIOA
//#define LR112X_NSS_PIN			LL_GPIO_PIN_4
//#define LR112X_RESET_PORT		GPIOA
//#define LR112X_RESET_PIN		LL_GPIO_PIN_3
//#define LR112X_BUSY_PORT		GPIOA
//#define LR112X_BUSY_PIN			LL_GPIO_PIN_5
//#define LR112X_INT_PORT			GPIOA
//#define LR112X_INT_PIN			LL_GPIO_PIN_1

//#define LR112X_SCK_PORT 		AUX_SCK_PORT 
//#define LR112X_SCK_PIN			AUX_SCK_PIN
//#define LR112X_MOSI_PORT		AUX_MOSI_PORT	
//#define LR112X_MOSI_PIN			AUX_MOSI_PIN
//#define LR112X_MISO_PORT		AUX_MISO_PORT
//#define LR112X_MISO_PIN			AUX_MISO_PIN


//#define SSD1306_SPI					AUX_SPI	
//#define SSD1306_RCC_PERIPHERAL RCC_PERIPHERAL_SSP2
//#define SSD1306_SCK_PORT    AUX_SCK_PORT
//#define SSD1306_SCK_PIN			AUX_SCK_PIN
//#define SSD1306_MOSI_PORT		AUX_MOSI_PORT
//#define SSD1306_MOSI_PIN		AUX_MOSI_PIN
//#define SSD1306_RST_PORT		GPIOB
//#define SSD1306_RST_PIN			LL_GPIO_PIN_7
//#define SSD1306_DC_PORT			GPIOA
//#define SSD1306_DC_PIN			LL_GPIO_PIN_11
//#define SSD1306_CS_PORT			GPIOB
//#define SSD1306_CS_PIN			LL_GPIO_PIN_6

#define UART0_BR						115200
#define UART1_BR						115200

void init_power_clk(void);
void init_peripherals(void);
void init_radio_specific(void);

void bsp_reset_proc(void);

void led_on(void);
void led_off(void);
void timing_irq_process(void);

extern volatile bool RxRestartFlag;

#endif
