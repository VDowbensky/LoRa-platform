/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
#include <stdint.h>
#include "stm32wlxx_it.h"
#include "bsp.h"
//#include "beeper.h"

//volatile uint32_t delay_ticks;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radio_proc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
//extern CRYP_HandleTypeDef hcryp;
//extern SUBGHZ_HandleTypeDef hsubghz;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
	
volatile uint32_t uwTick;

void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
//	adc_ticks++;
//	if(adc_ticks == ADC_MS)
//	{
//		adc_ticks = 0;
//		adc_start(true);
//	}
	timing_irq_process();
}

/******************************************************************************/
/* STM32WLxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wlxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC Interrupt.
  */
//void ADC_IRQHandler(void)
//{
//  /* USER CODE BEGIN ADC_IRQn 0 */

//  /* USER CODE END ADC_IRQn 0 */
//  /* USER CODE BEGIN ADC_IRQn 1 */

//  /* USER CODE END ADC_IRQn 1 */
//}

/**
  * @brief This function handles TIM1 Update Interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 Global Interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	tim2_irq_process();
  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 Interrupt.
  */
//void USART1_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART1_IRQn 0 */

//  /* USER CODE END USART1_IRQn 0 */
//  /* USER CODE BEGIN USART1_IRQn 1 */

//  /* USER CODE END USART1_IRQn 1 */
//}

/**
  * @brief This function handles HSEM Interrupt.
  */
void HSEM_IRQHandler(void)
{
  /* USER CODE BEGIN HSEM_IRQn 0 */

  /* USER CODE END HSEM_IRQn 0 */
  /* USER CODE BEGIN HSEM_IRQn 1 */

  /* USER CODE END HSEM_IRQn 1 */
}

/**
  * @brief This function handles SUBGHZ Radio Interrupt.
  */
void SUBGHZ_Radio_IRQHandler(void)
{
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 0 */

  /* USER CODE END SUBGHZ_Radio_IRQn 0 */
  //HAL_SUBGHZ_IRQHandler(&hsubghz);
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 1 */
	radio_irq_handler();
  /* USER CODE END SUBGHZ_Radio_IRQn 1 */
}

/**
  * @brief This function handles AES Interrupt.
  */
void AES_IRQHandler(void)
{
  /* USER CODE BEGIN AES_IRQn 0 */

  /* USER CODE END AES_IRQn 0 */
  //HAL_CRYP_IRQHandler(&hcryp);
  /* USER CODE BEGIN AES_IRQn 1 */

  /* USER CODE END AES_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
