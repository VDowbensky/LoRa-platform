/**************************************************************************//**
 * @file
 * @brief USART1 prototypes and definitions
 * @version 4.4.0
 ******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


#ifndef __RETARGET_H
#define __RETARGET_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "bsp.h"
#include "cdc_class.h"
//#include "usb_lib.h"
//#include "usb_desc.h"
//#include "usb_pwr.h"

/***************************************************************************//**
 * @addtogroup kitdrv
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup RetargetIo
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__CROSSWORKS_ARM)
int __putchar(int ch);
int __getchar(void);
#endif

#define RXBUFSIZE    		1024 /**< Buffer size for RX */
#define TXBUFSIZE				1024

int  RETARGET_ReadChar(void);
int  RETARGET_WriteChar(char c);

void RETARGET_CrLf(int on);
void RETARGET_Init(void);

extern volatile uint8_t rxBuffer[];
extern volatile uint8_t txBuffer[];
extern volatile int rxCount;
extern volatile int txCount;

#ifdef __cplusplus
}
#endif

/** @} (end group RetargetIo) */
/** @} (end group Drivers) */

#endif
