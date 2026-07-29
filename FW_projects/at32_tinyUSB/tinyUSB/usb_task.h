#ifndef _USB_TASK_H_
#define _USB_TASK_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "bsp.h"
#include "tusb.h"
#include "ring_buffer.h"

#define TUD_RXBUFSIZE		512

void tud_mount_cb(void);
void tud_umount_cb(void);
void usb_task(void);

void cdc_print(uint8_t itf,char* str);

extern uint8_t tud_rx_buffer[];
extern volatile uint32_t tud_rxcount;

#endif
