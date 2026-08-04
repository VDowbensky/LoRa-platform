#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "bsp.h"

#define ENC_FIFO_SIZE    8
#define ENC_COUNT					5

typedef enum
{
	ENC_EVENT_NONE = 0,
	ENC_EVENT_CW,
	ENC_EVENT_CCW,
	ENC_EVENT_BUTTON_DOWN,
	ENC_EVENT_BUTTON_UP,
	ENC_EVENT_CLICK,
	ENC_EVENT_DOUBLE_CLICK,
	ENC_EVENT_LONG_PRESS,
	ENC_EVENT_REPEAT
} encoder_event_t;

void Encoder_Init(void);
void Encoder_Task(void);          /* called every 5 ms */
encoder_event_t Encoder_GetEvent(void);

void encoder_proc(void);

#endif