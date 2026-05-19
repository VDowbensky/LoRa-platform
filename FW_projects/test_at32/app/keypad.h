#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#include "bsp.h"

#define K_0			0x30
#define K_1			0x31
#define K_2			0x32
#define K_3			0x33
#define K_4			0x34
#define K_5			0x35
#define K_6			0x36
#define K_7			0x37
#define K_8			0x38
#define K_9			0x39
#define K_A			0x3a
#define K_B			0x3b
#define K_C			0x3c
#define K_D			0x3d
#define K_AST		0x3e
#define K_SRP		0x3f
#define K_NONE	-1

void keypad_init(void);
void keypad_scan(void);
void keypad_proc(void);
extern int16_t Key;

#endif
