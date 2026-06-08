#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#include "bsp.h"
#include <stdint.h>


/* ---------- COLUMN PINS ---------- */

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4



typedef enum
{
    KEY_NONE = 0,

    KEY_1,
    KEY_2,
    KEY_3,
    KEY_A,

    KEY_4,
    KEY_5,
    KEY_6,
    KEY_B,

    KEY_7,
    KEY_8,
    KEY_9,
    KEY_C,

    KEY_STAR,
    KEY_0,
    KEY_HASH,
    KEY_D

} keypad_key_t;

void Keypad_Init(void);
/* call from SysTick ISR */
void Keypad_Scan_ISR(void);
char Keypad_GetKey(void);

extern bool PttFlag;

#endif
