#ifndef MENU_H
#define MENU_H

#include <stdint.h>

typedef struct menu_item_s menu_item_t;

typedef void (*menu_callback_t)(void);

struct menu_item_s
{
    const char* text;

    const menu_item_t* parent;

    const menu_item_t* children;

    uint8_t child_count;

    menu_callback_t callback;
};

void Menu_Init(void);

void Menu_KeyUp(void);
void Menu_KeyDown(void);
void Menu_Enter(void);
void Menu_Back(void);
void Menu_Home(void);

void Menu_Draw(void);

#endif