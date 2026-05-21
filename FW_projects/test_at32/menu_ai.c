static void Action_Reset(void)
{
    /* perform reset */
}

static void Action_Save(void)
{
    /* save settings */
}

static const menu_item_t menu_settings[];
static const menu_item_t menu_system[];
static const menu_item_t menu_sensors[];

static const menu_item_t menu_root[] =
{
    {
        "Settings",
        NULL,
        menu_settings,
        4,
        NULL
    },

    {
        "System",
        NULL,
        menu_system,
        4,
        NULL
    },

    {
        "Sensors",
        NULL,
        menu_sensors,
        4,
        NULL
    },

    {
        "About",
        NULL,
        NULL,
        0,
        NULL
    }
};

static const menu_item_t menu_settings[] =
{
    {"Brightness", menu_root, NULL,0,NULL},
    {"Contrast",   menu_root, NULL,0,NULL},
    {"Volume",     menu_root, NULL,0,NULL},
    {"Language",   menu_root, NULL,0,NULL}
};

static const menu_item_t menu_system[] =
{
    {"Info",       menu_root, NULL,0,NULL},
    {"Reset",      menu_root, NULL,0,Action_Reset},
    {"Bootloader", menu_root, NULL,0,NULL},
    {"Save",       menu_root, NULL,0,Action_Save}
};

static const menu_item_t menu_sensors[] =
{
    {"Temperature", menu_root,NULL,0,NULL},
    {"Pressure",    menu_root,NULL,0,NULL},
    {"Voltage",     menu_root,NULL,0,NULL},
    {"Current",     menu_root,NULL,0,NULL}
};

static const menu_item_t* current_menu = menu_root;

static uint8_t current_index = 0;

static uint8_t current_count = 4;

static const menu_item_t* parent_menu = NULL;

void Menu_Init(void)
{
    current_menu = menu_root;
    current_index = 0;
    current_count = 4;
}

void Menu_KeyUp(void)
{
    if(current_index > 0)
        current_index--;
}

void Menu_KeyDown(void)
{
    if(current_index < (current_count - 1))
        current_index++;
}

void Menu_Enter(void)
{
    const menu_item_t* item =
        &current_menu[current_index];

    if(item->child_count)
    {
        parent_menu = current_menu;

        current_menu = item->children;

        current_count = item->child_count;

        current_index = 0;
    }
    else
    {
        if(item->callback)
            item->callback();
    }
}

void Menu_Back(void)
{
    if(current_menu == menu_root)
        return;

    current_menu = parent_menu;

    current_count = 4;

    current_index = 0;
}

void Menu_Home(void)
{
    current_menu = menu_root;

    current_count = 4;

    current_index = 0;
}

void Menu_Draw(void)
{
    SSD1306_Clear();

    for(uint8_t i=0; i<current_count; i++)
    {
        uint8_t y = i * 16;

        SSD1306_GotoXY(0,y);

        if(i == current_index)
            SSD1306_Puts(">", &Font_7x10, White);
        else
            SSD1306_Puts(" ", &Font_7x10, White);

        SSD1306_GotoXY(12,y);

        SSD1306_Puts(
            current_menu[i].text,
            &Font_7x10,
            White);
    }

    SSD1306_UpdateScreen();
}

void Process_Keyboard(void)
{
    keypad_key_t key = Keypad_GetKey();

    switch(key)
    {
        case KEY_2:
            Menu_KeyUp();
            break;

        case KEY_8:
            Menu_KeyDown();
            break;

        case KEY_6:
            Menu_Enter();
            break;

        case KEY_4:
            Menu_Back();
            break;

        case KEY_HASH:
            Menu_Home();
            break;

        default:
            break;
    }
}

int main(void)
{
    SSD1306_Init();

    Keypad_Init();

    Menu_Init();

    while(1)
    {
        Process_Keyboard();

        Menu_Draw();
    }
}

