
#include "stm32c0xx.h"
#include "tusb.h"

//static void USB_GPIO_Init(void)
//{
//    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

//    GPIOA->MODER &= ~(
//        GPIO_MODER_MODE11_Msk |
//        GPIO_MODER_MODE12_Msk);

//    GPIOA->MODER |=
//        (2 << GPIO_MODER_MODE11_Pos) |
//        (2 << GPIO_MODER_MODE12_Pos);

//    GPIOA->AFR[1] |=
//        (14 << ((11-8)*4)) |
//        (14 << ((12-8)*4));
//}

void USB_Init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_USBEN;

    NVIC_SetPriority(USB_DRD_FS_IRQn,2);
    NVIC_EnableIRQ(USB_DRD_FS_IRQn);

    tusb_init();
}


