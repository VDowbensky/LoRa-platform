/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd ("HDSC").
 *
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
 * BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
 *
 * This software contains source code for use with HDSC
 * components. This software is licensed by HDSC to be adapted only
 * for use in systems utilizing HDSC components. HDSC shall not be
 * responsible for misuse or illegal use of this software for devices not
 * supported herein. HDSC is providing this software "AS IS" and will
 * not be responsible for issues arising from incorrect user implementation
 * of the software.
 *
 * Disclaimer:
 * HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
 * REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
 * ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
 * WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
 * WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
 * WARRANTY OF NONINFRINGEMENT.
 * HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
 * NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
 * LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
 * INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
 * SAVINGS OR PROFITS,
 * EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
 * INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
 * FROM, THE SOFTWARE.
 *
 * This software may be replicated in part or whole for the licensed use,
 * with the restriction that this Disclaimer and Copyright notice must be
 * included with each copy of this software, whether used in part or whole,
 * at all times.
 */
/******************************************************************************/
/** \file usb_bsp_template.c
 **
 ** \brief  This file is responsible to offer board support package and is
 **         configurable by user.
 **
 **   - 2019-06-28  lsq   First version for USB mouse demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_bsp.h"
#include "hc32l176.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
#define RCH_CR_TRIM_24M_VAL         (*((volatile uint16_t*) (0x00100C00ul)))
#define RCH_CR_TRIM_22_12M_VAL      (*((volatile uint16_t*) (0x00100C02ul)))
#define RCH_CR_TRIM_16M_VAL         (*((volatile uint16_t*) (0x00100C04ul)))
#define RCH_CR_TRIM_8M_VAL          (*((volatile uint16_t*) (0x00100C06ul)))
#define RCH_CR_TRIM_4M_VAL          (*((volatile uint16_t*) (0x00100C08ul)))

#define RCL_CR_TRIM_38_4K_VAL       (*((volatile uint16_t*) (0x00100C20ul)))
#define RCL_CR_TRIM_32_8K_VAL       (*((volatile uint16_t*) (0x00100C22ul)))
/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
extern  USB_OTG_CORE_HANDLE      USB_OTG_dev;
extern  uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

/**
 ******************************************************************************
 ** \brief  Usb interrupt handle
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/
void USBFS_IRQHandler(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

/**
 ******************************************************************************
  * \brief  use systick to delay
  *
  * \param  u32Cnt: the number of ms
  * \retval None
 ******************************************************************************/
void delay1ms(uint32_t u32Cnt)
{
    uint32_t u32end;
    
    SysTick->LOAD = 0xFFFFFF;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    
    while(u32Cnt-- > 0)
    {
        SysTick->VAL  = 0;
        u32end = 0x1000000 - 4000000/1000;
        while(SysTick->VAL > u32end)
        {
            ;
        }
    }
    
    SysTick->CTRL = (SysTick->CTRL & (~SysTick_CTRL_ENABLE_Msk));
}

/**
 ******************************************************************************
  * \brief  USB_OTG_BSP_Init
  *         Initilizes BSP configurations
  *
  * \param  None
  * \retval None
 ******************************************************************************/
void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{	
    //打开GPIO、FLASH外设时钟
    M0P_SYSCTRL->PERI_CLKEN0_f.GPIO  = 1;
    M0P_SYSCTRL->PERI_CLKEN0_f.FLASH = 1;       
    //FLASH WAIT CYCLE 
    M0P_FLASH->BYPASS_f.BYSEQ = 0x5A5A;
    M0P_FLASH->BYPASS_f.BYSEQ = 0xA5A5;
    M0P_FLASH->CR_f.WAIT = 1;             //等待2个周期   
    //XTH IN  设置接XHT晶振的引脚为模拟口    
    M0P_GPIO->PFADS_f.PF00 = 1;
    M0P_GPIO->PFADS_f.PF01 = 1; 
    //启动XTH
    M0P_SYSCTRL->XTH_CR_f.STARTUP    = 3;//设置外部高速时钟XTH稳定时间最长，为16384个周期
    M0P_SYSCTRL->XTH_CR_f.XTH_FSEL   = 1;//外部晶振工作频率为8MHz，属于6M-12M范围
    M0P_SYSCTRL->XTH_CR_f.DRIVER     = 3;//外部晶振驱动能力选择最强驱动能力
    M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
    M0P_SYSCTRL->SYSCTRL0_f.XTH_EN   = 1;//使能外部高速进制XTH   
    while(1 != M0P_SYSCTRL->XTH_CR_f.STABLE); //等待外部高速晶振XTH稳定 
    delay1ms(10);
    //FLASH WAIT CYCLE 
    M0P_FLASH->BYPASS_f.BYSEQ = 0x5A5A;
    M0P_FLASH->BYPASS_f.BYSEQ = 0xA5A5;
    M0P_FLASH->CR_f.WAIT = 1;             //等待2个周期     
    M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
    M0P_SYSCTRL->SYSCTRL0_f.CLKSW = 1;        //切换到外部XHT时钟   
    //将8MHz的XTH倍频成48MHz
    M0P_SYSCTRL->PLL_CR_f.STARTUP = 7;        //设置PLL稳定时间为最长：16384个PLL周期
    M0P_SYSCTRL->PLL_CR_f.FRSEL   = 1;        //PLL的输入频率为8MHz，因此选择01：6M-12M
    M0P_SYSCTRL->PLL_CR_f.DIVN    = 6;        //PLL的倍频系数
    M0P_SYSCTRL->PLL_CR_f.FOSC    = 4;        //倍频以后的频率为48MHz，因此该值设置为1xx: 36M-48M
    M0P_SYSCTRL->PLL_CR_f.REFSEL  = 2;        //输入时钟为XTH晶振生产的时钟 x0
    M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
    M0P_SYSCTRL->SYSCTRL0_f.PLL_EN = 1;       //使能PLL
    while(1 != M0P_SYSCTRL->PLL_CR_f.STABLE); //等待PLL稳定   
    delay1ms(100);
    //FLASH WAIT CYCLE 
    M0P_FLASH->BYPASS_f.BYSEQ = 0x5A5A;
    M0P_FLASH->BYPASS_f.BYSEQ = 0xA5A5;
    M0P_FLASH->CR_f.WAIT = 1;                 //等待2个周期    
    M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
    M0P_SYSCTRL->SYSCTRL0_f.CLKSW = 4;        //切换到PLL
    
    M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.RCH_EN = 0;

    M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.PCLK_PRS = 0;     //SystemClk时钟来源选择内部PLL

    M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.HCLK_PRS = 0;     //HCLK时钟源选择SystemClk
   
    //使用倍频后的输出频率作为USB的时钟
    M0P_SYSCTRL->SYSCTRL2_f.SYSCTRL2 = 0x5A5A;
    M0P_SYSCTRL->SYSCTRL2_f.SYSCTRL2 = 0xA5A5;    
    M0P_SYSCTRL->SYSCTRL1 |= 1<<13;         //USB时钟选择PLL倍频后的时钟
   
    M0P_SYSCTRL->PERI_CLKEN1_f.USB = 1;       //使能USB时钟模块
    M0P_RESET->PERI_RESET1_f.USB = 0;         //是USB进入复位状态
    M0P_RESET->PERI_RESET1_f.USB = 1;         //是USB进入正常工作状态
}

/**
  * \brief  USB_OTG_BSP_EnableInterrupt
  *         Enabele USB Global interrupt
  * \param  None
  * \retval None
  */
void USB_OTG_BSP_EnableInterrupt(void)
{
    NVIC_ClearPendingIRQ(USBFS_IRQn);
    NVIC_SetPriority(USBFS_IRQn, 3);
    NVIC_EnableIRQ(USBFS_IRQn);
}

/**
  * \brief  BSP_Drive_VBUS
  *         Drives the Vbus signal through IO
  * \param  speed : Full, Low
  * \param  state : VBUS states
  * \retval None
  */

void USB_OTG_BSP_DriveVBUS(uint32_t speed, uint8_t state)
{

}

/**
  * \brief  USB_OTG_BSP_ConfigVBUS
  *         Configures the IO for the Vbus and OverCurrent
  * \param  Speed : Full, Low
  * \retval None
  */

void  USB_OTG_BSP_ConfigVBUS(uint32_t speed)
{

}

/**
  * \brief  USB_OTG_BSP_TimeInit
  *         Initialises delay unit Systick timer /Timer2
  * \param  None
  * \retval None
  */
void USB_OTG_BSP_TimeInit ( void )
{

}

/**
  * \brief  USB_OTG_BSP_uDelay
  *         This function provides delay time in micro sec
  * \param  usec : Value of delay required in micro sec
  * \retval None
  */
void USB_OTG_BSP_uDelay(const uint32_t usec)
{
  uint32_t count = usec;
  do
  {
    if (--count == 0)
    {
      return;
    }
  }
  while (1);
}
/**
  * \brief  USB_OTG_BSP_mDelay
  *          This function provides delay time in milli sec
  * \param  msec : Value of delay required in milli sec
  * \retval None
  */
void USB_OTG_BSP_mDelay (const uint32_t msec)
{
    USB_OTG_BSP_uDelay(msec * 1000);
}


/**
  * \brief  USB_OTG_BSP_TimerIRQ
  *         Time base IRQ
  * \param  None
  * \retval None
  */

void USB_OTG_BSP_TimerIRQ (void)
{

}
