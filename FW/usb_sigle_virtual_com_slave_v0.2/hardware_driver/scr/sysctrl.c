
#include "sysctrl.h"

/**
 ******************************************************************************
 ** \brief  打开外部高速晶振输入，再通过PLL倍频成48MHz作为主频
 ** \brief  需要修改输出频率，可以通过修改“PLL输入时钟与输出时钟的倍频关系”参数，即通过PLL_CR进行修改
 **
 ** \param  none
 ** \return none
 ******************************************************************************/
 
void systemClock_xth(void)
{
	M0P_SYSCTRL->PERI_CLKEN0_f.FLASH=1;//打开FLASH外设时钟
	M0P_FLASH->BYPASS = 0x5A5A;
	M0P_FLASH->BYPASS = 0xA5A5;
	M0P_FLASH->CR_f.WAIT = 1;
	
  M0P_GPIO->PFADS_f.PF00 = 1;
  M0P_GPIO->PFADS_f.PF01 = 1;	
//	M0P_GPIO->PFDIR_f.PF00 = 1;
//	M0P_GPIO->PFDIR_f.PF01 = 1;
	M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
  M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.XTH_EN    = 1;	//使能XTH外部晶振
	

	M0P_SYSCTRL->PLL_CR_f.STARTUP    = 7;//16384个PLL周期
	M0P_SYSCTRL->PLL_CR_f.FRSEL      = 1;//输入PLL的频率范围为6M-12M
	M0P_SYSCTRL->PLL_CR_f.DIVN       = 6;//PLL输入时钟与输出时钟的倍频关系
	M0P_SYSCTRL->PLL_CR_f.FOSC       = 7;//PLL输出时钟频率范围为36M-48M
	M0P_SYSCTRL->PLL_CR_f.REFSEL     = 0;//选择外部XTH作为PLL的时钟输入

	M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
  M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.PLL_EN    = 1;

	while(1 != M0P_SYSCTRL->PLL_CR_f.STABLE);//等待PLL稳定
	
		M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
  M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.CLKSW=4;//系统时钟选择PLL
	
	M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
  M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.HCLK_PRS=0;//HCLK直接选择SystemClk
	
	M0P_SYSCTRL->SYSCTRL2 = 0x5A5A;
  M0P_SYSCTRL->SYSCTRL2 = 0xA5A5;
	M0P_SYSCTRL->SYSCTRL0_f.PCLK_PRS=0;//PCLK选择直接HCLK
}	











