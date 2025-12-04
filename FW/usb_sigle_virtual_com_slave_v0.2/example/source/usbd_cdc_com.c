/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
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
/** \file usbd_desc.c
 **
 ** A detailed description is available at
 ** @link 
		This file provides the USBD descriptors and string formating method.  
	@endlink
 **
 **   - 2019-06-28  lsq   First version for USB demo.
 **
 ******************************************************************************/
 
 
#include "usbd_cdc_com.h" 
#include "string.h"	
#include "stdarg.h"		 
#include "stdio.h"	 


//USB虚拟串口相关配置参数
LINE_CODING linecoding =
{
	115200,		//波特率
	0x00,   	//停止位,默认1位
	0x00,   	//校验位,默认无
	0x08    	//数据位,默认8位
}; 

uint8_t  USART_PRINTF_Buffer[USB_USART_REC_LEN];	//usb_printf发送缓冲区
uint8_t USB_USART_RX_BUF[USB_USART_REC_LEN]; 	    //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
uint16_t USB_USART_RX_STA=0;       				    //接收状态标记	 
extern uint8_t  APP_Rx_Buffer [];			        //虚拟串口发送缓冲区(发给电脑) 
extern uint32_t APP_Rx_ptr_in;   			        //虚拟串口接收缓冲区(接收来自电脑的数据)

//虚拟串口配置函数接口(供USB内核调用)
CDC_IF_Prop_TypeDef VCP_fops = 
{
	VCP_Init,
	VCP_DeInit,
	VCP_Ctrl,
	VCP_DataTx,
	VCP_DataRx
}; 

/**
******************************************************************************
	** \brief  初始化VCP
	**
	** \param  无
	** \retval USBD_OK
	**
******************************************************************************/
uint16_t VCP_Init(void)
{ 
	return USBD_OK;
} 

/**
******************************************************************************
	** \brief  去初始化VCP
	**
	** \param  无
	** \retval USBD_OK
	**
******************************************************************************/
uint16_t VCP_DeInit(void)
{ 
	return USBD_OK;
} 

/**
******************************************************************************
	** \brief  控制VCP的设置
	**
	** \param  Cmd：指令类型
    **         Buf：控制指令数据
    **         Len：指令长度
	** \retval USBD_OK
	**
******************************************************************************/
uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
	switch (Cmd)
	{
		case SEND_ENCAPSULATED_COMMAND:break;   
		case GET_ENCAPSULATED_RESPONSE:break;  
		case SET_COMM_FEATURE:break;  
		case GET_COMM_FEATURE:break;  
 		case CLEAR_COMM_FEATURE:break;  
		case SET_LINE_CODING:                            //设置串口参数：波特率、停止位、奇偶校验位和有效数据位
			linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));//波特率
			linecoding.format = Buf[4];                                                               //停止位
			linecoding.paritytype = Buf[5];                                                           //校验位
			linecoding.datatype = Buf[6];                                                             //数据位数
			break; 
		case GET_LINE_CODING:                            //读取串口参数：波特率、停止位、奇偶校验位和有效数据位
			Buf[0] = (uint8_t)(linecoding.bitrate);
			Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
			Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
			Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
			Buf[4] = linecoding.format;
			Buf[5] = linecoding.paritytype;
			Buf[6] = linecoding.datatype; 
			break; 
		case SET_CONTROL_LINE_STATE:break;   
		case SEND_BREAK:break;   
		default:break;  
	} 
	return USBD_OK;
}

/**
******************************************************************************
    ** \brief  向虚拟串口发送一个数据
	**
    ** \param  data：所要发送的数据
	** \retval USBD_OK
	**
******************************************************************************/
uint16_t VCP_DataTx (uint8_t data)
{  
	APP_Rx_Buffer[APP_Rx_ptr_in]=data;	//写入发送buf
	APP_Rx_ptr_in++;  					//写位置加1
	if(APP_Rx_ptr_in==APP_RX_DATA_SIZE)	//超过buf大小了,归零.
	{
		APP_Rx_ptr_in = 0;
	}   
	return USBD_OK;
} 

/**
******************************************************************************
    ** \brief  处理从USB虚拟串口接收到的数据
	**
    ** \param Buf：接收到的数据缓存指针
    **        Len：接收到的数据字节数
	** \retval USBD_OK
	**
******************************************************************************/
uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
	uint8_t i;
	uint8_t res;
	for(i=0;i<Len;i++)
	{  
		res=Buf[i];      
		if((USB_USART_RX_STA&0x8000)==0)		//接收未完成
		{
			if(USB_USART_RX_STA&0x4000)			//接收到了0x0d
			{
				if(res!=0x0a)USB_USART_RX_STA=0;//接收错误,重新开始
				else USB_USART_RX_STA|=0x8000;	//接收完成了 
			}else //还没收到0X0D
			{	
				if(res==0x0d)USB_USART_RX_STA|=0x4000;
				else
				{
					USB_USART_RX_BUF[USB_USART_RX_STA&0X3FFF]=res;
					USB_USART_RX_STA++;
					if(USB_USART_RX_STA>(USB_USART_REC_LEN-1))USB_USART_RX_STA=0;//接收数据错误,重新开始接收	
				}					
			}
		}   
	}  
	return USBD_OK;
}




























