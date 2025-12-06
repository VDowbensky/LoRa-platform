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


//USB virtual serial port configuration parameters
LINE_CODING linecoding =
{
	115200,		//baud rate
	0x00,   	//Stop bit, default 1 bit
	0x00,   	//Parity check, none by default.
	0x08    	//Data bits, default 8 bits
}; 

//uint8_t  USART_PRINTF_Buffer[USB_USART_REC_LEN];	//usb_printf transmit buffer
uint8_t USB_USART_RX_BUF[USB_USART_REC_LEN]; 	    //Receive buffer, maximum USART_REC_LEN bytes.
//Receive status
uint16_t USB_USART_RX_STA=0;       				    //Receive status flags
extern uint8_t  APP_Rx_Buffer [];			        //Virtual serial port transmit buffer (to computer)
extern uint32_t APP_Rx_ptr_in;   			        //Virtual serial port receive buffer (for receiving data from the computer)

//Virtual serial port configuration function interface (for USB kernel to call)
CDC_IF_Prop_TypeDef VCP_fops = 
{
	VCP_Init,
	VCP_DeInit,
	VCP_Ctrl,
	VCP_DataTx,
	VCP_DataRx
}; 

volatile int rxCount = 0;

/**
******************************************************************************
	** \brief  Initialize VCP
	**
	** \param  none
	** \retval USBD_OK
	**
******************************************************************************/
uint16_t VCP_Init(void)
{ 
	return USBD_OK;
} 

/**
******************************************************************************
	** \brief  deinitialize VCP
	**
	** \param  none
	** \retval USBD_OK
	**
******************************************************************************/
uint16_t VCP_DeInit(void)
{ 
	return USBD_OK;
} 

/**
******************************************************************************
	** \brief  Controlling VCP settings
	**
	** \param  Cmd£ºInstruction type
    **         Buf£ºControl command data
    **         Len£ºInstruction length
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
		case SET_LINE_CODING:                            //Configure serial port parameters: baud rate, stop bits, parity bits, and significant data bits.
			linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));//baud rate
			linecoding.format = Buf[4];                                                               //Stop bit
			linecoding.paritytype = Buf[5];                                                           //Parity
			linecoding.datatype = Buf[6];                                                             //Data bits
			break; 
		case GET_LINE_CODING:                            //Read serial port parameters: baud rate, stop bits, parity bits, and significant data bits.
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
    ** \brief  Send data to the virtual serial port
	**
    ** \param  data£ºData to be sent
	** \retval USBD_OK
	**
******************************************************************************/
uint16_t VCP_DataTx (uint8_t data)
{  
	APP_Rx_Buffer[APP_Rx_ptr_in]=data;	//Write to send buf
	APP_Rx_ptr_in++;  					//Write position +1
	if(APP_Rx_ptr_in==APP_RX_DATA_SIZE)	//If the buffer size is exceeded, reset to zero.
	{
		APP_Rx_ptr_in = 0;
	}   
	return USBD_OK;
} 

/**
******************************************************************************
    ** \brief  Process data received from the USB virtual serial port
	**
    ** \param Buf£ºReceived data buffer pointer
    **        Len£ºNumber of bytes of data received
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
//		if((USB_USART_RX_STA&0x8000)==0)		//Reception not completed
//		{
			//if(USB_USART_RX_STA&0x4000)			//Received 0x0d
			//{
				//if(res!=0x0a)USB_USART_RX_STA=0;//Error received, restarting
				//else USB_USART_RX_STA|=0x8000;	//Received successfully
			//}else //Haven't received 0X0D yet
			//{	
				//if(res==0x0d)USB_USART_RX_STA|=0x4000;
				//else
				//{
					USB_USART_RX_BUF[USB_USART_RX_STA&0X3FFF]=res;
					rxCount++;
					USB_USART_RX_STA++;
					if(USB_USART_RX_STA>(USB_USART_REC_LEN-1))USB_USART_RX_STA=0;//Data received error, restart receiving.
				//}					
			//}
//		}   
	}  
	return USBD_OK;
}
