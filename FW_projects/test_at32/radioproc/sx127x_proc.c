#include "sx127x_proc.h"

int8_t SX127x_init(void)
{
	
}

int8_t SX127x_set_mod_params(uint16_t bw_khz,uint8_t sf,uint8_t cr,uint8_t ldropt)
{
	
}

int8_t SX127x_set_packet_params(uint8_t sync,uint16_t prelen,uint8_t paylen,uint8_t header,uint8_t crc,uint8_t invertiq)
{
	
}

void SX127X_setopmode(uint8_t mode)
{
	
}

void SX127X_irq_handler(void)
{
	
}


//Some hardware-related hard-coded content
//Switching RF output port
uint8_t SX1276GetPaSelect(uint32_t channel)
{
	//Since the hardware is connected to PABOOST and RFO is not used, it uniformly returns RF_PACONFIG_PASELECT_PABOOST.
	return RF_PACONFIG_PASELECT_PABOOST;
	//if(channel < RF_MID_BAND_THRESH) return RF_PACONFIG_PASELECT_PABOOST;
	//else return RF_PACONFIG_PASELECT_RFO;
}

void SX1276AntSwInit( void )
{
	//Our RA-01/02 module's antenna switching is automatically controlled by hardware, requiring no software control.
	//GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
	//GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
}

void SX1276AntSwDeInit( void )
{
	//Our RA-01/02 module's antenna switching switch is automatically controlled by hardware and does not require software control.
	//GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
	//GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
}

void SX1276SetAntSw( uint8_t rxTx )
{
	//Our RA-01/02 module's antenna switching is automatically controlled by hardware, requiring no software control.
	//if( rxTx != 0 ) // 1: TX, 0: RX
	//{
			//GpioWrite( &AntSwitchLf, 0 );
			//GpioWrite( &AntSwitchHf, 1 );
	//}
	//else
	//{
			//GpioWrite( &AntSwitchLf, 1 );
			//GpioWrite( &AntSwitchHf, 0 );
	//}
}
