#include "i2c.h"

void i2c1_init(void)
{
  LL_I2C_InitTypeDef I2C_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);
	
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00B07CB4;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
	LL_I2C_Enable(I2C1);
}

int8_t i2c1_write(uint8_t addr, uint8_t* buf,uint8_t len)
{
	// Send register address + data
	int32_t timeout;
	uint8_t i;
	
	LL_I2C_HandleTransfer(I2C1,addr,LL_I2C_ADDRSLAVE_7BIT,len,LL_I2C_MODE_AUTOEND,LL_I2C_GENERATE_START_WRITE); //2 - ???
	
	for(i = 0; i < len; i++)
	{
		timeout = I2C_TIMEOUT;
		while(!LL_I2C_IsActiveFlag_TXIS(I2C1))
		{
			timeout--;
			if(timeout == 0) return I2C_BUSY_TIMEOUT;
		}
		LL_I2C_TransmitData8(I2C1,buf[i]);
	}
	while(!LL_I2C_IsActiveFlag_STOP(I2C1))
	{
		timeout--;
		if(timeout == 0) return I2C_BUSY_TIMEOUT;
	}
	LL_I2C_ClearFlag_STOP(I2C1);
	return I2C_OK;
}
