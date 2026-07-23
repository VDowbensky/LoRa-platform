#include "aux_spi.h"

void auxspi_init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	//config GPIO's
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	GPIO_InitStruct.Pin = AUX_SCK_PIN | AUX_MOSI_PIN | AUX_MISO_PIN;
  LL_GPIO_Init(AUX_SCK_PORT, &GPIO_InitStruct);
//	GPIO_InitStruct.Pin = AUX_MOSI_PIN;
//	LL_GPIO_Init(AUX_MOSI_PORT, &GPIO_InitStruct);
//	GPIO_InitStruct.Pin = AUX_MISO_PIN;
//	LL_GPIO_Init(AUX_MISO_PORT, &GPIO_InitStruct);
	//config SPI peripheral
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4; //1000000; //maybe up to 16000000
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 7;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	LL_SPI_Init(AUX_SPI,&SPI_InitStruct);
	//LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
	//enable SPI
	LL_SPI_Enable(AUX_SPI);
}

uint8_t auxspi_transfer(uint8_t b)
{
	uint8_t data = 0;
	
	while (LL_SPI_IsActiveFlag_TXE(AUX_SPI) == 0);
	LL_SPI_TransmitData8(AUX_SPI,b);
	while (LL_SPI_IsActiveFlag_BSY(AUX_SPI) == 1);
	while(LL_SPI_IsActiveFlag_RXNE(AUX_SPI) == 0);
	data = LL_SPI_ReceiveData8(AUX_SPI);
  return data;
}