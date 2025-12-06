#include "subghz.h"

void subghz_interface_init(void)
{
  LL_GPIO_InitTypeDef  gpio_init_structure;
  /* Enable the Radio Switch Clock */
  //LL_AHB2_GRP1_EnableClock(RF_SW_CTRL1_GPIO);//(LL_AHB2_GRP1_PERIPH_GPIOA)
  //LL_AHB2_GRP1_EnableClock(RF_SW_CTRL2_GPIO);
  /* Configure the Radio Switch pin */
  gpio_init_structure.Pin   = RFSW_TX_PIN;
  gpio_init_structure.Mode  = LL_GPIO_MODE_OUTPUT;
	gpio_init_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_structure.Pull  = LL_GPIO_PULL_NO;
  gpio_init_structure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  LL_GPIO_Init(RFSW_TX_PORT, &gpio_init_structure);
  
	gpio_init_structure.Pin = RFSW_RX_PIN;
  LL_GPIO_Init(RFSW_RX_PORT, &gpio_init_structure);
	
	LL_GPIO_ResetOutputPin(RFSW_TX_PORT, RFSW_TX_PIN);
	LL_GPIO_ResetOutputPin(RFSW_RX_PORT, RFSW_RX_PIN);
  //SPI init
	LL_PWR_UnselectSUBGHZSPI_NSS();
	LL_RCC_RF_DisableReset();
	while (LL_RCC_IsRFUnderReset() != 0UL) {};
  LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_44);
  /* Enable wakeup signal of the Radio peripheral */
  //LL_PWR_SetRadioBusyTrigger(LL_PWR_RADIO_BUSY_TRIGGER_WU_IT); //???
  /* Clear Pending Flag */
  LL_PWR_ClearFlag_RFBUSY();
	SUBGHZSPI_Init(SUBGHZSPI_BAUDRATEPRESCALER_4);
	NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0);
	NVIC_ClearPendingIRQ(SUBGHZ_Radio_IRQn);
  NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}

void SUBGHZSPI_Init(uint32_t BaudratePrescaler)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
	
	LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_SUBGHZSPI);
	
//  SUBGHZSPI->CR1 &= ~SPI_CR1_SPE;
//SPI Mode: Master
//Communication Mode: 2 lines (Full-Duplex)
//Clock polarity: Low  
//Phase: 1st Edge
//NSS management: Internal (Done with External bit inside PWR
//Communication speed: BaudratePrescaler
//First bit: MSB
//CRC calculation: Disable
//  SUBGHZSPI->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | BaudratePrescaler | SPI_CR1_SSM;
//  SUBGHZSPI->CR2 = SPI_CR2_FRXTH |  SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
//Enable SUBGHZSPI Peripheral
//  SUBGHZSPI->CR1 |= SPI_CR1_SPE;
	
	LL_SPI_Disable(SUBGHZSPI);
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
	LL_SPI_Init(SUBGHZSPI,&SPI_InitStruct);
	LL_SPI_SetStandard(SUBGHZSPI, LL_SPI_PROTOCOL_MOTOROLA);
	//enable SPI
	LL_SPI_Enable(SUBGHZSPI);
}


uint8_t subghz_spi_transfer(uint8_t b)
{
	uint8_t data = 0;
	
	while (LL_SPI_IsActiveFlag_TXE(SUBGHZSPI) == 0);
	LL_SPI_TransmitData8(SUBGHZSPI,b);
	while (LL_SPI_IsActiveFlag_BSY(SUBGHZSPI) == 1);
	while(LL_SPI_IsActiveFlag_RXNE(SUBGHZSPI) == 0);
	data = LL_SPI_ReceiveData8(SUBGHZSPI);
  return data;
}

void subghz_select(void)
{
	LL_PWR_SelectSUBGHZSPI_NSS();
}

void subghz_deselect(void)
{
	LL_PWR_UnselectSUBGHZSPI_NSS();
}

void subghz_reset(void)
{
  LL_RCC_RF_EnableReset();
  delay_us(50);
  LL_RCC_RF_DisableReset();
  while (LL_RCC_IsRFUnderReset());
  delay_us(20);
}

void subghz_wait_on_busy(void) //not in use
{
	delay_us(10);
	while(PWR->SR2 & PWR_SR2_RFBUSYMS);
}

void subghz_rfsw_rx(void)
{
  LL_GPIO_ResetOutputPin(RFSW_TX_PORT, RFSW_TX_PIN);
  LL_GPIO_SetOutputPin(RFSW_RX_PORT, RFSW_RX_PIN); 
}

void subghz_rfsw_tx(void)
{
  LL_GPIO_SetOutputPin(RFSW_TX_PORT, RFSW_TX_PIN);
  LL_GPIO_ResetOutputPin(RFSW_RX_PORT, RFSW_RX_PIN); 
}
