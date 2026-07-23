#include "uart.h"


void myuart_init(uint32_t br)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
#if WORK_UART == 1	
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
	//LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
#elif WORK_UART == 2
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
#else
#error UART not defined!
#endif
	
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
#if WORK_UART == 1	
	GPIO_InitStruct.Pin = TXD0_PIN;
	LL_GPIO_Init(TXD0_PORT, &GPIO_InitStruct);
#elif WORK_UART == 2
	GPIO_InitStruct.Pin = TXD1_PIN;
  LL_GPIO_Init(TXD1_PORT, &GPIO_InitStruct);
#else
#error UART not defined!
#endif
	//GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
#if WORK_UART == 1	
	GPIO_InitStruct.Pin = RXD0_PIN;
	LL_GPIO_Init(RXD0_PORT, &GPIO_InitStruct);
#elif WORK_UART == 2
	GPIO_InitStruct.Pin = RXD1_PIN;
	LL_GPIO_Init(RXD1_PORT, &GPIO_InitStruct);
#else
#error UART not defined!
#endif
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = br;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
#if WORK_UART == 1	
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
	while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))));
	//config USART1 interrupts
	LL_USART_EnableIT_RXNE_RXFNE(USART1);
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
#elif WORK_UART == 2
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
	while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))));
	//config USART1 interrupts
	LL_USART_EnableIT_RXNE_RXFNE(USART2);
	NVIC_ClearPendingIRQ(USART2_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
#else
#error UART not defined!
#endif
}


#if WORK_UART == 1
void USART1_IRQHandler(void)
#elif WORK_UART == 2
void USART2_IRQHandler(void)
#else
#error UART not defined!
#endif
{
	cbRETARGET_Rx();
}






