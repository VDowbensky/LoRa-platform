#include "adc.h"

float Vcc;
float T;

const float Vref =  1200.0f; //mV
static uint8_t phase;

void myadc_init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
  //PA15   ------> ADC_IN11
  GPIO_InitStruct.Pin = VBATT_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(VBATT_PORT, &GPIO_InitStruct);
	
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC), LL_ADC_PATH_INTERNAL_VREFINT|LL_ADC_PATH_INTERNAL_TEMPSENSOR|LL_ADC_PATH_INTERNAL_VBAT);
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC, &ADC_InitStruct);
  LL_ADC_REG_SetSequencerConfigurable(ADC, LL_ADC_REG_SEQ_CONFIGURABLE);
  /* Poll for ADC channel configuration ready */
  while (LL_ADC_IsActiveFlag_CCRDY(ADC) == 0);
   /* Clear flag ADC channel configuration ready */
  LL_ADC_ClearFlag_CCRDY(ADC);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC, &ADC_REG_InitStruct);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0) wait_loop_index--;
  LL_ADC_SetOverSamplingScope(ADC, LL_ADC_OVS_DISABLE);
  LL_ADC_SetSamplingTimeCommonChannels(ADC, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetSamplingTimeCommonChannels(ADC, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_DisableIT_EOC(ADC);
  LL_ADC_DisableIT_EOS(ADC);
  LL_ADC_SetTriggerFrequencyMode(ADC, LL_ADC_TRIGGER_FREQ_HIGH);
  //Configure Regular Channel
  LL_ADC_REG_SetSequencerRanks(ADC, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);
  //Poll for ADC channel configuration ready 
  while (LL_ADC_IsActiveFlag_CCRDY(ADC) == 0);
  LL_ADC_ClearFlag_CCRDY(ADC);
  LL_ADC_SetChannelSamplingTime(ADC, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_COMMON_1);

	LL_ADC_Enable(ADC);
	LL_ADC_EnableIT_EOC(ADC);
	NVIC_SetPriority(ADC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_EnableIRQ(ADC_IRQn);

}

void ADC_IRQHandler(void)
{
	uint16_t adc_val;
	float mv;
	
	if(LL_ADC_IsActiveFlag_EOC(ADC))
	{
		LL_ADC_ClearFlag_EOC(ADC);//ADC_ISR_EOS
		//adc_start(false);
		adc_val = LL_ADC_REG_ReadConversionData12(ADC);
		//mv = ((Vref/4096.0) * adc_val - dco_value) / gain_value;
		//mv = (Vref/4096.0) * adc_val;
		//Vcc = mv;
		//adc_enable(false);
		//if (phase == 0)
		//{
		//	T = mv;
		//	adc_config_sample_sequence(0, 15); 
		//	phase = 1;
		//}
		//else
		//{
		//	Vcc = mv;
		//	adc_config_sample_sequence(0, 13); 
		//	phase = 0;
		//}
		//adc_enable(true);
	}
}
