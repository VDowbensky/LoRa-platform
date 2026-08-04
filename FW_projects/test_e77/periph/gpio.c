#include "gpio.h"

void mygpio_init(void)
{	
	LL_GPIO_InitTypeDef  gpio_init_structure;
	
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOH);

//LED
	gpio_init_structure.Mode  = LL_GPIO_MODE_OUTPUT;
	gpio_init_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init_structure.Pull  = LL_GPIO_PULL_NO;
  gpio_init_structure.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio_init_structure.Pin   = LED_PIN;
  LL_GPIO_Init(LED_PORT, &gpio_init_structure);
	LL_GPIO_ResetOutputPin(LED_PORT,LED_PIN);
	
//Key
  gpio_init_structure.Mode  = LL_GPIO_MODE_INPUT;
	gpio_init_structure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init_structure.Pull  = LL_GPIO_PULL_UP;
  gpio_init_structure.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio_init_structure.Pin   = ENCODER_PUSH_PIN;
	LL_GPIO_Init(ENCODER_PUSH_PORT, &gpio_init_structure);
//	
//	gpio_init_structure.Mode  = LL_GPIO_MODE_OUTPUT;
//	gpio_init_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  gpio_init_structure.Pull  = LL_GPIO_PULL_NO;
//  gpio_init_structure.Speed = LL_GPIO_SPEED_FREQ_LOW;
//	gpio_init_structure.Pin   = EXTRF_POWER_PIN;
//  LL_GPIO_Init(EXTRF_POWER_PORT, &gpio_init_structure);
//	LL_GPIO_SetOutputPin(EXTRF_POWER_PORT,EXTRF_POWER_PIN);

}
