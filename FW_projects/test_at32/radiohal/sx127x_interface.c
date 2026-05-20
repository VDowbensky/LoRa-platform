#include "sx127x_interface.h"

void Sx1276SetNSS(uint8_t nss)
{
	if(nss == 0) gpio_bits_reset(RF_CS_GPIO_PORT,RF_CS_PIN);
	else gpio_bits_set(RF_CS_GPIO_PORT,RF_CS_PIN);
}

uint8_t Sx1276SpiInOut(uint8_t b)
{
	return spi1_transfer(b);
}