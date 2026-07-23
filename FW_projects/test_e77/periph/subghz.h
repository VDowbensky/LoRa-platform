#ifndef _SUBGHZ_H_
#define _SUBGHZ_H_

#include "bsp.h"

#define SUBGHZSPI_BAUDRATEPRESCALER_2       (0x00000000U)
#define SUBGHZSPI_BAUDRATEPRESCALER_4       (SPI_CR1_BR_0)
#define SUBGHZSPI_BAUDRATEPRESCALER_8       (SPI_CR1_BR_1)
#define SUBGHZSPI_BAUDRATEPRESCALER_16      (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SUBGHZSPI_BAUDRATEPRESCALER_32      (SPI_CR1_BR_2)
#define SUBGHZSPI_BAUDRATEPRESCALER_64      (SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SUBGHZSPI_BAUDRATEPRESCALER_128     (SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SUBGHZSPI_BAUDRATEPRESCALER_256     (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

void subghz_interface_init(void);
void SUBGHZSPI_Init(uint32_t BaudratePrescaler);
uint8_t subghz_spi_transfer(uint8_t b);
void subghz_reset(void);
void subghz_select(void);
void subghz_deselect(void);
void subghz_wait_on_busy(void); //not in use because whole loop
void subghz_rfsw_rx(void);
void subghz_rfsw_tx(void);
void subghz_rfsw_off(void);

#endif
