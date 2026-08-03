#ifndef _I2C_H_
#define _I2C_H_

#include "bsp.h"

#define I2C_TIME							10000UL
#define I2C_OK								0
#define I2C_ERROR							-1

#define I2C_BUSY_TIMEOUT			-1
#define I2C_START_TIMEOUT			-2
#define I2C_T_ADDR_TIMEOUT		-3
#define I2C_R_ADDR_TIMEOUT		-4
#define I2C_WR_TIMEOUT				-5
#define I2C_RD_ACK_TIMEOUT		-6
#define I2C_RD_NAK_TIMEOUT		-7

#define I2C_TIMEOUT							10000UL

void i2c1_init(void);
int8_t i2c1_write(uint8_t addr, uint8_t* buf,uint8_t len);
void i2c1_writessd1306(uint8_t control,uint8_t *buf,uint16_t len);

#endif
