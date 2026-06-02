#ifndef _BASE64_H_
#define _BASE64_H_

#include "bsp.h"

size_t base64_encode(const uint8_t* data, size_t data_len, uint8_t* output, size_t output_len);
size_t base64_decode(const uint8_t* data, size_t data_len, uint8_t* output, size_t output_len); 

#endif
