#include "base64.h"
// Base64 encoding/decoding
static const uint8_t BASE64_CHARS[64] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

// Base64 decode table (constant initialization in C)
static const int8_t BASE64_DECODE[128] = 
{
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, 62, -1, 63,
  52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, -1, -1, -1,
  -1,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
  15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, 63,
  -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
  41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1
};

// Base64 encode
size_t base64_encode(const uint8_t* data, size_t data_len, uint8_t* output, size_t output_len) 
{
  size_t o = 0;
  size_t i = 0;
  
  while (i + 2 < data_len) 
  {
    if (o + 4 > output_len) break;
    uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8) | (uint32_t)data[i + 2];
    output[o] = BASE64_CHARS[(n >> 18) & 0x3F];
    output[o + 1] = BASE64_CHARS[(n >> 12) & 0x3F];
    output[o + 2] = BASE64_CHARS[(n >> 6) & 0x3F];
    output[o + 3] = BASE64_CHARS[n & 0x3F];
    i += 3;
    o += 4;
  }
  if (i < data_len && o + 4 <= output_len) 
  {
    size_t remaining = data_len - i;
    if (remaining == 1) 
    {
      uint32_t n = (uint32_t)data[i] << 16;
      output[o] = BASE64_CHARS[(n >> 18) & 0x3F];
      output[o + 1] = BASE64_CHARS[(n >> 12) & 0x3F];
      o += 2;
    } 
    else if (remaining == 2) 
    {
      uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8);
      output[o] = BASE64_CHARS[(n >> 18) & 0x3F];
      output[o + 1] = BASE64_CHARS[(n >> 12) & 0x3F];
      output[o + 2] = BASE64_CHARS[(n >> 6) & 0x3F];
      o += 3;
    }
  }
  return o;
}

// Base64 decode
size_t base64_decode(const uint8_t* data, size_t data_len, uint8_t* output, size_t output_len) 
{
  size_t o = 0;
  size_t i = 0;
  // Skip URL prefix if present
  const char* prefix = "https://meshtastic.org/e/#";
  size_t prefix_len = strlen(prefix);
  if (data_len >= prefix_len && memcmp(data, prefix, prefix_len) == 0) 
  {
    data += prefix_len;
    data_len -= prefix_len;
  }
  while (i + 3 < data_len) 
  {
    if (o + 3 > output_len) break;
    int8_t b0 = (data[i] < 128) ? BASE64_DECODE[data[i]] : -1;
    int8_t b1 = (data[i + 1] < 128) ? BASE64_DECODE[data[i + 1]] : -1;
    int8_t b2 = (data[i + 2] < 128) ? BASE64_DECODE[data[i + 2]] : -1;
    int8_t b3 = (data[i + 3] < 128) ? BASE64_DECODE[data[i + 3]] : -1;
    if (b0 < 0 || b1 < 0) break;
    uint32_t n = ((uint32_t)b0 << 18) | ((uint32_t)b1 << 12) | (b2 >= 0 ? ((uint32_t)b2 << 6) : 0) | (b3 >= 0 ? (uint32_t)b3 : 0);
    output[o] = (uint8_t)(n >> 16);
    o += 1;
    if (b2 >= 0) 
    {
      output[o] = (uint8_t)(n >> 8);
      o += 1;
    }
    if (b3 >= 0) 
    {
      output[o] = (uint8_t)n;
      o += 1;
    }
    i += 4;
  }
  if (i + 1 < data_len && o < output_len) 
  {
    int8_t b0 = (data[i] < 128) ? BASE64_DECODE[data[i]] : -1;
    int8_t b1 = (data[i + 1] < 128) ? BASE64_DECODE[data[i + 1]] : -1;
    if (b0 >= 0 && b1 >= 0) 
    {
      uint32_t n = ((uint32_t)b0 << 18) | ((uint32_t)b1 << 12);
      output[o] = (uint8_t)(n >> 16);
      o += 1;
      if (i + 2 < data_len && o < output_len) 
      {
        int8_t b2 = (data[i + 2] < 128) ? BASE64_DECODE[data[i + 2]] : -1;
        if (b2 >= 0) 
        {
          n = ((uint32_t)b0 << 18) | ((uint32_t)b1 << 12) | ((uint32_t)b2 << 6);
          output[o - 1] = (uint8_t)(n >> 16);
          output[o] = (uint8_t)(n >> 8);
          o += 1;
        }
      }
    }
  }
  return o;
}