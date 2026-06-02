#include "hmac.h"

#include "sha256.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// HmacSha256 structure definition
typedef struct 
{
  Sha256 inner;
  uint8_t outer_key[BLOCK_SIZE];
} HmacSha256;

// Initialize a new HmacSha256 instance with the given key
void hmac_sha256_new(HmacSha256* hmac, const uint8_t* key, size_t key_len) 
{
  uint8_t key_block[BLOCK_SIZE];
  memset(key_block, 0, BLOCK_SIZE);
  // If key is longer than BLOCK_SIZE, hash it first
  if (key_len > BLOCK_SIZE) 
  {
    uint8_t hashed[DIGEST_SIZE];
    sha256_hash(hashed, key, key_len);
    memcpy(key_block, hashed, DIGEST_SIZE);
  } 
  else memcpy(key_block, key, key_len);
  uint8_t inner_key[BLOCK_SIZE];
  memset(inner_key, 0, BLOCK_SIZE);
  memset(hmac->outer_key, 0, BLOCK_SIZE);
  // XOR key_block with ipad (0x36) and opad (0x5c)
  for (size_t i = 0; i < BLOCK_SIZE; i++) 
  {
    inner_key[i] = key_block[i] ^ 0x36;
    hmac->outer_key[i] = key_block[i] ^ 0x5c;
  }
  // Initialize inner hash with inner_key
  sha256_init(&hmac->inner);
  sha256_update(&hmac->inner, inner_key, BLOCK_SIZE);
}

// Update the HMAC with additional data
void hmac_sha256_update(HmacSha256* hmac, const uint8_t* data, size_t data_len) 
{
  sha256_update(&hmac->inner, data, data_len);
}

// Finalize the HMAC computation and return the MAC
void hmac_sha256_finalize(HmacSha256* hmac, uint8_t output[DIGEST_SIZE]) 
{
  uint8_t inner_hash[DIGEST_SIZE];
  // Finalize the inner hash
  sha256_finalize(&hmac->inner, inner_hash);
  // Compute outer hash: H(outer_key || inner_hash)
  Sha256 outer;
  sha256_init(&outer);
  sha256_update(&outer, hmac->outer_key, BLOCK_SIZE);
  sha256_update(&outer, inner_hash, DIGEST_SIZE);
  sha256_finalize(&outer, output);
}

// One-shot function to compute HMAC-SHA256
void hmac_sha256_mac(uint8_t output[DIGEST_SIZE], const uint8_t* key, size_t key_len,const uint8_t* data, size_t data_len) 
{
  HmacSha256 hmac;
  hmac_sha256_new(&hmac, key, key_len);
  hmac_sha256_update(&hmac, data, data_len);
  hmac_sha256_finalize(&hmac, output);
}

// Verify an HMAC-SHA256 MAC
bool hmac_sha256_verify(const uint8_t* key, size_t key_len,
                        const uint8_t* data, size_t data_len,
                        const uint8_t expected[DIGEST_SIZE]) 
{
  uint8_t computed[DIGEST_SIZE];
  hmac_sha256_mac(computed, key, key_len, data, data_len);
  return constant_time_eq(computed, expected, DIGEST_SIZE);
}

// Clone an HmacSha256 instance
void hmac_sha256_clone(HmacSha256* dest, const HmacSha256* src) 
{
  sha256_clone(&dest->inner, &src->inner);
  memcpy(dest->outer_key, src->outer_key, BLOCK_SIZE);
}
