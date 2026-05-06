#include "poly1305.h"

/* Constants */
#define TAG_SIZE 16
#define KEY_SIZE 32

/* Forward declarations for external dependencies */
typedef struct ChaCha20 ChaCha20;
ChaCha20* chacha20_new(const uint8_t key[32], const uint8_t nonce[12]);
ChaCha20* chacha20_new_with_counter(const uint8_t key[32], const uint8_t nonce[12], uint32_t counter);
void chacha20_keystream(ChaCha20* chacha, uint8_t* output, size_t len);
void chacha20_encrypt(ChaCha20* chacha, uint8_t* data, size_t len);
void chacha20_decrypt(ChaCha20* chacha, uint8_t* data, size_t len);
void chacha20_free(ChaCha20* chacha);
bool constant_time_eq(const uint8_t* a, const uint8_t* b, size_t len);

/* Poly1305 structure */
typedef struct 
{
  uint32_t r[5];
  uint32_t s[4];
  uint32_t h[5];
  uint8_t buffer[16];
  size_t buffer_len;
} Poly1305;

/* Helper function to convert 4 bytes to u32 (little-endian) */
static inline uint32_t bytes_to_u32_le(const uint8_t bytes[4]) 
{
  return ((uint32_t)bytes[0]) | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

/* Helper function to convert u32 to bytes (little-endian) */
static inline void u32_to_bytes_le(uint32_t val, uint8_t bytes[4]) 
{
  bytes[0] = val & 0xff;
  bytes[1] = (val >> 8) & 0xff;
  bytes[2] = (val >> 16) & 0xff;
  bytes[3] = (val >> 24) & 0xff;
}

/* Helper function to convert u64 to bytes (little-endian) */
static inline void u64_to_bytes_le(uint64_t val, uint8_t bytes[8]) 
{
  bytes[0] = val & 0xff;
  bytes[1] = (val >> 8) & 0xff;
  bytes[2] = (val >> 16) & 0xff;
  bytes[3] = (val >> 24) & 0xff;
  bytes[4] = (val >> 32) & 0xff;
  bytes[5] = (val >> 40) & 0xff;
  bytes[6] = (val >> 48) & 0xff;
  bytes[7] = (val >> 56) & 0xff;
}

/* Poly1305 constructor */
Poly1305 poly1305_new(const uint8_t key[KEY_SIZE]) 
{
  Poly1305 poly;
  /* Extract r values from key[0..16] */
  uint32_t r0 = bytes_to_u32_le(&key[0]) & 0x0fffffff;
  uint32_t r1 = bytes_to_u32_le(&key[4]) & 0x0ffffffc;
  uint32_t r2 = bytes_to_u32_le(&key[8]) & 0x0ffffffc;
  uint32_t r3 = bytes_to_u32_le(&key[12]) & 0x0ffffffc;
  /* Pack r into 26-bit limbs */
  poly.r[0] = r0 & 0x03ffffff;
  poly.r[1] = ((r0 >> 26) | (r1 << 6)) & 0x03ffffff;
  poly.r[2] = ((r1 >> 20) | (r2 << 12)) & 0x03ffffff;
  poly.r[3] = ((r2 >> 14) | (r3 << 18)) & 0x03ffffff;
  poly.r[4] = r3 >> 8;
  /* Extract s values from key[16..32] */
  poly.s[0] = bytes_to_u32_le(&key[16]);
  poly.s[1] = bytes_to_u32_le(&key[20]);
  poly.s[2] = bytes_to_u32_le(&key[24]);
  poly.s[3] = bytes_to_u32_le(&key[28]);
  /* Initialize h to zero */
  poly.h[0] = 0;
  poly.h[1] = 0;
  poly.h[2] = 0;
  poly.h[3] = 0;
  poly.h[4] = 0;
  /* Initialize buffer */
  memset(poly.buffer, 0, 16);
  poly.buffer_len = 0;
  return poly;
}

/* Process a single 16-byte block */
static void poly1305_process_block(Poly1305* poly, const uint8_t block[16], bool final_block) 
{
  /* Extract block into t values */
  uint32_t t0 = bytes_to_u32_le(&block[0]);
  uint32_t t1 = bytes_to_u32_le(&block[4]);
  uint32_t t2 = bytes_to_u32_le(&block[8]);
  uint32_t t3 = bytes_to_u32_le(&block[12]);
  /* Set hibit based on final_block */
  uint32_t hibit = final_block ? 0 : (1 << 24);
  /* Add block to accumulator h */
  poly->h[0] += t0 & 0x03ffffff;
  poly->h[1] += ((t0 >> 26) | (t1 << 6)) & 0x03ffffff;
  poly->h[2] += ((t1 >> 20) | (t2 << 12)) & 0x03ffffff;
  poly->h[3] += ((t2 >> 14) | (t3 << 18)) & 0x03ffffff;
  poly->h[4] += (t3 >> 8) | hibit;
  /* Multiply h by r */
  uint64_t r0 = poly->r[0];
  uint64_t r1 = poly->r[1];
  uint64_t r2 = poly->r[2];
  uint64_t r3 = poly->r[3];
  uint64_t r4 = poly->r[4];
  uint64_t s1 = r1 * 5;
  uint64_t s2 = r2 * 5;
  uint64_t s3 = r3 * 5;
  uint64_t s4 = r4 * 5;
  uint64_t h0 = poly->h[0];
  uint64_t h1 = poly->h[1];
  uint64_t h2 = poly->h[2];
  uint64_t h3 = poly->h[3];
  uint64_t h4 = poly->h[4];
  uint64_t d0 = h0 * r0 + h1 * s4 + h2 * s3 + h3 * s2 + h4 * s1;
  uint64_t d1 = h0 * r1 + h1 * r0 + h2 * s4 + h3 * s3 + h4 * s2;
  uint64_t d2 = h0 * r2 + h1 * r1 + h2 * r0 + h3 * s4 + h4 * s3;
  uint64_t d3 = h0 * r3 + h1 * r2 + h2 * r1 + h3 * r0 + h4 * s4;
  uint64_t d4 = h0 * r4 + h1 * r3 + h2 * r2 + h3 * r1 + h4 * r0;
   /* Carry propagation */
  uint64_t c;
  c = d0 >> 26;
  poly->h[0] = (uint32_t)(d0 & 0x03ffffff);
  d1 = d1 + c;
  c = d1 >> 26;
  poly->h[1] = (uint32_t)(d1 & 0x03ffffff);
  d2 = d2 + c;
  c = d2 >> 26;
  poly->h[2] = (uint32_t)(d2 & 0x03ffffff);
  d3 = d3 + c;
  c = d3 >> 26;
  poly->h[3] = (uint32_t)(d3 & 0x03ffffff);
  d4 = d4 + c;
  c = d4 >> 26;
  poly->h[4] = (uint32_t)(d4 & 0x03ffffff);
  poly->h[0] += (uint32_t)(c * 5);
  c = (uint64_t)(poly->h[0] >> 26);
  poly->h[0] &= 0x03ffffff;
  poly->h[1] += (uint32_t)c;
}

/* Update with data */
void poly1305_update(Poly1305* poly, const uint8_t* data, size_t len) 
{
  size_t offset = 0;
  /* Handle buffered data */
  if (poly->buffer_len > 0) 
  {
    size_t needed = 16 - poly->buffer_len;
    if (len >= needed) 
    {
      memcpy(&poly->buffer[poly->buffer_len], data, needed);
      poly1305_process_block(poly, poly->buffer, false);
      poly->buffer_len = 0;
      offset = needed;
    } 
    else 
    {
      memcpy(&poly->buffer[poly->buffer_len], data, len);
      poly->buffer_len += len;
      return;
    }
  }
  /* Process full blocks */
  while (offset + 16 <= len) 
  {
    poly1305_process_block(poly, &data[offset], false);
    offset += 16;
  }
  /* Buffer remaining data */
  if (offset < len) 
  {
    size_t remaining = len - offset;
    memcpy(poly->buffer, &data[offset], remaining);
    poly->buffer_len = remaining;
  }
}

/* Finalize and compute tag */
void poly1305_finalize(Poly1305* poly, uint8_t tag[TAG_SIZE]) 
{
  /* Process final block if there's buffered data */
  if (poly->buffer_len > 0) 
  {
    /* Pad with 0x01 followed by zeros */
    poly->buffer[poly->buffer_len] = 1;
    for (size_t i = poly->buffer_len + 1; i < 16; i++) poly->buffer[i] = 0;
    poly1305_process_block(poly, poly->buffer, true);
  }
  /* Fully carry h */
  uint32_t c;
  c = poly->h[1] >> 26;
  poly->h[1] &= 0x03ffffff;
  poly->h[2] += c;
  c = poly->h[2] >> 26;
  poly->h[2] &= 0x03ffffff;
  poly->h[3] += c;
  c = poly->h[3] >> 26;
  poly->h[3] &= 0x03ffffff;
  poly->h[4] += c;
  c = poly->h[4] >> 26;
  poly->h[4] &= 0x03ffffff;
  poly->h[0] += c * 5;
  c = poly->h[0] >> 26;
  poly->h[0] &= 0x03ffffff;
  poly->h[1] += c;
   /* Compute h + 5 */
  uint32_t g0 = poly->h[0] + 5;
  c = g0 >> 26;
  g0 &= 0x03ffffff;
  uint32_t g1 = poly->h[1] + c;
  c = g1 >> 26;
  g1 &= 0x03ffffff;
  uint32_t g2 = poly->h[2] + c;
  c = g2 >> 26;
  g2 &= 0x03ffffff;
  uint32_t g3 = poly->h[3] + c;
  c = g3 >> 26;
  g3 &= 0x03ffffff;
  uint32_t g4 = poly->h[4] + c - (1 << 26);
  /* Select h or h+5 based on overflow */
  uint32_t mask = (g4 >> 31) - 1;
  g0 &= mask;
  g1 &= mask;
  g2 &= mask;
  g3 &= mask;
  mask = ~mask;
  poly->h[0] = (poly->h[0] & mask) | g0;
  poly->h[1] = (poly->h[1] & mask) | g1;
  poly->h[2] = (poly->h[2] & mask) | g2;
  poly->h[3] = (poly->h[3] & mask) | g3;
  /* Unpack h into 32-bit words */
  uint32_t h0 = poly->h[0] | (poly->h[1] << 26);
  uint32_t h1 = (poly->h[1] >> 6) | (poly->h[2] << 20);
  uint32_t h2 = (poly->h[2] >> 12) | (poly->h[3] << 14);
  uint32_t h3 = (poly->h[3] >> 18) | (poly->h[4] << 8);
  /* Add s to h */
  uint64_t f;
  f = (uint64_t)h0 + (uint64_t)poly->s[0];
  uint32_t t0 = (uint32_t)f;
  f = (uint64_t)h1 + (uint64_t)poly->s[1] + (f >> 32);
  uint32_t t1 = (uint32_t)f;
  f = (uint64_t)h2 + (uint64_t)poly->s[2] + (f >> 32);
  uint32_t t2 = (uint32_t)f;
  f = (uint64_t)h3 + (uint64_t)poly->s[3] + (f >> 32);
  uint32_t t3 = (uint32_t)f;
  /* Convert to bytes */
  u32_to_bytes_le(t0, &tag[0]);
  u32_to_bytes_le(t1, &tag[4]);
  u32_to_bytes_le(t2, &tag[8]);
  u32_to_bytes_le(t3, &tag[12]);
}

/* Compute MAC in one shot */
void poly1305_mac(const uint8_t key[KEY_SIZE], const uint8_t* data, size_t len, uint8_t tag[TAG_SIZE]) 
{
  Poly1305 poly = poly1305_new(key);
  poly1305_update(&poly, data, len);
  poly1305_finalize(&poly, tag);
}

/* Verify MAC */
bool poly1305_verify(const uint8_t key[KEY_SIZE], const uint8_t* data, size_t len, const uint8_t expected[TAG_SIZE]) 
{
  uint8_t computed[TAG_SIZE];
  poly1305_mac(key, data, len, computed);
  return constant_time_eq(computed, expected, TAG_SIZE);
}

/* ChaCha20-Poly1305 AEAD seal operation */
void chacha20poly1305_seal(const uint8_t key[32],const uint8_t nonce[12], const uint8_t* aad,
                          size_t aad_len,const uint8_t* plaintext,size_t plaintext_len,
                          uint8_t* ciphertext,size_t ciphertext_len,uint8_t tag[16]) 
{
  assert(ciphertext_len >= plaintext_len);
  /* Generate Poly1305 key */
  uint8_t poly_key[32];
  ChaCha20* chacha = chacha20_new(key, nonce);
  chacha20_keystream(chacha, poly_key, 32);
  chacha20_free(chacha);
  /* Encrypt plaintext */
  memcpy(ciphertext, plaintext, plaintext_len);
  chacha = chacha20_new_with_counter(key, nonce, 1);
  chacha20_encrypt(chacha, ciphertext, plaintext_len);
  chacha20_free(chacha);
  /* Compute authentication tag */
  Poly1305 poly = poly1305_new(poly_key);
  /* Authenticate AAD */
  poly1305_update(&poly, aad, aad_len);
  /* Pad AAD to 16-byte boundary */
  size_t aad_pad = (16 - (aad_len % 16)) % 16;
  if (aad_pad > 0) 
  {
    uint8_t padding[16] = {0};
    poly1305_update(&poly, padding, aad_pad);
  }
   /* Authenticate ciphertext */
  poly1305_update(&poly, ciphertext, plaintext_len);
  /* Pad ciphertext to 16-byte boundary */
  size_t ct_pad = (16 - (plaintext_len % 16)) % 16;
  if (ct_pad > 0) 
  {
    uint8_t padding[16] = {0};
    poly1305_update(&poly, padding, ct_pad);
  }
  /* Authenticate lengths */
  uint8_t len_bytes[8];
  u64_to_bytes_le((uint64_t)aad_len, len_bytes);
  poly1305_update(&poly, len_bytes, 8);
  u64_to_bytes_le((uint64_t)plaintext_len, len_bytes);
  poly1305_update(&poly, len_bytes, 8);
  /* Finalize tag */
  poly1305_finalize(&poly, tag);
}

/* ChaCha20-Poly1305 AEAD open operation */
bool chacha20poly1305_open(const uint8_t key[32],const uint8_t nonce[12],const uint8_t* aad,size_t aad_len,const uint8_t* ciphertext,
                          size_t ciphertext_len,const uint8_t tag[16],uint8_t* plaintext,size_t plaintext_len) 
{
  assert(plaintext_len >= ciphertext_len);
  /* Generate Poly1305 key */
  uint8_t poly_key[32];
  ChaCha20* chacha = chacha20_new(key, nonce);
  chacha20_keystream(chacha, poly_key, 32);
  chacha20_free(chacha);
  /* Verify authentication tag */
  Poly1305 poly = poly1305_new(poly_key);
  /* Authenticate AAD */
  poly1305_update(&poly, aad, aad_len);
  /* Pad AAD to 16-byte boundary */
  size_t aad_pad = (16 - (aad_len % 16)) % 16;
  if (aad_pad > 0) 
  {
    uint8_t padding[16] = {0};
    poly1305_update(&poly, padding, aad_pad);
  }
  /* Authenticate ciphertext */
  poly1305_update(&poly, ciphertext, ciphertext_len);
  /* Pad ciphertext to 16-byte boundary */
  size_t ct_pad = (16 - (ciphertext_len % 16)) % 16;
  if (ct_pad > 0) 
  {
    uint8_t padding[16] = {0};
    poly1305_update(&poly, padding, ct_pad);
  }
  /* Authenticate lengths */
  uint8_t len_bytes[8];
  u64_to_bytes_le((uint64_t)aad_len, len_bytes);
  poly1305_update(&poly, len_bytes, 8);
  u64_to_bytes_le((uint64_t)ciphertext_len, len_bytes);
  poly1305_update(&poly, len_bytes, 8);
  /* Compute and verify tag */
  uint8_t computed_tag[TAG_SIZE];
  poly1305_finalize(&poly, computed_tag);
  if (!constant_time_eq(computed_tag, tag, TAG_SIZE)) return false; 
  /* Decrypt ciphertext */
  memcpy(plaintext, ciphertext, ciphertext_len);
  chacha = chacha20_new_with_counter(key, nonce, 1);
  chacha20_decrypt(chacha, plaintext, ciphertext_len);
  chacha20_free(chacha);
  return true;
}
