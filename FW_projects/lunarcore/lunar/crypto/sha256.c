#include "sha256.h"

// SHA-256 constants
static const uint32_t K[64] = 
{
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static const uint32_t H_INIT[8] = 
{
  0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
  0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
};

#define DIGEST_SIZE 32

#define BLOCK_SIZE 64

typedef struct 
{
  uint32_t state[8];
  uint64_t total_len;
  uint8_t buffer[BLOCK_SIZE];
  size_t buffer_len;
} Sha256;

// Helper function for right rotation
static inline uint32_t rotate_right(uint32_t value, unsigned int count) 
{
  return (value >> count) | (value << (32 - count));
}

// Helper function to convert uint32_t to big-endian bytes
static void u32_to_be_bytes(uint32_t value, uint8_t *bytes) 
{
  bytes[0] = (value >> 24) & 0xFF;
  bytes[1] = (value >> 16) & 0xFF;
  bytes[2] = (value >> 8) & 0xFF;
  bytes[3] = value & 0xFF;
}

// Helper function to convert uint64_t to big-endian bytes
static void u64_to_be_bytes(uint64_t value, uint8_t *bytes) 
{
  bytes[0] = (value >> 56) & 0xFF;
  bytes[1] = (value >> 48) & 0xFF;
  bytes[2] = (value >> 40) & 0xFF;
  bytes[3] = (value >> 32) & 0xFF;
  bytes[4] = (value >> 24) & 0xFF;
  bytes[5] = (value >> 16) & 0xFF;
  bytes[6] = (value >> 8) & 0xFF;
  bytes[7] = value & 0xFF;
}

// Helper function to convert big-endian bytes to uint32_t
static uint32_t u32_from_be_bytes(const uint8_t *bytes) 
{
  return ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) | ((uint32_t)bytes[2] << 8) | ((uint32_t)bytes[3]);
}

// Forward declaration for process_block
static void sha256_process_block(Sha256 *self, const uint8_t block[BLOCK_SIZE]);

// Constructor: pub fn new() -> Self
void sha256_new(Sha256 *self) 
{
  memcpy(self->state, H_INIT, sizeof(H_INIT));
  self->total_len = 0;
  memset(self->buffer, 0, BLOCK_SIZE);
  self->buffer_len = 0;
}

// Method: pub fn update(&mut self, data: &[u8])
void sha256_update(Sha256 *self, const uint8_t *data, size_t data_len) 
{
  size_t offset = 0;
  if (self->buffer_len > 0) 
  {
    size_t needed = BLOCK_SIZE - self->buffer_len;
    if (data_len >= needed) 
    {
      memcpy(&self->buffer[self->buffer_len], &data[0], needed);
      uint8_t block_copy[BLOCK_SIZE];
      memcpy(block_copy, self->buffer, BLOCK_SIZE);
      sha256_process_block(self, block_copy);
      self->buffer_len = 0;
      offset = needed;
    } 
    else 
    {
      memcpy(&self->buffer[self->buffer_len], data, data_len);
      self->buffer_len += data_len;
      self->total_len += (uint64_t)data_len;
      return;
    }
  }
  while (offset + BLOCK_SIZE <= data_len) 
  {
    uint8_t block[BLOCK_SIZE];
    memcpy(block, &data[offset], BLOCK_SIZE);
    sha256_process_block(self, block);
    offset += BLOCK_SIZE;
  }
  if (offset < data_len) 
  {
    size_t remaining = data_len - offset;
    memcpy(&self->buffer[0], &data[offset], remaining);
    self->buffer_len = remaining;
  }
  self->total_len += (uint64_t)data_len;
}

// Method: pub fn finalize(mut self) -> [u8; DIGEST_SIZE]
void sha256_finalize(Sha256 *self, uint8_t digest[DIGEST_SIZE]) 
{
  uint64_t total_bits = self->total_len * 8;
  self->buffer[self->buffer_len] = 0x80;
  self->buffer_len += 1;
  if (self->buffer_len > 56) 
  {
    // Zero out the rest of the buffer
    for (size_t i = self->buffer_len; i < BLOCK_SIZE; i++) self->buffer[i] = 0;
    uint8_t block_copy[BLOCK_SIZE];
    memcpy(block_copy, self->buffer, BLOCK_SIZE);
    sha256_process_block(self, block_copy);
    self->buffer_len = 0;
  }
  // Zero out from buffer_len to 56
  for (size_t i = self->buffer_len; i < 56; i++) self->buffer[i] = 0;
  // Append length as big-endian 64-bit integer
  u64_to_be_bytes(total_bits, &self->buffer[56]);
  uint8_t block_copy[BLOCK_SIZE];
  memcpy(block_copy, self->buffer, BLOCK_SIZE);
  sha256_process_block(self, block_copy);
  // Convert state to digest bytes
  for (size_t i = 0; i < 8; i++) u32_to_be_bytes(self->state[i], &digest[i * 4]);
}

// Private method: fn process_block(&mut self, block: &[u8; BLOCK_SIZE])
static void sha256_process_block(Sha256 *self, const uint8_t block[BLOCK_SIZE]) 
{
  uint32_t w[64];
  // Initialize first 16 words from block
  for (int i = 0; i < 16; i++) w[i] = u32_from_be_bytes(&block[i * 4]);
  // Extend the first 16 words into the remaining 48 words
  for (int i = 16; i < 64; i++) 
  {
    uint32_t s0 = rotate_right(w[i - 15], 7) ^ rotate_right(w[i - 15], 18) ^ (w[i - 15] >> 3);
    uint32_t s1 = rotate_right(w[i - 2], 17) ^ rotate_right(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  // Initialize working variables
  uint32_t a = self->state[0];
  uint32_t b = self->state[1];
  uint32_t c = self->state[2];
  uint32_t d = self->state[3];
  uint32_t e = self->state[4];
  uint32_t f = self->state[5];
  uint32_t g = self->state[6];
  uint32_t h = self->state[7];
  // Main loop
  for (int i = 0; i < 64; i++) 
  {
    uint32_t s1 = rotate_right(e, 6) ^ rotate_right(e, 11) ^ rotate_right(e, 25);
    uint32_t ch = (e & f) ^ ((~e) & g);
    uint32_t temp1 = h + s1 + ch + K[i] + w[i];
    uint32_t s0 = rotate_right(a, 2) ^ rotate_right(a, 13) ^ rotate_right(a, 22);
    uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    uint32_t temp2 = s0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + temp1;
    d = c;
    c = b;
    b = a;
    a = temp1 + temp2;
  }
  // Add the compressed chunk to the current hash value
  self->state[0] = self->state[0] + a;
  self->state[1] = self->state[1] + b;
  self->state[2] = self->state[2] + c;
  self->state[3] = self->state[3] + d;
  self->state[4] = self->state[4] + e;
  self->state[5] = self->state[5] + f;
  self->state[6] = self->state[6] + g;
  self->state[7] = self->state[7] + h;
}

// Static method: pub fn hash(data: &[u8]) -> [u8; DIGEST_SIZE]
void sha256_hash(const uint8_t *data, size_t data_len, uint8_t digest[DIGEST_SIZE]) 
{
  Sha256 hasher;
  sha256_new(&hasher);
  sha256_update(&hasher, data, data_len);
  sha256_finalize(&hasher, digest);
}

// Static method: pub fn hash256(data: &[u8]) -> [u8; DIGEST_SIZE]
void sha256_hash256(const uint8_t *data, size_t data_len, uint8_t digest[DIGEST_SIZE]) 
{
  uint8_t first[DIGEST_SIZE];
  sha256_hash(data, data_len, first);
  sha256_hash(first, DIGEST_SIZE, digest);
}

// Default trait implementation: impl Default for Sha256
void sha256_default(Sha256 *self) 
{
  sha256_new(self);
}

// Clone trait implementation: impl Clone for Sha256
void sha256_clone(const Sha256 *src, Sha256 *dst) 
{
  memcpy(dst->state, src->state, sizeof(src->state));
  dst->total_len = src->total_len;
  memcpy(dst->buffer, src->buffer, sizeof(src->buffer));
  dst->buffer_len = src->buffer_len;
}
