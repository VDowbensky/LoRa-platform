#include "chacha20.h"

#define STATE_SIZE 16
#define BLOCK_SIZE 64
#define KEY_SIZE 32
#define NONCE_SIZE 12

// ChaCha20 structure
typedef struct 
{
  uint32_t state[STATE_SIZE];
} ChaCha20;

// XChaCha20 structure
typedef struct 
{
  ChaCha20 inner;
} XChaCha20;

// Simple Vec structure for keystream output (max 1024 bytes)
typedef struct 
{
  uint8_t data[1024];
  size_t len;
} Vec_u8_1024;

// Forward declarations
static void secure_zero_u32(uint32_t *data, size_t len);
static inline uint32_t u32_from_le_bytes(const uint8_t bytes[4]);
static inline void u32_to_le_bytes(uint32_t value, uint8_t bytes[4]);
static inline uint32_t rotate_left(uint32_t value, uint32_t shift);
static inline uint32_t wrapping_add(uint32_t a, uint32_t b);
static inline void quarter_round(uint32_t state[STATE_SIZE], size_t a, size_t b, size_t c, size_t d);
static void chacha20_block(const ChaCha20 *self, uint32_t counter, uint8_t output[BLOCK_SIZE]);

// ChaCha20 constants
static const uint32_t CHACHA20_CONSTANTS[4] = {0x61707865, 0x3320646e, 0x79622d32, 0x6b206574};

// Secure zero function for u32 arrays
static void secure_zero_u32(uint32_t *data, size_t len) 
{
  volatile uint32_t *ptr = data;
  for (size_t i = 0; i < len; i++) ptr[i] = 0;
}

// Convert 4 bytes to u32 (little-endian)
static inline uint32_t u32_from_le_bytes(const uint8_t bytes[4]) 
{
  return ((uint32_t)bytes[0]) | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

// Convert u32 to 4 bytes (little-endian)
static inline void u32_to_le_bytes(uint32_t value, uint8_t bytes[4]) 
{
  bytes[0] = (uint8_t)(value & 0xFF);
  bytes[1] = (uint8_t)((value >> 8) & 0xFF);
  bytes[2] = (uint8_t)((value >> 16) & 0xFF);
  bytes[3] = (uint8_t)((value >> 24) & 0xFF);
}

// Rotate left operation
static inline uint32_t rotate_left(uint32_t value, uint32_t shift) 
{
  return (value << shift) | (value >> (32 - shift));
}

// Wrapping add (overflow is expected and handled)
static inline uint32_t wrapping_add(uint32_t a, uint32_t b) 
{
  return a + b;
}

// ChaCha20 destructor - must be called manually to clean up
void chacha20_drop(ChaCha20 *self) 
{
  secure_zero_u32(self->state, STATE_SIZE);
}

// ChaCha20 constructor
ChaCha20 chacha20_new(const uint8_t key[KEY_SIZE], const uint8_t nonce[NONCE_SIZE]) 
{
  ChaCha20 self;
  uint32_t *state = self.state;
  state[0] = CHACHA20_CONSTANTS[0];
  state[1] = CHACHA20_CONSTANTS[1];
  state[2] = CHACHA20_CONSTANTS[2];
  state[3] = CHACHA20_CONSTANTS[3];
  state[4] = u32_from_le_bytes(&key[0]);
  state[5] = u32_from_le_bytes(&key[4]);
  state[6] = u32_from_le_bytes(&key[8]);
  state[7] = u32_from_le_bytes(&key[12]);
  state[8] = u32_from_le_bytes(&key[16]);
  state[9] = u32_from_le_bytes(&key[20]);
  state[10] = u32_from_le_bytes(&key[24]);
  state[11] = u32_from_le_bytes(&key[28]);
  state[12] = 0;
  state[13] = u32_from_le_bytes(&nonce[0]);
  state[14] = u32_from_le_bytes(&nonce[4]);
  state[15] = u32_from_le_bytes(&nonce[8]);
  return self;
}

// ChaCha20 constructor with counter
ChaCha20 chacha20_new_with_counter(const uint8_t key[KEY_SIZE], const uint8_t nonce[NONCE_SIZE], uint32_t counter) 
{
  ChaCha20 cipher = chacha20_new(key, nonce);
  cipher.state[12] = counter;
  return cipher;
}

// Quarter round function
static inline void quarter_round(uint32_t state[STATE_SIZE], size_t a, size_t b, size_t c, size_t d) 
{
  state[a] = wrapping_add(state[a], state[b]);
  state[d] ^= state[a];
  state[d] = rotate_left(state[d], 16);
  state[c] = wrapping_add(state[c], state[d]);
  state[b] ^= state[c];
  state[b] = rotate_left(state[b], 12);
  state[a] = wrapping_add(state[a], state[b]);
  state[d] ^= state[a];
  state[d] = rotate_left(state[d], 8);
  state[c] = wrapping_add(state[c], state[d]);
  state[b] ^= state[c];
  state[b] = rotate_left(state[b], 7);
}

// ChaCha20 block function
static void chacha20_block(const ChaCha20 *self, uint32_t counter, uint8_t output[BLOCK_SIZE]) 
{
  uint32_t state[STATE_SIZE];
  uint32_t working[STATE_SIZE];
  // Copy state and set counter
  memcpy(state, self->state, sizeof(uint32_t) * STATE_SIZE);
  state[12] = counter;
  // Copy state to working
  memcpy(working, state, sizeof(uint32_t) * STATE_SIZE);
  // Perform 10 double rounds
  for (int round = 0; round < 10; round++) 
  {
    // Column rounds
    quarter_round(working, 0, 4, 8, 12);
    quarter_round(working, 1, 5, 9, 13);
    quarter_round(working, 2, 6, 10, 14);
    quarter_round(working, 3, 7, 11, 15);
    // Diagonal rounds
    quarter_round(working, 0, 5, 10, 15);
    quarter_round(working, 1, 6, 11, 12);
    quarter_round(working, 2, 7, 8, 13);
    quarter_round(working, 3, 4, 9, 14);
  }
  // Add original state to working state
  for (size_t i = 0; i < STATE_SIZE; i++) working[i] = wrapping_add(working[i], state[i]);
  // Convert to bytes (little-endian)
  for (size_t i = 0; i < STATE_SIZE; i++) 
  {
    uint8_t bytes[4];
    u32_to_le_bytes(working[i], bytes);
    output[i * 4] = bytes[0];
    output[i * 4 + 1] = bytes[1];
    output[i * 4 + 2] = bytes[2];
    output[i * 4 + 3] = bytes[3];
  }
}

// Apply keystream to data
void chacha20_apply_keystream(const ChaCha20 *self, uint8_t *data, size_t data_len) 
{
  uint32_t counter = self->state[12];
  size_t offset = 0;
  while (offset < data_len) 
  {
    uint8_t keystream[BLOCK_SIZE];
    chacha20_block(self, counter, keystream);
    size_t chunk_len = data_len - offset;
    if (chunk_len > BLOCK_SIZE) chunk_len = BLOCK_SIZE;
    for (size_t i = 0; i < chunk_len; i++) data[offset + i] ^= keystream[i];
    offset += chunk_len;
    counter = wrapping_add(counter, 1);
  }
}

// Encrypt data
void chacha20_encrypt(const ChaCha20 *self, uint8_t *data, size_t data_len) 
{
  chacha20_apply_keystream(self, data, data_len);
}

// Decrypt data
void chacha20_decrypt(const ChaCha20 *self, uint8_t *data, size_t data_len) 
{
  chacha20_apply_keystream(self, data, data_len);
}

// Generate keystream
Vec_u8_1024 chacha20_keystream(const ChaCha20 *self, size_t len) 
{
  Vec_u8_1024 output;
  output.len = 0;
  uint32_t counter = self->state[12];
  size_t remaining = len;
  while (remaining > 0) 
  {
    uint8_t block[BLOCK_SIZE];
    chacha20_block(self, counter, block);
    size_t to_copy = remaining;
    if (to_copy > BLOCK_SIZE) to_copy = BLOCK_SIZE;
    for (size_t i = 0; i < to_copy; i++) 
    {
      if (output.len < 1024) 
      {
        output.data[output.len] = block[i];
        output.len++;
      }
    }
    remaining -= to_copy;
    counter = wrapping_add(counter, 1);
  }
  return output;
}

// HChaCha20 function
void hchacha20(const uint8_t key[32], const uint8_t nonce[16], uint8_t output[32]) 
{
  uint32_t state[16];
  state[0] = 0x61707865;
  state[1] = 0x3320646e;
  state[2] = 0x79622d32;
  state[3] = 0x6b206574;
  for (size_t i = 0; i < 8; i++) 
  {
    uint8_t key_bytes[4];
    key_bytes[0] = key[i * 4];
    key_bytes[1] = key[i * 4 + 1];
    key_bytes[2] = key[i * 4 + 2];
    key_bytes[3] = key[i * 4 + 3];
    state[4 + i] = u32_from_le_bytes(key_bytes);
  }
  for (size_t i = 0; i < 4; i++) 
  {
    uint8_t nonce_bytes[4];
    nonce_bytes[0] = nonce[i * 4];
    nonce_bytes[1] = nonce[i * 4 + 1];
    nonce_bytes[2] = nonce[i * 4 + 2];
    nonce_bytes[3] = nonce[i * 4 + 3];
    state[12 + i] = u32_from_le_bytes(nonce_bytes);
  }
  for (int round = 0; round < 10; round++) 
  {
    quarter_round(state, 0, 4, 8, 12);
    quarter_round(state, 1, 5, 9, 13);
    quarter_round(state, 2, 6, 10, 14);
    quarter_round(state, 3, 7, 11, 15);
    quarter_round(state, 0, 5, 10, 15);
    quarter_round(state, 1, 6, 11, 12);
    quarter_round(state, 2, 7, 8, 13);
    quarter_round(state, 3, 4, 9, 14);
  }
  for (size_t i = 0; i < 4; i++) 
  {
    uint8_t bytes[4];
    u32_to_le_bytes(state[i], bytes);
    output[i * 4] = bytes[0];
    output[i * 4 + 1] = bytes[1];
    output[i * 4 + 2] = bytes[2];
    output[i * 4 + 3] = bytes[3];
  }
  for (size_t i = 0; i < 4; i++) 
  {
    uint8_t bytes[4];
    u32_to_le_bytes(state[12 + i], bytes);
    output[16 + i * 4] = bytes[0];
    output[16 + i * 4 + 1] = bytes[1];
    output[16 + i * 4 + 2] = bytes[2];
    output[16 + i * 4 + 3] = bytes[3];
  }
}

// XChaCha20 constructor
XChaCha20 xchacha20_new(const uint8_t key[32], const uint8_t nonce[24]) 
{
  XChaCha20 self;
  // Extract first 16 bytes of nonce for hchacha20
  uint8_t hnonce[16];
  memcpy(hnonce, nonce, 16);
  // Derive subkey
  uint8_t subkey[32];
  hchacha20(key, hnonce, subkey);
  // Prepare ChaCha20 nonce (last 8 bytes of nonce with 4-byte zero prefix)
  uint8_t chacha_nonce[12];
  memset(chacha_nonce, 0, 4);
  memcpy(&chacha_nonce[4], &nonce[16], 8);
  // Initialize inner ChaCha20
  self.inner = chacha20_new(subkey, chacha_nonce);
  return self;
}

// XChaCha20 apply keystream
void xchacha20_apply_keystream(const XChaCha20 *self, uint8_t *data, size_t data_len) 
{
  chacha20_apply_keystream(&self->inner, data, data_len);
}
