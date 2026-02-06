// encryption.h
#ifndef ENCRYPTION_H
#define ENCRYPTION_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "crypto/aes.h"
#include "crypto/sha256.h"
#include "crypto/hmac.h"
#include "crypto/hkdf.h"
#include "crypto/x25519.h"
#include "crypto/chacha20.h"
#include "crypto/poly1305.h"
#include "channel.h"

// Constants
#define MAX_PAYLOAD_SIZE 237
#define NONCE_SIZE 16
#define MIC_SIZE 4
#define MIC_SIZE_ENHANCED 8
#define PKI_OVERHEAD (32 + 16)
#define MAX_VEC_SIZE 256
#define MAX_PKI_DECRYPT_SIZE 240
#define MAX_CHANNEL_KEYS 8

// Fixed-size vector structure to replace heapless::Vec
typedef struct 
{
  uint8_t data[MAX_PAYLOAD_SIZE];
  size_t len;
  size_t capacity;
} Vec;

// Large vector structure for PKI operations
typedef struct 
{
  uint8_t data[MAX_VEC_SIZE];
  size_t len;
  size_t capacity;
} VecLarge;

// MIC Mode enumeration
typedef enum 
{
  MIC_MODE_STANDARD,
  MIC_MODE_ENHANCED
} MicMode;

// EncryptionContext structure
typedef struct 
{
  ChannelKey key;
} EncryptionContext;

// KeyStore structure
typedef struct 
{
  EncryptionContext* channel_keys[MAX_CHANNEL_KEYS];
  uint8_t node_privkey[32];
  uint8_t node_pubkey[32];
  bool has_node_privkey;
  bool has_node_pubkey;
} KeyStore;

// Initialize a Vec
static inline void vec_init(Vec* v) 
{
  v->len = 0;
  v->capacity = MAX_PAYLOAD_SIZE;
  memset(v->data, 0, MAX_PAYLOAD_SIZE);
}

// Create Vec from slice
static inline bool vec_from_slice(Vec* v, const uint8_t* data, size_t len) 
{
  if (len > MAX_PAYLOAD_SIZE) return false;
  memcpy(v->data, data, len);
  v->len = len;
  v->capacity = MAX_PAYLOAD_SIZE;
  return true;
}

// Extend Vec from slice
static inline bool vec_extend_from_slice(Vec* v, const uint8_t* data, size_t len) 
{
  if (v->len + len > v->capacity) return false;
  memcpy(v->data + v->len, data, len);
  v->len += len;
  return true;
}

// Initialize a VecLarge
static inline void vec_large_init(VecLarge* v) 
{
  v->len = 0;
  v->capacity = MAX_VEC_SIZE;
  memset(v->data, 0, MAX_VEC_SIZE);
}

// Extend VecLarge from slice
static inline bool vec_large_extend_from_slice(VecLarge* v, const uint8_t* data, size_t len) 
{
  if (v->len + len > v->capacity) return false;
  memcpy(v->data + v->len, data, len);
  v->len += len;
  return true;
}

// Function declarations for EncryptionContext
EncryptionContext* encryption_context_new(ChannelKey key);
EncryptionContext* encryption_context_with_default_key(void);
EncryptionContext* encryption_context_from_channel_name(const char* name);
EncryptionContext* encryption_context_from_key_bytes(const uint8_t* key, size_t key_len);
void encryption_context_free(EncryptionContext* ctx);
bool encryption_context_is_encrypted(const EncryptionContext* ctx);
bool encryption_context_encrypt(const EncryptionContext* ctx, uint32_t packet_id, uint32_t sender, 
                                const uint8_t* plaintext, size_t plaintext_len, Vec* output);
bool encryption_context_decrypt(const EncryptionContext* ctx, uint32_t packet_id, uint32_t sender,
                                const uint8_t* ciphertext, size_t ciphertext_len, Vec* output);
uint8_t encryption_context_key_hash(const EncryptionContext* ctx);
EncryptionContext* encryption_context_default(void);

// MIC functions
void compute_mic(const uint8_t* data, size_t len, uint8_t mic[MIC_SIZE]);
bool verify_mic(const uint8_t* data, size_t data_len, const uint8_t* expected_mic, size_t mic_len);
void compute_mic_enhanced(const uint8_t* key, size_t key_len, const uint8_t* data, size_t data_len, 
                         uint8_t mic[MIC_SIZE_ENHANCED]);
bool verify_mic_enhanced(const uint8_t* key, size_t key_len, const uint8_t* data, size_t data_len,
                        const uint8_t* expected_mic, size_t mic_len);

// PKI functions
bool pki_encrypt(const uint8_t recipient_pubkey[32], const uint8_t sender_privkey[32],
                const uint8_t* plaintext, size_t plaintext_len, const uint8_t nonce[12],
                VecLarge* output);
bool pki_decrypt(const uint8_t recipient_privkey[32], const uint8_t* encrypted, size_t encrypted_len,
                const uint8_t nonce[12], VecLarge* output);

// KeyStore functions
KeyStore* keystore_new(void);
void keystore_free(KeyStore* store);
void keystore_set_channel_key(KeyStore* store, uint8_t index, const uint8_t* key, size_t key_len);
void keystore_set_channel_name(KeyStore* store, uint8_t index, const char* name);
const EncryptionContext* keystore_get_channel(const KeyStore* store, uint8_t index);
void keystore_set_node_keypair(KeyStore* store, const uint8_t privkey[32]);
void keystore_generate_node_keypair(KeyStore* store, const uint8_t entropy[32]);
const uint8_t* keystore_node_pubkey(const KeyStore* store);
bool keystore_encrypt_for_node(const KeyStore* store, const uint8_t recipient_pubkey[32],
                               const uint8_t* plaintext, size_t plaintext_len, const uint8_t nonce[12],
                               VecLarge* output);
bool keystore_decrypt_from_node(const KeyStore* store, const uint8_t* encrypted, size_t encrypted_len,
                                const uint8_t nonce[12], VecLarge* output);

// Helper function for constant-time comparison
static inline bool constant_time_eq(const uint8_t* a, size_t a_len, const uint8_t* b, size_t b_len);

#endif // ENCRYPTION_H

/*************************************EOF********************************************/
