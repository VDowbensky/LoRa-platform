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


// Fixed-capacity vector structure (mimics heapless::Vec)
typedef struct 
{
  uint8_t data[MAX_PAYLOAD_SIZE];
  size_t len;
  size_t capacity;
} Vec_u8_237;

typedef struct 
{
  uint8_t data[256];
  size_t len;
  size_t capacity;
} Vec_u8_256;

typedef struct 
{
  uint8_t data[240];
  size_t len;
  size_t capacity;
} Vec_u8_240;


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

// EncryptionContext structure
typedef struct 
{
  ChannelKey key;
} EncryptionContext;

// MIC mode enumeration
typedef enum 
{
  MIC_MODE_STANDARD,   // Standard MIC mode
  MIC_MODE_ENHANCED    // Enhanced MIC mode
} MicMode;

// KeyStore structure for managing channel keys and node keypair
typedef struct 
{
  // Array of optional encryption contexts for channels
  EncryptionContext channel_keys_data[MAX_CHANNEL_KEYS];
  bool channel_keys_present[MAX_CHANNEL_KEYS];
  // Node private key (optional)
  uint8_t node_privkey[32];
  bool node_privkey_present;
  // Node public key (optional)
  uint8_t node_pubkey[32];
  bool node_pubkey_present;
} KeyStore;


void Vec_u8_237_init(Vec_u8_237* vec);
bool Vec_u8_237_from_slice(Vec_u8_237* vec, const uint8_t* slice, size_t slice_len);
bool Vec_u8_237_extend_from_slice(Vec_u8_237* vec, const uint8_t* slice, size_t slice_len);
void Vec_u8_256_init(Vec_u8_256* vec);
bool Vec_u8_256_extend_from_slice(Vec_u8_256* vec, const uint8_t* slice, size_t slice_len);
void Vec_u8_240_init(Vec_u8_240* vec);
bool Vec_u8_240_extend_from_slice(Vec_u8_240* vec, const uint8_t* slice, size_t slice_len);
EncryptionContext EncryptionContext_new(ChannelKey key);
EncryptionContext EncryptionContext_with_default_key(void);
EncryptionContext EncryptionContext_from_channel_name(const char* name);
EncryptionContext EncryptionContext_from_key_bytes(const uint8_t* key, size_t key_len);
bool EncryptionContext_is_encrypted(const EncryptionContext* ctx);
bool EncryptionContext_encrypt(const EncryptionContext* ctx,uint32_t packet_id,uint32_t sender,const uint8_t* plaintext,size_t plaintext_len,
                              Vec_u8_237* ciphertext_out);
bool EncryptionContext_decrypt(const EncryptionContext* ctx,uint32_t packet_id,uint32_t sender,const uint8_t* ciphertext,
                              size_t ciphertext_len,Vec_u8_237* plaintext_out);
uint8_t EncryptionContext_key_hash(const EncryptionContext* ctx);
EncryptionContext EncryptionContext_default(void);
void compute_mic(const uint8_t* data, size_t data_len, uint8_t mic_out[MIC_SIZE]);
bool constant_time_eq(const uint8_t* a, const uint8_t* b, size_t len);
bool verify_mic(const uint8_t* data, size_t data_len, const uint8_t* expected_mic, size_t expected_mic_len);
void compute_mic_enhanced(const uint8_t* key, size_t key_len, const uint8_t* data, size_t data_len, uint8_t mic_out[MIC_SIZE_ENHANCED]);
bool verify_mic_enhanced(const uint8_t* key, size_t key_len, const uint8_t* data, size_t data_len, const uint8_t* expected_mic, size_t expected_mic_len);
MicMode MicMode_default(void);
bool pki_encrypt(const uint8_t recipient_pubkey[32],const uint8_t sender_privkey[32],const uint8_t* plaintext,size_t plaintext_len,
                const uint8_t nonce[12], Vec_u8_256* output);
bool pki_decrypt(const uint8_t recipient_privkey[32],const uint8_t* encrypted,size_t encrypted_len,
                const uint8_t nonce[12],Vec_u8_240* output);
void KeyStore_new(KeyStore* store);
void KeyStore_drop(KeyStore* store);
void KeyStore_set_channel_key(KeyStore* store, uint8_t index, const uint8_t* key, size_t key_len);
void KeyStore_set_channel_name(KeyStore* store, uint8_t index, const char* name);
const EncryptionContext* KeyStore_get_channel(const KeyStore* store, uint8_t index);
void KeyStore_set_node_keypair(KeyStore* store, const uint8_t privkey[32]);
void KeyStore_generate_node_keypair(KeyStore* store, const uint8_t entropy[32]);
const uint8_t* KeyStore_node_pubkey(const KeyStore* store);
bool KeyStore_encrypt_for_node(const KeyStore* store,const uint8_t recipient_pubkey[32],const uint8_t* plaintext,
                              size_t plaintext_len,const uint8_t nonce[12],Vec_u8_256* output);
bool KeyStore_decrypt_from_node(const KeyStore* store,const uint8_t* encrypted,size_t encrypted_len,
                                const uint8_t nonce[12],Vec_u8_240* output);
KeyStore KeyStore_default(void); 

#endif // ENCRYPTION_H

/*************************************EOF********************************************/
