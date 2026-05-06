#include "encryption.h"


// Initialize a Vec_u8_237
void Vec_u8_237_init(Vec_u8_237* vec) 
{
  vec->len = 0;
  vec->capacity = MAX_PAYLOAD_SIZE;
  memset(vec->data, 0, MAX_PAYLOAD_SIZE);
}

// Create Vec_u8_237 from slice
bool Vec_u8_237_from_slice(Vec_u8_237* vec, const uint8_t* slice, size_t slice_len) 
{
  if (slice_len > MAX_PAYLOAD_SIZE) return false;
  Vec_u8_237_init(vec);
  memcpy(vec->data, slice, slice_len);
  vec->len = slice_len;
  return true;
}

// Extend Vec_u8_237 from slice
bool Vec_u8_237_extend_from_slice(Vec_u8_237* vec, const uint8_t* slice, size_t slice_len) 
{
  if (vec->len + slice_len > vec->capacity) return false;
  memcpy(vec->data + vec->len, slice, slice_len);
  vec->len += slice_len;
  return true;
}

// Initialize a Vec_u8_256
void Vec_u8_256_init(Vec_u8_256* vec) 
{
  vec->len = 0;
  vec->capacity = 256;
  memset(vec->data, 0, 256);
}

// Extend Vec_u8_256 from slice
bool Vec_u8_256_extend_from_slice(Vec_u8_256* vec, const uint8_t* slice, size_t slice_len) 
{
  if (vec->len + slice_len > vec->capacity) return false;
  memcpy(vec->data + vec->len, slice, slice_len);
  vec->len += slice_len;
  return true;
}

// Initialize a Vec_u8_240
void Vec_u8_240_init(Vec_u8_240* vec) 
{
  vec->len = 0;
  vec->capacity = 240;
  memset(vec->data, 0, 240);
}

// Extend Vec_u8_240 from slice
bool Vec_u8_240_extend_from_slice(Vec_u8_240* vec, const uint8_t* slice, size_t slice_len) 
{
  if (vec->len + slice_len > vec->capacity) return false;
  memcpy(vec->data + vec->len, slice, slice_len);
  vec->len += slice_len;
  return true;
}

// Create new EncryptionContext with given key
EncryptionContext EncryptionContext_new(ChannelKey key) 
{
  EncryptionContext ctx;
  ctx.key = key;
  return ctx;
}

// Create EncryptionContext with default key
EncryptionContext EncryptionContext_with_default_key(void) 
{
  return EncryptionContext_new(ChannelKey_default_key());
}

// Create EncryptionContext from channel name
EncryptionContext EncryptionContext_from_channel_name(const char* name) 
{
  return EncryptionContext_new(ChannelKey_from_channel_name(name));
}

// Create EncryptionContext from key bytes
EncryptionContext EncryptionContext_from_key_bytes(const uint8_t* key, size_t key_len) 
{
  return EncryptionContext_new(ChannelKey_from_bytes(key, key_len));
}

// Check if encryption context uses encryption
bool EncryptionContext_is_encrypted(const EncryptionContext* ctx) 
{
  return ChannelKey_is_encrypted(&ctx->key);
}

// Encrypt plaintext with given packet_id and sender
// Returns true if successful, false otherwise
// Output is written to ciphertext_out, length to ciphertext_len_out
bool EncryptionContext_encrypt(const EncryptionContext* ctx,uint32_t packet_id,uint32_t sender,const uint8_t* plaintext,size_t plaintext_len,Vec_u8_237* ciphertext_out) 
{
  // If not encrypted, just copy plaintext
  if (!ChannelKey_is_encrypted(&ctx->key)) return Vec_u8_237_from_slice(ciphertext_out, plaintext, plaintext_len);
  // Check size constraint
  if (plaintext_len > MAX_PAYLOAD_SIZE) return false;
  // Derive nonce
  uint8_t nonce[NONCE_SIZE];
  mesh_kdf_derive_nonce(packet_id, sender, nonce);
  // Initialize ciphertext with plaintext
  Vec_u8_237_init(ciphertext_out);
  if (!Vec_u8_237_extend_from_slice(ciphertext_out, plaintext, plaintext_len)) return false;
  // Encrypt based on key type
  switch (ctx->key.type) 
  {
    case CHANNEL_KEY_AES128: 
    {
      Aes128 cipher;
      Aes128_new(&cipher, ctx->key.key);
      Aes128_encrypt_ctr(&cipher, nonce, ciphertext_out->data, ciphertext_out->len);
      break;
    }
    
    case CHANNEL_KEY_AES256: 
    {
      Aes256 cipher;
      Aes256_new(&cipher, ctx->key.key);
      Aes256_encrypt_ctr(&cipher, nonce, ciphertext_out->data, ciphertext_out->len);
      break;
    }
    
    case CHANNEL_KEY_NONE:
      // No encryption
      break;
  }
  return true;
}

// Decrypt ciphertext with given packet_id and sender
// Returns true if successful, false otherwise
// Output is written to plaintext_out, length to plaintext_len_out
bool EncryptionContext_decrypt(const EncryptionContext* ctx,uint32_t packet_id,uint32_t sender,const uint8_t* ciphertext,size_t ciphertext_len,Vec_u8_237* plaintext_out) 
{
  // If not encrypted, just copy ciphertext
  if (!ChannelKey_is_encrypted(&ctx->key)) return Vec_u8_237_from_slice(plaintext_out, ciphertext, ciphertext_len);
  // Check size constraints
  if (ciphertext_len == 0 || ciphertext_len > MAX_PAYLOAD_SIZE) return false;
  // Derive nonce
  uint8_t nonce[NONCE_SIZE];
  mesh_kdf_derive_nonce(packet_id, sender, nonce);
  // Initialize plaintext with ciphertext
  Vec_u8_237_init(plaintext_out);
  if (!Vec_u8_237_extend_from_slice(plaintext_out, ciphertext, ciphertext_len)) return false;
  // Decrypt based on key type
  switch (ctx->key.type) 
  {
    case CHANNEL_KEY_AES128: 
    {
      Aes128 cipher;
      Aes128_new(&cipher, ctx->key.key);
      Aes128_decrypt_ctr(&cipher, nonce, plaintext_out->data, plaintext_out->len);
      break;
    }
    
    case CHANNEL_KEY_AES256: 
    {
      Aes256 cipher;
      Aes256_new(&cipher, ctx->key.key);
      Aes256_decrypt_ctr(&cipher, nonce, plaintext_out->data, plaintext_out->len);
      break;
    }
    
    case CHANNEL_KEY_NONE:
    // No decryption
    break;
  }
  return true;
}

// Compute key hash (XOR of all key bytes)
uint8_t EncryptionContext_key_hash(const EncryptionContext* ctx) 
{
  const uint8_t* key_bytes;
  size_t key_len;
  ChannelKey_as_bytes(&ctx->key, &key_bytes, &key_len);
  if (key_len == 0) return 0;
  uint8_t h = 0;
  for (size_t i = 0; i < key_len; i++) h ^= key_bytes[i];
  return h;
}

// Create default EncryptionContext
EncryptionContext EncryptionContext_default(void) 
{
  return EncryptionContext_with_default_key();
}

// Compute MIC (Message Integrity Code) using SHA256
void compute_mic(const uint8_t* data, size_t data_len, uint8_t mic_out[MIC_SIZE]) 
{
  uint8_t hash[32];
  Sha256_hash(data, data_len, hash);
  memcpy(mic_out, hash, MIC_SIZE);
}

// Constant-time equality comparison (prevents timing attacks)
bool constant_time_eq(const uint8_t* a, const uint8_t* b, size_t len) 
{
  uint8_t result = 0;
  for (size_t i = 0; i < len; i++) result |= a[i] ^ b[i];
  return result == 0;
}

// Verify MIC against expected value
bool verify_mic(const uint8_t* data, size_t data_len, const uint8_t* expected_mic, size_t expected_mic_len) 
{
  if (expected_mic_len != MIC_SIZE) return false;
  uint8_t computed[MIC_SIZE];
  compute_mic(data, data_len, computed);
  return constant_time_eq(computed, expected_mic, MIC_SIZE);
}

// Compute enhanced MIC using HMAC-SHA256
void compute_mic_enhanced(const uint8_t* key, size_t key_len, const uint8_t* data, size_t data_len, uint8_t mic_out[MIC_SIZE_ENHANCED]) 
{
  // Prepare HMAC key (32 bytes)
  uint8_t hmac_key[32];
  memset(hmac_key, 0, 32);
  if (key_len >= 32) memcpy(hmac_key, key, 32);
  else memcpy(hmac_key, key, key_len);
  // Compute HMAC
  uint8_t mac[32];
  HmacSha256_mac(hmac_key, data, data_len, mac);
  // Copy first MIC_SIZE_ENHANCED bytes
  memcpy(mic_out, mac, MIC_SIZE_ENHANCED);
}

// Verify enhanced MIC against expected value
bool verify_mic_enhanced(const uint8_t* key, size_t key_len, const uint8_t* data, size_t data_len, const uint8_t* expected_mic, size_t expected_mic_len) 
{
  if (expected_mic_len != MIC_SIZE_ENHANCED) return false;
  uint8_t computed[MIC_SIZE_ENHANCED];
  compute_mic_enhanced(key, key_len, data, data_len, computed);
  return constant_time_eq(computed, expected_mic, MIC_SIZE_ENHANCED);
}

// Default MIC mode
MicMode MicMode_default(void) 
{
  return MIC_MODE_STANDARD;
}

// PKI encryption: encrypt plaintext for a recipient
// Returns true if successful, false otherwise
bool pki_encrypt(const uint8_t recipient_pubkey[32],const uint8_t sender_privkey[32],const uint8_t* plaintext,size_t plaintext_len,
                const uint8_t nonce[12], Vec_u8_256* output) 
{
  if (plaintext_len + PKI_OVERHEAD > 256) return false;
  // Derive ephemeral seed from nonce and sender private key
  uint8_t ephemeral_seed[32];
  Hkdf_derive((const uint8_t*)"ephemeral", 9, nonce, 12, sender_privkey, 32, ephemeral_seed, 32);
  // Generate ephemeral public key
  uint8_t ephemeral_pubkey[32];
  x25519_base(ephemeral_seed, ephemeral_pubkey);
  // Compute shared secret
  uint8_t shared_secret[32];
  x25519_scalarmult(ephemeral_seed, recipient_pubkey, shared_secret);
  // Derive encryption key
  uint8_t key[32];
  Hkdf_derive((const uint8_t*)"meshtastic-pki", 14, shared_secret, 32, ephemeral_pubkey, 32, key, 32);
  // Encrypt with ChaCha20-Poly1305
  uint8_t ciphertext[240];
  uint8_t tag[16];
  if (plaintext_len > 240) return false;
  ChaCha20Poly1305_seal(key, nonce, NULL, 0, plaintext, plaintext_len, ciphertext, tag);
  // Build output: ephemeral_pubkey || ciphertext || tag
  Vec_u8_256_init(output);
  if (!Vec_u8_256_extend_from_slice(output, ephemeral_pubkey, 32)) return false; 
  if (!Vec_u8_256_extend_from_slice(output, ciphertext, plaintext_len)) return false;
  if (!Vec_u8_256_extend_from_slice(output, tag, 16)) return false;
  return true;
}

// PKI decryption: decrypt encrypted data with recipient's private key
// Returns true if successful, false otherwise
bool pki_decrypt(const uint8_t recipient_privkey[32],const uint8_t* encrypted,size_t encrypted_len,
                const uint8_t nonce[12],Vec_u8_240* output) 
{
  if (encrypted_len < PKI_OVERHEAD) return false;
  // Extract ephemeral public key
  uint8_t ephemeral_pubkey[32];
  memcpy(ephemeral_pubkey, encrypted, 32);
  // Extract ciphertext and tag
  const uint8_t* ciphertext = encrypted + 32;
  size_t ciphertext_len = encrypted_len - 32 - 16;
  uint8_t tag[16];
  memcpy(tag, encrypted + encrypted_len - 16, 16);
  // Compute shared secret
  uint8_t shared_secret[32];
  x25519_scalarmult(recipient_privkey, ephemeral_pubkey, shared_secret);
  // Derive decryption key
  uint8_t key[32];
  Hkdf_derive((const uint8_t*)"meshtastic-pki", 14, shared_secret, 32, ephemeral_pubkey, 32, key, 32);
  // Decrypt with ChaCha20-Poly1305
  uint8_t plaintext[240];
  if (ciphertext_len > 240) return false;
  bool success = ChaCha20Poly1305_open(key, nonce, NULL, 0, ciphertext, ciphertext_len, tag, plaintext);
  if (!success) return false;
  // Build output
  Vec_u8_240_init(output);
  if (!Vec_u8_240_extend_from_slice(output, plaintext, ciphertext_len)) return false;
  return true;
}

// Initialize a new KeyStore
void KeyStore_new(KeyStore* store) 
{
  memset(store, 0, sizeof(KeyStore));
  for (int i = 0; i < MAX_CHANNEL_KEYS; i++) store->channel_keys_present[i] = false;
  store->node_privkey_present = false;
  store->node_pubkey_present = false;
}

// Drop/cleanup a KeyStore (securely zero private key)
void KeyStore_drop(KeyStore* store) 
{
  // Securely zero the private key if present
  if (store->node_privkey_present) secure_zero(store->node_privkey, 32);
  // Note: The Rust comment indicates other keys don't need secure zeroing
}

// Set channel key from raw bytes
void KeyStore_set_channel_key(KeyStore* store, uint8_t index, const uint8_t* key, size_t key_len) 
{
  if (index < MAX_CHANNEL_KEYS) 
  {
    store->channel_keys_data[index] = EncryptionContext_from_key_bytes(key, key_len);
    store->channel_keys_present[index] = true;
  }
}

// Set channel key from channel name
void KeyStore_set_channel_name(KeyStore* store, uint8_t index, const char* name) 
{
  if (index < MAX_CHANNEL_KEYS) 
  {
    store->channel_keys_data[index] = EncryptionContext_from_channel_name(name);
    store->channel_keys_present[index] = true;
  }
}

// Get channel encryption context by index
const EncryptionContext* KeyStore_get_channel(const KeyStore* store, uint8_t index) 
{
  if (index >= MAX_CHANNEL_KEYS) return NULL;
  if (!store->channel_keys_present[index]) return NULL;
  return &store->channel_keys_data[index];
}

// Set node keypair from private key
void KeyStore_set_node_keypair(KeyStore* store, const uint8_t privkey[32]) 
{
  memcpy(store->node_privkey, privkey, 32);
  store->node_privkey_present = true;
  // Generate public key from private key
  x25519_base(privkey, store->node_pubkey);
  store->node_pubkey_present = true;
}

// Generate node keypair from entropy
void KeyStore_generate_node_keypair(KeyStore* store, const uint8_t entropy[32]) 
{
  // Derive private key from entropy
  uint8_t privkey[32];
  Hkdf_derive((const uint8_t*)"node-key", 8, (const uint8_t*)"meshtastic", 10, entropy, 32, privkey, 32);
  KeyStore_set_node_keypair(store, privkey);
}

// Get node public key
const uint8_t* KeyStore_node_pubkey(const KeyStore* store) 
{
  if (!store->node_pubkey_present) return NULL;
  return store->node_pubkey;
}

// Encrypt for a specific node using PKI
bool KeyStore_encrypt_for_node(const KeyStore* store,const uint8_t recipient_pubkey[32],const uint8_t* plaintext,
                              size_t plaintext_len,const uint8_t nonce[12],Vec_u8_256* output) 
{
  if (!store->node_privkey_present) return false;
  return pki_encrypt(recipient_pubkey, store->node_privkey, plaintext, plaintext_len, nonce, output);
}

// Decrypt from a node using PKI
bool KeyStore_decrypt_from_node(const KeyStore* store,const uint8_t* encrypted,size_t encrypted_len,
                                const uint8_t nonce[12],Vec_u8_240* output) 
{
  if (!store->node_privkey_present) return false;
  return pki_decrypt(store->node_privkey, encrypted, encrypted_len, nonce, output);
}

// Create default KeyStore
KeyStore KeyStore_default(void) 
{
  KeyStore store;
  KeyStore_new(&store);
  return store;
}
