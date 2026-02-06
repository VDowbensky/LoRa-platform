#ifndef _CHANNEL_H_
#define _CHANNEL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

// Forward declarations for crypto functions
typedef struct Aes128 Aes128;
typedef struct Aes256 Aes256;
typedef struct Sha256 Sha256;

// Crypto function declarations (these would be in separate crypto headers)
extern void crypto_secure_zero(void* ptr, size_t len);
extern Aes128* aes128_new(const uint8_t key[16]);
extern void aes128_encrypt_ctr(const Aes128* cipher, const uint8_t nonce[16], uint8_t* data, size_t len);
extern void aes128_decrypt_ctr(const Aes128* cipher, const uint8_t nonce[16], uint8_t* data, size_t len);
extern void aes128_free(Aes128* cipher);
extern Aes256* aes256_new(const uint8_t key[32]);
extern void aes256_encrypt_ctr(const Aes256* cipher, const uint8_t nonce[16], uint8_t* data, size_t len);
extern void aes256_decrypt_ctr(const Aes256* cipher, const uint8_t nonce[16], uint8_t* data, size_t len);
extern void aes256_free(Aes256* cipher);
extern void mesh_kdf_derive_nonce(uint32_t packet_id, uint32_t sender, uint8_t nonce[16]);
extern void mesh_kdf_derive_channel_key(const char* name, uint8_t hash[32]);
extern const uint8_t MESH_KDF_DEFAULT_KEY[16];

// Constants
#define MAX_CHANNEL_NAME 12
#define KEY_SIZE_128 16
#define KEY_SIZE_256 32
#define NONCE_SIZE 16
#define MAX_CHANNELS 8
#define MAX_VEC_256 256

// Modem preset enumeration
typedef enum 
{
  MODEM_PRESET_LONG_SLOW = 0,
  MODEM_PRESET_LONG_FAST = 1,
  MODEM_PRESET_LONG_MODERATE = 2,
  MODEM_PRESET_VERY_LONG_SLOW = 3,
  MODEM_PRESET_MEDIUM_SLOW = 4,
  MODEM_PRESET_MEDIUM_FAST = 5,
  MODEM_PRESET_SHORT_SLOW = 6,
  MODEM_PRESET_SHORT_FAST = 7,
  MODEM_PRESET_SHORT_TURBO = 8
} ModemPreset;

// Channel key type enumeration
typedef enum 
{
  CHANNEL_KEY_NONE = 0,
  CHANNEL_KEY_AES128 = 1,
  CHANNEL_KEY_AES256 = 2
} ChannelKeyType;

// Channel key structure
typedef struct 
{
  ChannelKeyType type;
  union 
  {
    uint8_t aes128[KEY_SIZE_128];
    uint8_t aes256[KEY_SIZE_256];
  } key;
} ChannelKey;

// Fixed-capacity vector for uint8_t
typedef struct 
{
  uint8_t data[MAX_CHANNEL_NAME];
  size_t len;
  size_t capacity;
} Vec_u8_12;

// Fixed-capacity vector for uint8_t (256 bytes)
typedef struct 
{
  uint8_t data[MAX_VEC_256];
  size_t len;
  size_t capacity;
} Vec_u8_256;

// Channel structure
typedef struct 
{
  uint8_t index;
  Vec_u8_12 name;
  ChannelKey key;
  ModemPreset modem_preset;
  bool uplink_enabled;
  bool downlink_enabled;
  uint8_t position_precision;
} Channel;

// Optional channel structure
typedef struct 
{
  bool has_value;
  Channel value;
} OptionChannel;

// Channel set structure
typedef struct 
{
  OptionChannel channels[MAX_CHANNELS];
} ChannelSet;

// Base64 decode table
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

// Base64 character table
static const uint8_t BASE64_CHARS[64] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";


// Default for ModemPreset
ModemPreset modem_preset_default(void);
// Calculate airtime in milliseconds for a modem preset
uint32_t modem_preset_airtime_ms(ModemPreset preset, size_t payload_bytes);
// Initialize channel key as None
void channel_key_init_none(ChannelKey* ck);
// Drop/cleanup channel key (securely zero memory)
void channel_key_drop(ChannelKey* ck);
// Clone channel key
void channel_key_clone(const ChannelKey* src, ChannelKey* dst);
// Create channel key from bytes
void channel_key_from_bytes(const uint8_t* key, size_t len, ChannelKey* ck);
// Create default channel key
void channel_key_default(ChannelKey* ck);
// Create channel key from channel name
void channel_key_from_channel_name(const char* name, ChannelKey* ck);
// Check if channel key is encrypted
bool channel_key_is_encrypted(const ChannelKey* ck);
// Get channel key as bytes
const uint8_t* channel_key_as_bytes(const ChannelKey* ck, size_t* len);
// Initialize Vec_u8_12
void vec_u8_12_init(Vec_u8_12* v);
// Clear Vec_u8_12
void vec_u8_12_clear(Vec_u8_12* v);
// Extend Vec_u8_12 from slice
bool vec_u8_12_extend_from_slice(Vec_u8_12* v, const uint8_t* slice, size_t slice_len);
// Initialize Vec_u8_256
void vec_u8_256_init(Vec_u8_256* v);
// Create Vec_u8_256 from slice
bool vec_u8_256_from_slice(Vec_u8_256* v, const uint8_t* slice, size_t slice_len);
// Extend Vec_u8_256 from slice
bool vec_u8_256_extend_from_slice(Vec_u8_256* v, const uint8_t* slice, size_t slice_len);
// Create new channel
void channel_new(uint8_t index, Channel* ch);
// Create primary channel
void channel_primary(Channel* ch);
// Set channel name
void channel_set_name(Channel* ch, const char* name);
// Set channel key
void channel_set_key(Channel* ch, const uint8_t* key, size_t len);
// Encrypt data with channel
bool channel_encrypt(const Channel* ch, uint32_t packet_id, uint32_t sender,
                                   const uint8_t* plaintext, size_t plaintext_len,
                                   Vec_u8_256* ciphertext);

// Decrypt data with channel
bool channel_decrypt(const Channel* ch, uint32_t packet_id, uint32_t sender,
                                   const uint8_t* ciphertext_data, size_t ciphertext_len,
                                   Vec_u8_256* plaintext);
// Calculate channel hash
uint8_t channel_hash(const Channel* ch);
// Get channel name as string
const char* channel_name_str(const Channel* ch);
// Clone channel
void channel_clone(const Channel* src, Channel* dst);
// Default channel (primary)
void channel_default(Channel* ch);
// Drop/cleanup channel
void channel_drop(Channel* ch);
// Initialize optional channel as None
void option_channel_init_none(OptionChannel* opt);
// Initialize optional channel with value
void option_channel_init_some(OptionChannel* opt, const Channel* ch);
// Get reference from optional channel
const Channel* option_channel_as_ref(const OptionChannel* opt);
// Get mutable reference from optional channel
Channel* option_channel_as_mut(OptionChannel* opt);
// Check if optional channel has value
bool option_channel_is_some(const OptionChannel* opt);
// Check if optional channel is none
bool option_channel_is_none(const OptionChannel* opt);
// Drop optional channel
void option_channel_drop(OptionChannel* opt);
// Create new channel set
void channel_set_new(ChannelSet* cs);
// Get channel from set
const Channel* channel_set_get(const ChannelSet* cs, uint8_t index);
// Get mutable channel from set
Channel* channel_set_get_mut(ChannelSet* cs, uint8_t index);
// Set channel in set
void channel_set_set(ChannelSet* cs, uint8_t index, const Channel* channel);
// Get primary channel
const Channel* channel_set_primary(const ChannelSet* cs);
// Get mutable primary channel
Channel* channel_set_primary_mut(ChannelSet* cs);
// Get or initialize primary channel
Channel* channel_set_primary_or_init(ChannelSet* cs);
// Find channel by hash
const Channel* channel_set_find_by_hash(const ChannelSet* cs, uint8_t hash);
// Count channels in set
size_t channel_set_count(const ChannelSet* cs);
// Default channel set
void channel_set_default(ChannelSet* cs);
// Drop/cleanup channel set
void channel_set_drop(ChannelSet* cs);

// Base64 encoding/decoding
// Encode data to base64
size_t base64_encode(const uint8_t* data, size_t data_len, uint8_t* output, size_t output_len);
// Decode base64 data
size_t base64_decode(const uint8_t* data, size_t data_len, uint8_t* output, size_t output_len);

#endif

/*******************************************EOF**************************************/
