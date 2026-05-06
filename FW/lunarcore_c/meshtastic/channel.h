#ifndef _CHANNEL_H_
#define _CHANNEL_H_

// Required dependencies
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "crypto/aes.h"
#include "crypto/sha256.h"
#include "crypto/hkdf.h"
#include "crypto/secure_zero.h"

// Constants
#define MAX_CHANNEL_NAME 12
#define KEY_SIZE_128 16
#define KEY_SIZE_256 32
#define NONCE_SIZE 16
#define MAX_CHANNELS 8
#define MAX_VEC_SIZE 256

// LoraParams structure
typedef struct 
{
  uint8_t spreading_factor;
  uint32_t bandwidth;
  uint8_t coding_rate;
} LoraParams;

// ModemPreset enum
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

// ChannelKey enum type
typedef enum 
{
  CHANNEL_KEY_NONE,
  CHANNEL_KEY_AES128,
  CHANNEL_KEY_AES256
} ChannelKeyType;

// ChannelKey structure (tagged union)
typedef struct 
{
  ChannelKeyType type;
  union 
  {
    uint8_t aes128[KEY_SIZE_128];
    uint8_t aes256[KEY_SIZE_256];
  } data;
} ChannelKey;

// Vec structure (heapless Vec equivalent)
typedef struct 
{
  uint8_t data[MAX_CHANNEL_NAME];
  size_t len;
} VecChannelName;

typedef struct 
{
  uint8_t data[MAX_VEC_SIZE];
  size_t len;
} Vec256;

// Channel structure
typedef struct 
{
  uint8_t index;
  VecChannelName name;
  ChannelKey key;
  ModemPreset modem_preset;
  bool uplink_enabled;
  bool downlink_enabled;
  uint8_t position_precision;
} Channel;

// ChannelSet structure
typedef struct 
{
  Channel* channels[MAX_CHANNELS];
} ChannelSet;





// Forward declarations for crypto functions
typedef struct Aes128 Aes128;
typedef struct Aes256 Aes256;
typedef struct Sha256 Sha256;


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

#endif

/*******************************************EOF**************************************/
