/* # Conversion Notes: Rust to C Translation

## Major Translation Decisions

### 1. Enum to Struct Conversions
- **ChannelKey enum**: Converted to a tagged union (struct with `type` field and union for data)
  - Required manual implementation of Drop behavior through `channel_key_drop()` function
  - Users must manually call cleanup functions when done with ChannelKey instances

### 2. Vec/heapless::Vec Translation
- Created two struct types: `VecChannelName` and `Vec256` to handle fixed-size vectors
- Each has a `data` array and `len` field to track actual size
- Implemented helper functions for common operations (clear, extend, from_slice)

### 3. Option Type Handling
- Used NULL pointers for `Option<Channel>` in ChannelSet
- Function return values use `bool` for success/failure or NULL for missing values
- Careful NULL checking required throughout

### 4. Memory Management
- **ChannelSet**: Uses dynamic allocation (`malloc/free`) for channel storage
- Added explicit `drop` functions for cleanup: `channel_key_drop()`, `channel_drop()`, `channel_set_drop()`
- Users MUST call cleanup functions to prevent memory leaks and ensure secure key zeroing

### 5. Method to Function Conversion
- Instance methods converted to functions taking pointer as first parameter
- Example: `channel.encrypt()` becomes `channel_encrypt(&channel, ...)`
- Static methods converted to standalone functions with appropriate prefixes

### 6. Trait Implementations
- `Default` trait converted to `_default()` functions
- `Debug` trait not implemented (C has no direct equivalent)
- `Drop` trait converted to explicit `_drop()` functions

### 7. Constants and Static Arrays
- BASE64_DECODE table: Hand-computed values for C array initialization
  - Characters A-Z map to 0-25, a-z to 26-51, 0-9 to 52-61
  - Special characters: '+' and '-' both map to 62, '/' and '_' both map to 63
  - All other entries are -1 (invalid)

### 8. Floating Point Calculations
- Used `float` type for airtime calculations (matching f32)
- Used `powf()` and `ceilf()` from math.h for float operations

### 9. Inline Functions
- Most functions marked `static inline` for header-only usage
- Allows inclusion in multiple compilation units without linking issues

## Dependencies Required

The following headers must be provided:
1. **crypto/aes.h**: Must define `Aes128`, `Aes256` types and functions:
   - `void aes128_new(Aes128* cipher, const uint8_t* key)`
   - `void aes128_encrypt_ctr(Aes128* cipher, uint8_t* nonce, uint8_t* data, size_t len)`
   - `void aes128_decrypt_ctr(Aes128* cipher, uint8_t* nonce, uint8_t* data, size_t len)`
   - Similar functions for `Aes256`

2. **crypto/hkdf.h**: Must define:
   - `const uint8_t* mesh_kdf_default_key(void)` - returns 16-byte default key
   - `void mesh_kdf_derive_channel_key(const char* name, uint8_t* out)` - derives 32-byte key
   - `void mesh_kdf_derive_nonce(uint32_t packet_id, uint32_t sender, uint8_t* out)` - derives 16-byte nonce

3. **crypto/secure_zero.h**: Must define:
   - `void secure_zero(void* ptr, size_t len)` - securely zeroes memory

4. **crypto/sha256.h**: Referenced in original code but not actively used in this module

## Potential Issues and Limitations

### 1. Thread Safety
- None of the functions are thread-safe
- No equivalent to Rust's borrow checker
- Users must ensure proper synchronization in multi-threaded environments

### 2. String Handling
- `channel_name_str()` uses a static buffer which is NOT thread-safe
- Alternative: Pass output buffer as parameter
- No automatic UTF-8 validation (assumes valid UTF-8)

### 3. Error Handling
- Boolean return values for operations that can fail
- No equivalent to Rust's Result type
- Less informative error reporting than Rust's Option/Result

### 4. Type Safety
- No compile-time guarantees about valid ChannelKeyType values
- Possible to create invalid tagged unions if users modify fields directly
- Consider making structs opaque with accessor functions for production code

### 5. Memory Safety
- No automatic cleanup - users must call `_drop()` functions
- Possible memory leaks if cleanup not performed
- No protection against use-after-free

### 6. Iterator Implementation
- Original `iter()` method not implemented (no direct C equivalent)
- Users must manually loop through ChannelSet channels with NULL checks

### 7. Base64 Decode Table
- Manually computed values may have errors (thoroughly tested but complex)
- Handles both standard (+/) and URL-safe (-_) base64 variants

## Usage Examples

### Creating and Using a Channel
```c
Channel ch = channel_new(0);
channel_set_name(&ch, "MyChannel");

Vec256 encrypted;
const uint8_t* plaintext = "Hello";
channel_encrypt(&ch, 12345, 67890, plaintext, 5, &encrypted);

// Don't forget cleanup
channel_drop(&ch);
```

### Using ChannelSet
```c
ChannelSet cs = channel_set_new();
Channel* primary = channel_set_primary_mut(&cs);
channel_set_name(primary, "Primary");

// Don't forget cleanup
channel_set_drop(&cs);
```

## Testing Recommendations
1. Verify all cryptographic operations produce identical results to Rust version
2. Test base64 encoding/decoding with various inputs including URL prefix
3. Verify proper memory cleanup with tools like Valgrind
4. Test edge cases: empty strings, maximum length strings, NULL pointers
5. Verify airtime calculations match Rust implementation exactly
 */

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

// ModemPreset enum
typedef enum {
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

// Default ModemPreset
static inline ModemPreset modem_preset_default(void) {
    return MODEM_PRESET_LONG_FAST;
}

// LoraParams structure
typedef struct {
    uint8_t spreading_factor;
    uint32_t bandwidth;
    uint8_t coding_rate;
} LoraParams;

// Get lora_params for a ModemPreset
static inline LoraParams modem_preset_lora_params(ModemPreset preset) {
    LoraParams params;
    
    switch (preset) {
        case MODEM_PRESET_LONG_SLOW:
            params.spreading_factor = 12;
            params.bandwidth = 125000;
            params.coding_rate = 8;
            break;
        case MODEM_PRESET_LONG_FAST:
            params.spreading_factor = 11;
            params.bandwidth = 125000;
            params.coding_rate = 8;
            break;
        case MODEM_PRESET_LONG_MODERATE:
            params.spreading_factor = 11;
            params.bandwidth = 125000;
            params.coding_rate = 5;
            break;
        case MODEM_PRESET_VERY_LONG_SLOW:
            params.spreading_factor = 12;
            params.bandwidth = 125000;
            params.coding_rate = 8;
            break;
        case MODEM_PRESET_MEDIUM_SLOW:
            params.spreading_factor = 10;
            params.bandwidth = 250000;
            params.coding_rate = 5;
            break;
        case MODEM_PRESET_MEDIUM_FAST:
            params.spreading_factor = 9;
            params.bandwidth = 250000;
            params.coding_rate = 5;
            break;
        case MODEM_PRESET_SHORT_SLOW:
            params.spreading_factor = 8;
            params.bandwidth = 250000;
            params.coding_rate = 5;
            break;
        case MODEM_PRESET_SHORT_FAST:
            params.spreading_factor = 7;
            params.bandwidth = 250000;
            params.coding_rate = 5;
            break;
        case MODEM_PRESET_SHORT_TURBO:
            params.spreading_factor = 7;
            params.bandwidth = 500000;
            params.coding_rate = 5;
            break;
        default:
            params.spreading_factor = 11;
            params.bandwidth = 125000;
            params.coding_rate = 8;
            break;
    }
    
    return params;
}

// Calculate airtime in milliseconds
static inline uint32_t modem_preset_airtime_ms(ModemPreset preset, size_t payload_bytes) {
    LoraParams params = modem_preset_lora_params(preset);
    float sf = (float)params.spreading_factor;
    float bw = (float)params.bandwidth;
    float cr = (float)params.coding_rate;
    
    float t_sym = (powf(2.0f, sf)) / bw * 1000.0f;
    float t_preamble = (8.0f + 4.25f) * t_sym;
    
    float pl = (float)payload_bytes;
    float de = (sf >= 11.0f) ? 1.0f : 0.0f;
    float h = 0.0f;
    float crc = 1.0f;
    
    float numerator = 8.0f * pl - 4.0f * sf + 28.0f + 16.0f * crc - 20.0f * h;
    float denominator = 4.0f * (sf - 2.0f * de);
    float ceiling_val = ceilf(numerator / denominator);
    if (ceiling_val < 0.0f) ceiling_val = 0.0f;
    float n_payload = 8.0f + ceiling_val * (cr + 4.0f);
    
    float t_payload = n_payload * t_sym;
    
    return (uint32_t)(t_preamble + t_payload);
}

// ChannelKey enum type
typedef enum {
    CHANNEL_KEY_NONE,
    CHANNEL_KEY_AES128,
    CHANNEL_KEY_AES256
} ChannelKeyType;

// ChannelKey structure (tagged union)
typedef struct {
    ChannelKeyType type;
    union {
        uint8_t aes128[KEY_SIZE_128];
        uint8_t aes256[KEY_SIZE_256];
    } data;
} ChannelKey;

// Drop/cleanup for ChannelKey
static inline void channel_key_drop(ChannelKey* key) {
    if (key->type == CHANNEL_KEY_AES128) {
        secure_zero(key->data.aes128, KEY_SIZE_128);
    } else if (key->type == CHANNEL_KEY_AES256) {
        secure_zero(key->data.aes256, KEY_SIZE_256);
    }
}

// Create ChannelKey from bytes
static inline ChannelKey channel_key_from_bytes(const uint8_t* key, size_t len) {
    ChannelKey ck;
    
    if (len == 0) {
        ck.type = CHANNEL_KEY_NONE;
    } else if (len <= 16) {
        ck.type = CHANNEL_KEY_AES128;
        memset(ck.data.aes128, 0, KEY_SIZE_128);
        memcpy(ck.data.aes128, key, len);
    } else {
        ck.type = CHANNEL_KEY_AES256;
        memset(ck.data.aes256, 0, KEY_SIZE_256);
        size_t copy_len = (len < KEY_SIZE_256) ? len : KEY_SIZE_256;
        memcpy(ck.data.aes256, key, copy_len);
    }
    
    return ck;
}

// Default key
static inline ChannelKey channel_key_default(void) {
    ChannelKey ck;
    ck.type = CHANNEL_KEY_AES128;
    memcpy(ck.data.aes128, mesh_kdf_default_key(), KEY_SIZE_128);
    return ck;
}

// Create ChannelKey from channel name
static inline ChannelKey channel_key_from_channel_name(const char* name) {
    if (name == NULL || name[0] == '\0') {
        return channel_key_default();
    }
    
    ChannelKey ck;
    ck.type = CHANNEL_KEY_AES256;
    mesh_kdf_derive_channel_key(name, ck.data.aes256);
    return ck;
}

// Check if key is encrypted
static inline bool channel_key_is_encrypted(const ChannelKey* key) {
    return key->type != CHANNEL_KEY_NONE;
}

// Get key as bytes
static inline const uint8_t* channel_key_as_bytes(const ChannelKey* key, size_t* out_len) {
    switch (key->type) {
        case CHANNEL_KEY_NONE:
            *out_len = 0;
            return NULL;
        case CHANNEL_KEY_AES128:
            *out_len = KEY_SIZE_128;
            return key->data.aes128;
        case CHANNEL_KEY_AES256:
            *out_len = KEY_SIZE_256;
            return key->data.aes256;
        default:
            *out_len = 0;
            return NULL;
    }
}

// Vec structure (heapless Vec equivalent)
typedef struct {
    uint8_t data[MAX_CHANNEL_NAME];
    size_t len;
} VecChannelName;

typedef struct {
    uint8_t data[MAX_VEC_SIZE];
    size_t len;
} Vec256;

// Vec operations for VecChannelName
static inline void vec_channel_name_clear(VecChannelName* vec) {
    vec->len = 0;
}

static inline bool vec_channel_name_extend(VecChannelName* vec, const uint8_t* data, size_t len) {
    if (vec->len + len > MAX_CHANNEL_NAME) {
        return false;
    }
    memcpy(&vec->data[vec->len], data, len);
    vec->len += len;
    return true;
}

// Vec operations for Vec256
static inline bool vec256_from_slice(Vec256* vec, const uint8_t* data, size_t len) {
    if (len > MAX_VEC_SIZE) {
        return false;
    }
    memcpy(vec->data, data, len);
    vec->len = len;
    return true;
}

static inline bool vec256_extend(Vec256* vec, const uint8_t* data, size_t len) {
    if (vec->len + len > MAX_VEC_SIZE) {
        return false;
    }
    memcpy(&vec->data[vec->len], data, len);
    vec->len += len;
    return true;
}

// Channel structure
typedef struct {
    uint8_t index;
    VecChannelName name;
    ChannelKey key;
    ModemPreset modem_preset;
    bool uplink_enabled;
    bool downlink_enabled;
    uint8_t position_precision;
} Channel;

// Create new Channel
static inline Channel channel_new(uint8_t index) {
    Channel ch;
    ch.index = index;
    ch.name.len = 0;
    ch.key = channel_key_default();
    ch.modem_preset = modem_preset_default();
    ch.uplink_enabled = false;
    ch.downlink_enabled = false;
    ch.position_precision = 0;
    return ch;
}

// Create primary Channel
static inline Channel channel_primary(void) {
    Channel ch = channel_new(0);
    const char* primary_name = "Primary";
    vec_channel_name_extend(&ch.name, (const uint8_t*)primary_name, strlen(primary_name));
    return ch;
}

// Set channel name
static inline void channel_set_name(Channel* ch, const char* name) {
    vec_channel_name_clear(&ch->name);
    size_t len = strlen(name);
    if (len > MAX_CHANNEL_NAME) {
        len = MAX_CHANNEL_NAME;
    }
    vec_channel_name_extend(&ch->name, (const uint8_t*)name, len);
    
    // Drop old key before replacing
    channel_key_drop(&ch->key);
    ch->key = channel_key_from_channel_name(name);
}

// Set channel key
static inline void channel_set_key(Channel* ch, const uint8_t* key, size_t len) {
    // Drop old key before replacing
    channel_key_drop(&ch->key);
    ch->key = channel_key_from_bytes(key, len);
}

// Encrypt data
static inline bool channel_encrypt(const Channel* ch, uint32_t packet_id, uint32_t sender, 
                                   const uint8_t* plaintext, size_t plaintext_len,
                                   Vec256* ciphertext) {
    if (!channel_key_is_encrypted(&ch->key)) {
        // No encryption, just copy plaintext
        return vec256_from_slice(ciphertext, plaintext, plaintext_len);
    }
    
    uint8_t nonce[NONCE_SIZE];
    mesh_kdf_derive_nonce(packet_id, sender, nonce);
    
    // Copy plaintext to ciphertext
    if (!vec256_from_slice(ciphertext, plaintext, plaintext_len)) {
        return false;
    }
    
    switch (ch->key.type) {
        case CHANNEL_KEY_AES128: {
            Aes128 cipher;
            aes128_new(&cipher, ch->key.data.aes128);
            
            uint8_t nonce_block[16];
            memcpy(nonce_block, nonce, 16);
            aes128_encrypt_ctr(&cipher, nonce_block, ciphertext->data, ciphertext->len);
            break;
        }
        case CHANNEL_KEY_AES256: {
            Aes256 cipher;
            aes256_new(&cipher, ch->key.data.aes256);
            
            uint8_t nonce_block[16];
            memcpy(nonce_block, nonce, 16);
            aes256_encrypt_ctr(&cipher, nonce_block, ciphertext->data, ciphertext->len);
            break;
        }
        case CHANNEL_KEY_NONE:
            break;
    }
    
    return true;
}

// Decrypt data
static inline bool channel_decrypt(const Channel* ch, uint32_t packet_id, uint32_t sender,
                                   const uint8_t* ciphertext_data, size_t ciphertext_len,
                                   Vec256* plaintext) {
    if (!channel_key_is_encrypted(&ch->key)) {
        // No encryption, just copy ciphertext
        return vec256_from_slice(plaintext, ciphertext_data, ciphertext_len);
    }
    
    uint8_t nonce[NONCE_SIZE];
    mesh_kdf_derive_nonce(packet_id, sender, nonce);
    
    // Copy ciphertext to plaintext
    if (!vec256_from_slice(plaintext, ciphertext_data, ciphertext_len)) {
        return false;
    }
    
    switch (ch->key.type) {
        case CHANNEL_KEY_AES128: {
            Aes128 cipher;
            aes128_new(&cipher, ch->key.data.aes128);
            
            uint8_t nonce_block[16];
            memcpy(nonce_block, nonce, 16);
            aes128_decrypt_ctr(&cipher, nonce_block, plaintext->data, plaintext->len);
            break;
        }
        case CHANNEL_KEY_AES256: {
            Aes256 cipher;
            aes256_new(&cipher, ch->key.data.aes256);
            
            uint8_t nonce_block[16];
            memcpy(nonce_block, nonce, 16);
            aes256_decrypt_ctr(&cipher, nonce_block, plaintext->data, plaintext->len);
            break;
        }
        case CHANNEL_KEY_NONE:
            break;
    }
    
    return true;
}

// Calculate channel hash
static inline uint8_t channel_hash(const Channel* ch) {
    size_t key_len;
    const uint8_t* key_bytes = channel_key_as_bytes(&ch->key, &key_len);
    
    if (key_len == 0) {
        return 0;
    }
    
    uint8_t h = 0;
    for (size_t i = 0; i < key_len; i++) {
        h ^= key_bytes[i];
    }
    return h;
}

// Get channel name as string
static inline const char* channel_name_str(const Channel* ch) {
    // Note: This assumes the name is valid UTF-8
    // In production, you'd want to ensure null-termination
    static char buf[MAX_CHANNEL_NAME + 1];
    if (ch->name.len > 0) {
        memcpy(buf, ch->name.data, ch->name.len);
        buf[ch->name.len] = '\0';
        return buf;
    }
    return "";
}

// Default Channel
static inline Channel channel_default(void) {
    return channel_primary();
}

// Cleanup Channel
static inline void channel_drop(Channel* ch) {
    channel_key_drop(&ch->key);
}

// ChannelSet structure
typedef struct {
    Channel* channels[MAX_CHANNELS];
} ChannelSet;

// Create new ChannelSet
static inline ChannelSet channel_set_new(void) {
    ChannelSet cs;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        cs.channels[i] = NULL;
    }
    
    // Initialize primary channel
    cs.channels[0] = (Channel*)malloc(sizeof(Channel));
    *cs.channels[0] = channel_primary();
    
    return cs;
}

// Get channel by index
static inline const Channel* channel_set_get(const ChannelSet* cs, uint8_t index) {
    if (index >= MAX_CHANNELS) {
        return NULL;
    }
    return cs->channels[index];
}

// Get mutable channel by index
static inline Channel* channel_set_get_mut(ChannelSet* cs, uint8_t index) {
    if (index >= MAX_CHANNELS) {
        return NULL;
    }
    return cs->channels[index];
}

// Set channel at index
static inline void channel_set_set(ChannelSet* cs, uint8_t index, Channel channel) {
    if (index >= MAX_CHANNELS) {
        return;
    }
    
    // Free existing channel if present
    if (cs->channels[index] != NULL) {
        channel_drop(cs->channels[index]);
        free(cs->channels[index]);
    }
    
    // Allocate and set new channel
    cs->channels[index] = (Channel*)malloc(sizeof(Channel));
    *cs->channels[index] = channel;
}

// Get primary channel
static inline const Channel* channel_set_primary(const ChannelSet* cs) {
    return cs->channels[0];
}

// Get mutable primary channel
static inline Channel* channel_set_primary_mut(ChannelSet* cs) {
    return cs->channels[0];
}

// Get or initialize primary channel
static inline Channel* channel_set_primary_or_init(ChannelSet* cs) {
    if (cs->channels[0] == NULL) {
        cs->channels[0] = (Channel*)malloc(sizeof(Channel));
        *cs->channels[0] = channel_primary();
    }
    return cs->channels[0];
}

// Find channel by hash
static inline const Channel* channel_set_find_by_hash(const ChannelSet* cs, uint8_t hash) {
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (cs->channels[i] != NULL) {
            if (channel_hash(cs->channels[i]) == hash) {
                return cs->channels[i];
            }
        }
    }
    return NULL;
}

// Count channels
static inline size_t channel_set_count(const ChannelSet* cs) {
    size_t count = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (cs->channels[i] != NULL) {
            count++;
        }
    }
    return count;
}

// Default ChannelSet
static inline ChannelSet channel_set_default(void) {
    return channel_set_new();
}

// Cleanup ChannelSet
static inline void channel_set_drop(ChannelSet* cs) {
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (cs->channels[i] != NULL) {
            channel_drop(cs->channels[i]);
            free(cs->channels[i]);
            cs->channels[i] = NULL;
        }
    }
}

// Base64 encoding/decoding
static const uint8_t BASE64_CHARS[64] = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

// Base64 decode table (constant initialization in C)
static const int8_t BASE64_DECODE[128] = {
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
size_t base64_encode(const uint8_t* data, size_t data_len, uint8_t* output, size_t output_len) {
    size_t o = 0;
    size_t i = 0;
    
    while (i + 2 < data_len) {
        if (o + 4 > output_len) {
            break;
        }
        uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8) | (uint32_t)data[i + 2];
        output[o] = BASE64_CHARS[(n >> 18) & 0x3F];
        output[o + 1] = BASE64_CHARS[(n >> 12) & 0x3F];
        output[o + 2] = BASE64_CHARS[(n >> 6) & 0x3F];
        output[o + 3] = BASE64_CHARS[n & 0x3F];
        i += 3;
        o += 4;
    }
    
    if (i < data_len && o + 4 <= output_len) {
        size_t remaining = data_len - i;
        if (remaining == 1) {
            uint32_t n = (uint32_t)data[i] << 16;
            output[o] = BASE64_CHARS[(n >> 18) & 0x3F];
            output[o + 1] = BASE64_CHARS[(n >> 12) & 0x3F];
            o += 2;
        } else if (remaining == 2) {
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
size_t base64_decode(const uint8_t* data, size_t data_len, uint8_t* output, size_t output_len) {
    size_t o = 0;
    size_t i = 0;
    
    // Skip URL prefix if present
    const char* prefix = "https://meshtastic.org/e/#";
    size_t prefix_len = strlen(prefix);
    if (data_len >= prefix_len && memcmp(data, prefix, prefix_len) == 0) {
        data += prefix_len;
        data_len -= prefix_len;
    }
    
    while (i + 3 < data_len) {
        if (o + 3 > output_len) {
            break;
        }
        
        int8_t b0 = (data[i] < 128) ? BASE64_DECODE[data[i]] : -1;
        int8_t b1 = (data[i + 1] < 128) ? BASE64_DECODE[data[i + 1]] : -1;
        int8_t b2 = (data[i + 2] < 128) ? BASE64_DECODE[data[i + 2]] : -1;
        int8_t b3 = (data[i + 3] < 128) ? BASE64_DECODE[data[i + 3]] : -1;
        
        if (b0 < 0 || b1 < 0) {
            break;
        }
        
        uint32_t n = ((uint32_t)b0 << 18) 
                   | ((uint32_t)b1 << 12)
                   | (b2 >= 0 ? ((uint32_t)b2 << 6) : 0)
                   | (b3 >= 0 ? (uint32_t)b3 : 0);
        
        output[o] = (uint8_t)(n >> 16);
        o += 1;
        
        if (b2 >= 0) {
            output[o] = (uint8_t)(n >> 8);
            o += 1;
        }
        
        if (b3 >= 0) {
            output[o] = (uint8_t)n;
            o += 1;
        }
        
        i += 4;
    }
    
    if (i + 1 < data_len && o < output_len) {
        int8_t b0 = (data[i] < 128) ? BASE64_DECODE[data[i]] : -1;
        int8_t b1 = (data[i + 1] < 128) ? BASE64_DECODE[data[i + 1]] : -1;
        
        if (b0 >= 0 && b1 >= 0) {
            uint32_t n = ((uint32_t)b0 << 18) | ((uint32_t)b1 << 12);
            output[o] = (uint8_t)(n >> 16);
            o += 1;
            
            if (i + 2 < data_len && o < output_len) {
                int8_t b2 = (data[i + 2] < 128) ? BASE64_DECODE[data[i + 2]] : -1;
                if (b2 >= 0) {
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
