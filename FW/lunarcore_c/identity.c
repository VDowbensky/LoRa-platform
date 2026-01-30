/* # Conversion Notes: Rust to C Translation

## Major Translation Decisions

### 1. Type Conversions
- **Rust arrays** → **C arrays**: Direct translation with explicit sizes
- **Rust slices** → **C pointers with length**: Used explicit pointer and size_t parameters
- **Option<T>** → **NULL pointers or bool return values**: 
  - For pointer returns: NULL indicates None
  - For value returns: Added bool return value with output parameter
- **Result<T, E>** → **bool return values**: true for success, false for error

### 2. Memory Management
- **Drop trait** → **Explicit cleanup functions**: Created `identity_cleanup()` and `identity_manager_cleanup()` functions
- **Ownership** → **Manual memory management**: Used malloc/free for heap allocations
- **RAII** → **Manual cleanup**: Callers must explicitly free resources
- **secure_zero** → **secure_zero function call**: Assumes external implementation

### 3. Error Handling
- **panic!()** → **abort()**: Unrecoverable errors call abort()
- **Option::unwrap()** → **NULL checks**: Explicit NULL pointer checks
- **Option combinators** → **if statements**: Replaced and_then(), map() with explicit conditionals

### 4. Concurrency
- **AtomicBool** → **atomic_bool (C11)**: Direct translation using C11 atomics
- **Ordering::SeqCst** → **memory_order_seq_cst**: C11 equivalent ordering
- **swap()** → **atomic_compare_exchange_strong()**: More explicit atomic operation

### 5. Platform-Specific Code
- **#[cfg(target_arch = "xtensa")]** → **#ifdef __XTENSA__**: Direct conditional compilation
- **unsafe blocks** → **Normal C code**: C is inherently unsafe
- **volatile reads** → **volatile pointer access**: Direct memory-mapped I/O access

### 6. Crypto Operations
- Assumed external implementations for:
  - `ed25519_sign()`, `ed25519_verify()`, `ed25519_public_key()`
  - `x25519()`, `x25519_base()`
  - `sha256_hash()`
  - `chacha20_new()`, `chacha20_encrypt()`, `chacha20_decrypt()`
  - `secure_zero()`, `constant_time_eq()`
  - `fill_random_checked()`

### 7. Global State
- **static mut** → **static pointer**: Used static pointer with atomic initialization
- **lazy initialization** → **init function**: Explicit `identity_init()` call required

## Key Implementation Details

### String and Byte Handling
- Used `memcpy()` for array copying
- Used `strlen()` for C string length (for context strings)
- Used explicit byte indexing for endian conversions

### Function Signatures
- Methods → Functions with explicit `self` parameter
- References → Pointers (const where appropriate)
- Return values → Output parameters for complex types

### Constants
- `const` variables → `#define` or `static const`
- Byte string literals (b"...") → Regular C string literals

### ESP-IDF Integration
- Direct NVS API calls for ESP32 flash storage
- Conditional compilation preserves platform independence

## Potential Issues and Limitations

1. **Thread Safety**: The global `IDENTITY_MANAGER` assumes single initialization. Multiple concurrent calls to `identity_init()` are handled by atomic compare-exchange.

2. **Memory Leaks**: The global `IDENTITY_MANAGER` is never freed. Consider adding a cleanup function for proper shutdown.

3. **Error Handling**: Many error conditions that would return Result in Rust now use bool or abort(). Callers must check return values.

4. **Type Safety**: C lacks Rust's type safety. Callers must ensure correct buffer sizes.

5. **Dependencies**: Requires external crypto library implementations and RNG functions.

6. **ESP-IDF Version**: NVS API calls assume ESP-IDF v4.x or later.

## Testing Recommendations

1. Verify all crypto function signatures match your crypto library
2. Test on both ESP32 (xtensa) and non-ESP32 platforms
3. Verify secure_zero() properly clears sensitive data
4. Test concurrent initialization scenarios
5. Verify NVS operations on actual ESP32 hardware
6. Test all Option-returning functions for proper NULL handling

## Required Headers

The following headers must be provided or implemented:
- `crypto/ed25519.h`: Ed25519 signature types and functions
- `crypto/x25519.h`: X25519 key exchange functions
- `crypto/sha256.h`: SHA-256 hashing
- `crypto/chacha20.h`: ChaCha20 encryption
- `crypto/secure.h`: secure_zero, constant_time_eq
- `rng.h`: fill_random_checked function

For ESP32 builds, ESP-IDF headers are required:
- `nvs_flash.h`
- `nvs.h`
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>

// Include necessary crypto headers
#include "crypto/ed25519.h"
#include "crypto/x25519.h"
#include "crypto/sha256.h"
#include "crypto/chacha20.h"
#include "crypto/secure.h"
#include "rng.h"

// For ESP32 target
#ifdef __XTENSA__
#include "nvs_flash.h"
#include "nvs.h"
#endif

// Constants
#define SEED_SIZE 32
#define NODE_ID_SIZE 4
#define SHORT_ID_SIZE 8

static const char IDENTITY_FLASH_KEY[] = "lunar_id";
static const uint8_t KDF_CONTEXT_IDENTITY[] = "LunarCore Identity v1";
static const uint8_t KDF_CONTEXT_SIGNING[] = "LunarCore Signing v1";
static const uint8_t KDF_CONTEXT_ENCRYPTION[] = "LunarCore Encryption v1";

// Identity structure
typedef struct {
    PrivateKey signing_key;
    PublicKey public_key;
    uint8_t encryption_key[32];
    uint8_t encryption_public[32];
    uint32_t created_at;
} Identity;

// EncryptedIdentity structure
typedef struct {
    uint8_t ciphertext[128];
    uint8_t nonce[NONCE_SIZE];
    uint8_t tag[32];
} EncryptedIdentity;

// IdentityManager structure
typedef struct {
    Identity* current;
    uint8_t storage_key[32];
} IdentityManager;

// Forward declarations
static void identity_cleanup(Identity* identity);
static void identity_manager_cleanup(IdentityManager* manager);
static uint8_t hex_char(uint8_t nibble);

// Drop implementation for Identity
static void identity_cleanup(Identity* identity) {
    if (identity != NULL) {
        secure_zero(&identity->signing_key, sizeof(identity->signing_key));
        secure_zero(identity->encryption_key, sizeof(identity->encryption_key));
    }
}

// Identity::derive_key
static void identity_derive_key(const uint8_t seed[SEED_SIZE], const uint8_t* context, size_t context_len, uint8_t output[32]) {
    uint8_t input[64];
    memset(input, 0, sizeof(input));
    memcpy(input, seed, 32);
    size_t copy_len = context_len < 32 ? context_len : 32;
    memcpy(input + 32, context, copy_len);
    sha256_hash(input, sizeof(input), output);
}

// Identity::generate
Identity identity_generate(void) {
    uint8_t seed[SEED_SIZE];
    memset(seed, 0, sizeof(seed));
    
    if (!fill_random_checked(seed, sizeof(seed))) {
        // panic equivalent
        abort(); // "RNG health check failed - cannot generate identity with weak entropy"
    }
    
    return identity_from_seed(seed);
}

// Identity::from_seed
Identity identity_from_seed(const uint8_t seed[SEED_SIZE]) {
    Identity identity;
    memset(&identity, 0, sizeof(identity));
    
    // Derive signing key
    identity_derive_key(seed, KDF_CONTEXT_SIGNING, strlen((const char*)KDF_CONTEXT_SIGNING), identity.signing_key);
    
    // Derive public key
    ed25519_public_key(&identity.signing_key, &identity.public_key);
    
    // Derive encryption key
    identity_derive_key(seed, KDF_CONTEXT_ENCRYPTION, strlen((const char*)KDF_CONTEXT_ENCRYPTION), identity.encryption_key);
    
    // Derive encryption public key
    x25519_base(identity.encryption_key, identity.encryption_public);
    
    identity.created_at = 0;
    
    return identity;
}

// Identity::public_key
const PublicKey* identity_public_key(const Identity* identity) {
    return &identity->public_key;
}

// Identity::encryption_public_key
const uint8_t* identity_encryption_public_key(const Identity* identity) {
    return identity->encryption_public;
}

// Identity::node_id
uint32_t identity_node_id(const Identity* identity) {
    uint8_t hash[32];
    sha256_hash((const uint8_t*)&identity->public_key, sizeof(PublicKey), hash);
    
    uint32_t node_id = ((uint32_t)hash[0]) |
                       ((uint32_t)hash[1] << 8) |
                       ((uint32_t)hash[2] << 16) |
                       ((uint32_t)hash[3] << 24);
    return node_id;
}

// Identity::short_id
void identity_short_id(const Identity* identity, uint8_t short_id[SHORT_ID_SIZE]) {
    uint8_t hash[32];
    sha256_hash((const uint8_t*)&identity->public_key, sizeof(PublicKey), hash);
    
    for (int i = 0; i < 4; i++) {
        uint8_t byte = hash[i];
        short_id[i * 2] = hex_char(byte >> 4);
        short_id[i * 2 + 1] = hex_char(byte & 0x0F);
    }
}

// Identity::sign
Signature identity_sign(const Identity* identity, const uint8_t* message, size_t message_len) {
    return ed25519_sign(&identity->signing_key, message, message_len);
}

// Identity::verify
bool identity_verify(const PublicKey* public_key, const uint8_t* message, size_t message_len, const Signature* signature) {
    return ed25519_verify(public_key, message, message_len, signature);
}

// Identity::key_agree
void identity_key_agree(const Identity* identity, const uint8_t their_public[32], uint8_t output[32]) {
    x25519(identity->encryption_key, their_public, output);
}

// Identity::encrypt_for_storage
EncryptedIdentity identity_encrypt_for_storage(const Identity* identity, const uint8_t storage_key[32]) {
    EncryptedIdentity encrypted;
    memset(&encrypted, 0, sizeof(encrypted));
    
    // Prepare plaintext
    uint8_t plaintext[128];
    memset(plaintext, 0, sizeof(plaintext));
    memcpy(plaintext, identity->signing_key, 32);
    memcpy(plaintext + 32, identity->encryption_key, 32);
    
    // Copy created_at as little-endian bytes
    plaintext[64] = (identity->created_at) & 0xFF;
    plaintext[65] = (identity->created_at >> 8) & 0xFF;
    plaintext[66] = (identity->created_at >> 16) & 0xFF;
    plaintext[67] = (identity->created_at >> 24) & 0xFF;
    
    // Generate nonce
    if (!fill_random_checked(encrypted.nonce, NONCE_SIZE)) {
        // panic equivalent
        abort(); // "RNG health check failed - cannot encrypt identity with weak nonce"
    }
    
    // Encrypt
    ChaCha20 cipher = chacha20_new(storage_key, encrypted.nonce);
    memcpy(encrypted.ciphertext, plaintext, 128);
    chacha20_encrypt(&cipher, encrypted.ciphertext, 128);
    
    // Compute tag
    uint8_t tag_input[128 + NONCE_SIZE];
    memcpy(tag_input, encrypted.ciphertext, 128);
    memcpy(tag_input + 128, encrypted.nonce, NONCE_SIZE);
    sha256_hash(tag_input, sizeof(tag_input), encrypted.tag);
    
    return encrypted;
}

// Identity::decrypt_from_storage
bool identity_decrypt_from_storage(const EncryptedIdentity* encrypted, const uint8_t storage_key[32], Identity* output) {
    // Verify tag
    uint8_t tag_input[128 + NONCE_SIZE];
    memcpy(tag_input, encrypted->ciphertext, 128);
    memcpy(tag_input + 128, encrypted->nonce, NONCE_SIZE);
    
    uint8_t expected_tag[32];
    sha256_hash(tag_input, sizeof(tag_input), expected_tag);
    
    if (!constant_time_eq(encrypted->tag, expected_tag, 32)) {
        return false;
    }
    
    // Decrypt
    ChaCha20 cipher = chacha20_new(storage_key, encrypted->nonce);
    uint8_t plaintext[128];
    memcpy(plaintext, encrypted->ciphertext, 128);
    chacha20_decrypt(&cipher, plaintext, 128);
    
    // Extract keys and data
    memset(output, 0, sizeof(Identity));
    memcpy(output->signing_key, plaintext, 32);
    memcpy(output->encryption_key, plaintext + 32, 32);
    
    output->created_at = ((uint32_t)plaintext[64]) |
                        ((uint32_t)plaintext[65] << 8) |
                        ((uint32_t)plaintext[66] << 16) |
                        ((uint32_t)plaintext[67] << 24);
    
    // Derive public keys
    ed25519_public_key(output->signing_key, &output->public_key);
    x25519_base(output->encryption_key, output->encryption_public);
    
    // Secure zero plaintext
    secure_zero(plaintext, sizeof(plaintext));
    
    return true;
}

// Identity::export_seed
void identity_export_seed(const Identity* identity, uint8_t seed[SEED_SIZE]) {
    memcpy(seed, identity->signing_key, SEED_SIZE);
}

// EncryptedIdentity::to_bytes
void encrypted_identity_to_bytes(const EncryptedIdentity* encrypted, uint8_t bytes[128 + NONCE_SIZE + 32]) {
    memcpy(bytes, encrypted->ciphertext, 128);
    memcpy(bytes + 128, encrypted->nonce, NONCE_SIZE);
    memcpy(bytes + 128 + NONCE_SIZE, encrypted->tag, 32);
}

// EncryptedIdentity::from_bytes
EncryptedIdentity encrypted_identity_from_bytes(const uint8_t bytes[128 + NONCE_SIZE + 32]) {
    EncryptedIdentity encrypted;
    
    memcpy(encrypted.ciphertext, bytes, 128);
    memcpy(encrypted.nonce, bytes + 128, NONCE_SIZE);
    memcpy(encrypted.tag, bytes + 128 + NONCE_SIZE, 32);
    
    return encrypted;
}

// Drop implementation for IdentityManager
static void identity_manager_cleanup(IdentityManager* manager) {
    if (manager != NULL) {
        secure_zero(manager->storage_key, sizeof(manager->storage_key));
        if (manager->current != NULL) {
            identity_cleanup(manager->current);
            free(manager->current);
            manager->current = NULL;
        }
    }
}

// IdentityManager::derive_storage_key
static void identity_manager_derive_storage_key(uint8_t storage_key[32]) {
    uint8_t efuse_data[32];
    memset(efuse_data, 0, sizeof(efuse_data));
    
#ifdef __XTENSA__
    // Read eFuse data on ESP32
    volatile uint32_t* efuse_base = (volatile uint32_t*)0x6001A000;
    for (int i = 0; i < 8; i++) {
        uint32_t val = efuse_base[i];
        efuse_data[i * 4] = val & 0xFF;
        efuse_data[i * 4 + 1] = (val >> 8) & 0xFF;
        efuse_data[i * 4 + 2] = (val >> 16) & 0xFF;
        efuse_data[i * 4 + 3] = (val >> 24) & 0xFF;
    }
#else
    // Fallback for non-ESP32 platforms
    memset(efuse_data, 0, sizeof(efuse_data));
#endif
    
    uint8_t kdf_input[64];
    memcpy(kdf_input, efuse_data, 32);
    memcpy(kdf_input + 32, "LunarCore Storage Key v1\0\0\0\0\0\0\0\0", 32);
    
    sha256_hash(kdf_input, sizeof(kdf_input), storage_key);
}

// IdentityManager::new
IdentityManager identity_manager_new(void) {
    IdentityManager manager;
    manager.current = NULL;
    identity_manager_derive_storage_key(manager.storage_key);
    return manager;
}

// IdentityManager::load_from_flash
static bool identity_manager_load_from_flash(const IdentityManager* manager, Identity* identity) {
#ifdef __XTENSA__
    nvs_handle_t handle = 0;
    const char* namespace = "lunar";
    
    esp_err_t ret = nvs_open(namespace, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return false;
    }
    
    const char* key = "identity";
    size_t size = 128 + NONCE_SIZE + 32;
    uint8_t bytes[128 + NONCE_SIZE + 32];
    
    ret = nvs_get_blob(handle, key, bytes, &size);
    nvs_close(handle);
    
    if (ret != ESP_OK || size != sizeof(bytes)) {
        return false;
    }
    
    EncryptedIdentity encrypted = encrypted_identity_from_bytes(bytes);
    return identity_decrypt_from_storage(&encrypted, manager->storage_key, identity);
#else
    (void)manager;
    (void)identity;
    return false;
#endif
}

// IdentityManager::save_to_flash
static void identity_manager_save_to_flash(const IdentityManager* manager, const Identity* identity) {
    EncryptedIdentity encrypted = identity_encrypt_for_storage(identity, manager->storage_key);
    uint8_t bytes[128 + NONCE_SIZE + 32];
    encrypted_identity_to_bytes(&encrypted, bytes);
    
#ifdef __XTENSA__
    nvs_handle_t handle = 0;
    const char* namespace = "lunar";
    
    esp_err_t ret = nvs_open(namespace, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return;
    }
    
    const char* key = "identity";
    nvs_set_blob(handle, key, bytes, sizeof(bytes));
    nvs_commit(handle);
    nvs_close(handle);
#else
    (void)bytes;
#endif
}

// IdentityManager::init
const Identity* identity_manager_init(IdentityManager* manager) {
    Identity loaded_identity;
    
    if (identity_manager_load_from_flash(manager, &loaded_identity)) {
        manager->current = (Identity*)malloc(sizeof(Identity));
        if (manager->current == NULL) {
            abort(); // Out of memory
        }
        memcpy(manager->current, &loaded_identity, sizeof(Identity));
    } else {
        // Generate new identity
        Identity new_identity = identity_generate();
        identity_manager_save_to_flash(manager, &new_identity);
        manager->current = (Identity*)malloc(sizeof(Identity));
        if (manager->current == NULL) {
            abort(); // Out of memory
        }
        memcpy(manager->current, &new_identity, sizeof(Identity));
    }
    
    return manager->current;
}

// IdentityManager::current
const Identity* identity_manager_current(const IdentityManager* manager) {
    return manager->current;
}

// IdentityManager::rotate
const Identity* identity_manager_rotate(IdentityManager* manager) {
    Identity new_identity = identity_generate();
    identity_manager_save_to_flash(manager, &new_identity);
    
    if (manager->current != NULL) {
        identity_cleanup(manager->current);
        free(manager->current);
    }
    
    manager->current = (Identity*)malloc(sizeof(Identity));
    if (manager->current == NULL) {
        abort(); // Out of memory
    }
    memcpy(manager->current, &new_identity, sizeof(Identity));
    
    return manager->current;
}

// IdentityManager::import_seed
const Identity* identity_manager_import_seed(IdentityManager* manager, const uint8_t seed[SEED_SIZE]) {
    Identity new_identity = identity_from_seed(seed);
    identity_manager_save_to_flash(manager, &new_identity);
    
    if (manager->current != NULL) {
        identity_cleanup(manager->current);
        free(manager->current);
    }
    
    manager->current = (Identity*)malloc(sizeof(Identity));
    if (manager->current == NULL) {
        abort(); // Out of memory
    }
    memcpy(manager->current, &new_identity, sizeof(Identity));
    
    return manager->current;
}

// hex_char helper function
static uint8_t hex_char(uint8_t nibble) {
    if (nibble <= 9) {
        return '0' + nibble;
    } else if (nibble <= 15) {
        return 'a' + (nibble - 10);
    } else {
        return '?';
    }
}

// Global identity manager
static IdentityManager* IDENTITY_MANAGER = NULL;
static atomic_bool IDENTITY_INIT = ATOMIC_VAR_INIT(false);

// Global init function
void identity_init(void) {
    bool expected = false;
    if (atomic_compare_exchange_strong(&IDENTITY_INIT, &expected, true)) {
        IDENTITY_MANAGER = (IdentityManager*)malloc(sizeof(IdentityManager));
        if (IDENTITY_MANAGER == NULL) {
            abort(); // Out of memory
        }
        *IDENTITY_MANAGER = identity_manager_new();
        identity_manager_init(IDENTITY_MANAGER);
    }
}

// Global node_id function
uint32_t global_node_id(void) {
    identity_init();
    const Identity* current = identity_manager_current(IDENTITY_MANAGER);
    if (current != NULL) {
        return identity_node_id(current);
    }
    return 0;
}

// Global public_key function
bool global_public_key(PublicKey* output) {
    identity_init();
    const Identity* current = identity_manager_current(IDENTITY_MANAGER);
    if (current != NULL) {
        memcpy(output, identity_public_key(current), sizeof(PublicKey));
        return true;
    }
    return false;
}

// Global encryption_public_key function
bool global_encryption_public_key(uint8_t output[32]) {
    identity_init();
    const Identity* current = identity_manager_current(IDENTITY_MANAGER);
    if (current != NULL) {
        memcpy(output, identity_encryption_public_key(current), 32);
        return true;
    }
    return false;
}

// Global sign function
bool global_sign(const uint8_t* message, size_t message_len, Signature* output) {
    identity_init();
    const Identity* current = identity_manager_current(IDENTITY_MANAGER);
    if (current != NULL) {
        *output = identity_sign(current, message, message_len);
        return true;
    }
    return false;
}

// Global key_agree function
bool global_key_agree(const uint8_t their_public[32], uint8_t output[32]) {
    identity_init();
    const Identity* current = identity_manager_current(IDENTITY_MANAGER);
    if (current != NULL) {
        identity_key_agree(current, their_public, output);
        return true;
    }
    return false;
}

// Global rotate function
bool global_rotate(uint32_t* output_node_id) {
    identity_init();
    if (IDENTITY_MANAGER != NULL) {
        const Identity* new_identity = identity_manager_rotate(IDENTITY_MANAGER);
        if (output_node_id != NULL) {
            *output_node_id = identity_node_id(new_identity);
        }
        return true;
    }
    return false;
}
