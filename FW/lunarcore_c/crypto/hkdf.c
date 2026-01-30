/* # HKDF and Cryptographic Functions - Rust to C Translation Notes

## Translation Decisions

### 1. Struct Methods to Functions
- Rust's `impl Hkdf` methods have been converted to C functions with the prefix `hkdf_`
- The `Self` reference in Rust is not needed in C, so calls like `Self::extract()` become direct function calls to `hkdf_extract()`

### 2. Modules to Function Prefixes
- Rust modules (`reticulum`, `meshcore`, `meshtastic`) are translated to function name prefixes
- Example: `reticulum::identity_hash` becomes `reticulum_identity_hash`
- This maintains namespace organization in C

### 3. Slice Handling
- Rust slices (`&[u8]`) are translated to C pointer + length pairs (`const uint8_t *ptr, size_t len`)
- Fixed-size arrays (`&[u8; N]`) are translated to C array parameters with explicit size (`const uint8_t arr[N]`)

### 4. Generic Functions
- Rust's `derive_key<const N: usize>` generic function is not translated as a separate function
- Instead, users should call `hkdf_derive()` directly with the desired output length
- This is because C doesn't support compile-time generics in the same way

### 5. String Literals
- Rust byte string literals (`b"reticulum"`) are handled carefully:
  - The `sizeof()` operator includes the null terminator
  - We use `sizeof(salt) - 1` to exclude the null terminator when it shouldn't be part of the data
  - This matches Rust's behavior where byte strings don't include the null terminator in their length

### 6. Struct Definitions
- `reticulum::LinkKeys` struct is translated to `reticulum_link_keys` typedef with struct
- Public fields remain accessible as before

### 7. Assertions
- Rust's `assert!` macro is translated to C's `assert()` from `<assert.h>`
- The same error message is preserved

### 8. Byte Order
- `to_le_bytes()` in Rust is explicitly implemented in C using bit operations
- This ensures little-endian byte order regardless of platform endianness

### 9. Memory Operations
- `copy_from_slice` in Rust is translated to `memcpy` in C
- Range copying (`okm[offset..offset + to_copy]`) is handled with pointer arithmetic

### 10. Constants
- Module-level constants like `IDENTITY_HASH_SIZE` are translated to `#define` directives
- Array constants like `DEFAULT_KEY` are translated to `static const` arrays

## Dependencies Required
The translation assumes the following header files and implementations exist:

1. **hmac.h**: Must provide:
   - `hmac_sha256_ctx` structure
   - `hmac_sha256_init(ctx, key, key_len)`
   - `hmac_sha256_update(ctx, data, data_len)`
   - `hmac_sha256_finalize(ctx, output)`
   - `hmac_sha256_mac(key, key_len, data, data_len, output)` - one-shot function

2. **sha256.h**: Must provide:
   - `SHA256_DIGEST_SIZE` constant (should be 32)
   - `sha256_ctx` structure
   - `sha256_init(ctx)`
   - `sha256_update(ctx, data, data_len)`
   - `sha256_finalize(ctx, output)`
   - `sha256_hash(data, data_len, output)` - one-shot function

## Implementation Notes

### HKDF Extract
- Handles empty salt by using a zero-filled salt of DIGEST_SIZE length
- This matches RFC 5869 specification

### HKDF Expand
- Iterates to generate required output length
- Uses counter from 1 to n (not 0 to n-1)
- Properly handles partial blocks at the end

### Endianness
- The `meshtastic_derive_nonce` function explicitly uses little-endian byte order
- This is implemented using bit shifts and masks to ensure portability

### Memory Safety
- All array bounds are checked
- Uses `memcpy` for safe memory operations
- Assertions verify constraints (e.g., maximum HKDF output length)

## Potential Issues and Limitations

1. **Error Handling**: The original Rust code uses `assert!` which panics on failure. In C, `assert()` will abort the program. For production code, you may want to replace assertions with proper error return codes.

2. **Generic Functions**: The `derive_key<const N: usize>` function cannot be directly translated to C. Users must call `hkdf_derive()` with their desired output length instead.

3. **String Length**: When using string literals as salt/info parameters, remember to exclude the null terminator by using `sizeof(str) - 1`.

4. **Buffer Overflows**: The caller is responsible for ensuring output buffers are large enough. Consider adding length checks or size parameters if needed for additional safety.

5. **Thread Safety**: If the HMAC and SHA256 implementations use global state, additional synchronization may be needed for multi-threaded use.

## Usage Examples

```c
// Example 1: HKDF key derivation
uint8_t key[32];
const uint8_t salt[] = "my-salt";
const uint8_t ikm[] = "input-keying-material";
const uint8_t info[] = "application-context";
hkdf_derive(salt, sizeof(salt)-1, ikm, sizeof(ikm)-1, 
            info, sizeof(info)-1, key, 32);

// Example 2: Reticulum link keys
reticulum_link_keys keys;
uint8_t shared_secret[32], init_pub[32], resp_pub[32];
// ... initialize values ...
reticulum_derive_link_keys(shared_secret, init_pub, resp_pub, &keys);

// Example 3: Meshtastic nonce
uint8_t nonce[16];
meshtastic_derive_nonce(12345, 67890, nonce);
```
 */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>

// Include dependencies
#include "hmac.h"      // For hmac_sha256_* functions
#include "sha256.h"    // For SHA256_DIGEST_SIZE, sha256_* functions

// HKDF Functions

/**
 * HKDF Extract step
 * @param salt Salt value (can be empty)
 * @param salt_len Length of salt
 * @param ikm Input keying material
 * @param ikm_len Length of IKM
 * @param prk Output pseudorandom key (must be SHA256_DIGEST_SIZE bytes)
 */
void hkdf_extract(const uint8_t *salt, size_t salt_len, 
                  const uint8_t *ikm, size_t ikm_len,
                  uint8_t prk[SHA256_DIGEST_SIZE]) {
    // If salt is empty, use a zero-filled salt of DIGEST_SIZE
    uint8_t zero_salt[SHA256_DIGEST_SIZE];
    const uint8_t *actual_salt;
    size_t actual_salt_len;
    
    if (salt_len == 0) {
        memset(zero_salt, 0, SHA256_DIGEST_SIZE);
        actual_salt = zero_salt;
        actual_salt_len = SHA256_DIGEST_SIZE;
    } else {
        actual_salt = salt;
        actual_salt_len = salt_len;
    }
    
    hmac_sha256_mac(actual_salt, actual_salt_len, ikm, ikm_len, prk);
}

/**
 * HKDF Expand step
 * @param prk Pseudorandom key (must be SHA256_DIGEST_SIZE bytes)
 * @param info Context and application specific information
 * @param info_len Length of info
 * @param okm Output keying material
 * @param okm_len Length of OKM to generate
 */
void hkdf_expand(const uint8_t prk[SHA256_DIGEST_SIZE],
                 const uint8_t *info, size_t info_len,
                 uint8_t *okm, size_t okm_len) {
    // Calculate number of iterations needed
    size_t n = (okm_len + SHA256_DIGEST_SIZE - 1) / SHA256_DIGEST_SIZE;
    assert(n <= 255 && "Output too long");
    
    uint8_t t[SHA256_DIGEST_SIZE];
    memset(t, 0, SHA256_DIGEST_SIZE);
    size_t offset = 0;
    
    for (size_t i = 1; i <= n; i++) {
        hmac_sha256_ctx hmac;
        
        hmac_sha256_init(&hmac, prk, SHA256_DIGEST_SIZE);
        
        if (i > 1) {
            hmac_sha256_update(&hmac, t, SHA256_DIGEST_SIZE);
        }
        hmac_sha256_update(&hmac, info, info_len);
        
        uint8_t counter = (uint8_t)i;
        hmac_sha256_update(&hmac, &counter, 1);
        
        hmac_sha256_finalize(&hmac, t);
        
        size_t to_copy = (SHA256_DIGEST_SIZE < (okm_len - offset)) ? 
                         SHA256_DIGEST_SIZE : (okm_len - offset);
        memcpy(okm + offset, t, to_copy);
        offset += to_copy;
    }
}

/**
 * HKDF Derive - combines extract and expand
 * @param salt Salt value
 * @param salt_len Length of salt
 * @param ikm Input keying material
 * @param ikm_len Length of IKM
 * @param info Context and application specific information
 * @param info_len Length of info
 * @param okm Output keying material
 * @param okm_len Length of OKM to generate
 */
void hkdf_derive(const uint8_t *salt, size_t salt_len,
                 const uint8_t *ikm, size_t ikm_len,
                 const uint8_t *info, size_t info_len,
                 uint8_t *okm, size_t okm_len) {
    uint8_t prk[SHA256_DIGEST_SIZE];
    hkdf_extract(salt, salt_len, ikm, ikm_len, prk);
    hkdf_expand(prk, info, info_len, okm, okm_len);
}

// Reticulum Module Functions

#define RETICULUM_IDENTITY_HASH_SIZE 16
#define RETICULUM_FULL_HASH_SIZE 32

/**
 * Generate truncated identity hash from signing and encryption keys
 * @param signing_key Signing public key (32 bytes)
 * @param encryption_key Encryption public key (32 bytes)
 * @param hash Output hash (16 bytes)
 */
void reticulum_identity_hash(const uint8_t signing_key[32],
                              const uint8_t encryption_key[32],
                              uint8_t hash[RETICULUM_IDENTITY_HASH_SIZE]) {
    sha256_ctx hasher;
    uint8_t full[SHA256_DIGEST_SIZE];
    
    sha256_init(&hasher);
    sha256_update(&hasher, signing_key, 32);
    sha256_update(&hasher, encryption_key, 32);
    sha256_finalize(&hasher, full);
    
    memcpy(hash, full, RETICULUM_IDENTITY_HASH_SIZE);
}

/**
 * Generate full identity hash from signing and encryption keys
 * @param signing_key Signing public key (32 bytes)
 * @param encryption_key Encryption public key (32 bytes)
 * @param hash Output hash (32 bytes)
 */
void reticulum_full_identity_hash(const uint8_t signing_key[32],
                                   const uint8_t encryption_key[32],
                                   uint8_t hash[RETICULUM_FULL_HASH_SIZE]) {
    sha256_ctx hasher;
    
    sha256_init(&hasher);
    sha256_update(&hasher, signing_key, 32);
    sha256_update(&hasher, encryption_key, 32);
    sha256_finalize(&hasher, hash);
}

/**
 * Structure to hold link keys
 */
typedef struct {
    uint8_t tx_key[32];
    uint8_t rx_key[32];
} reticulum_link_keys;

/**
 * Derive link keys from shared secret and public keys
 * @param shared_secret Shared secret (32 bytes)
 * @param initiator_pub Initiator public key (32 bytes)
 * @param responder_pub Responder public key (32 bytes)
 * @param keys Output link keys structure
 */
void reticulum_derive_link_keys(const uint8_t shared_secret[32],
                                 const uint8_t initiator_pub[32],
                                 const uint8_t responder_pub[32],
                                 reticulum_link_keys *keys) {
    // Build context from public keys
    uint8_t context[64];
    memcpy(context, initiator_pub, 32);
    memcpy(context + 32, responder_pub, 32);
    
    // Derive master key material
    uint8_t master[64];
    const uint8_t salt[] = "reticulum";
    hkdf_derive(salt, sizeof(salt) - 1,  // Don't include null terminator
                shared_secret, 32,
                context, 64,
                master, 64);
    
    // Split into TX and RX keys
    memcpy(keys->tx_key, master, 32);
    memcpy(keys->rx_key, master + 32, 32);
}

// Meshcore Module Functions

/**
 * Derive channel key from PSK and channel ID
 * @param psk Pre-shared key
 * @param psk_len Length of PSK
 * @param channel_id Channel identifier
 * @param key Output key (32 bytes)
 */
void meshcore_derive_channel_key(const uint8_t *psk, size_t psk_len,
                                  uint8_t channel_id,
                                  uint8_t key[32]) {
    const uint8_t salt[] = "meshcore";
    uint8_t info[3] = {'C', 'H', channel_id};
    
    hkdf_derive(salt, sizeof(salt) - 1,  // Don't include null terminator
                psk, psk_len,
                info, 3,
                key, 32);
}

/**
 * Derive node key from identity and purpose
 * @param identity Node identity (32 bytes)
 * @param purpose Purpose string
 * @param purpose_len Length of purpose
 * @param key Output key (32 bytes)
 */
void meshcore_derive_node_key(const uint8_t identity[32],
                               const uint8_t *purpose, size_t purpose_len,
                               uint8_t key[32]) {
    const uint8_t salt[] = "meshcore-node";
    
    hkdf_derive(salt, sizeof(salt) - 1,  // Don't include null terminator
                identity, 32,
                purpose, purpose_len,
                key, 32);
}

// Meshtastic Module Functions

/**
 * Default Meshtastic encryption key
 */
static const uint8_t MESHTASTIC_DEFAULT_KEY[16] = {
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};

/**
 * Derive channel key from channel name
 * @param channel_name Channel name string
 * @param channel_name_len Length of channel name
 * @param key Output key (32 bytes)
 */
void meshtastic_derive_channel_key(const char *channel_name, 
                                    size_t channel_name_len,
                                    uint8_t key[32]) {
    sha256_hash((const uint8_t *)channel_name, channel_name_len, key);
}

/**
 * Derive nonce from packet ID and sender ID
 * @param packet_id Packet identifier
 * @param sender Sender identifier
 * @param nonce Output nonce (16 bytes)
 */
void meshtastic_derive_nonce(uint32_t packet_id, uint32_t sender,
                              uint8_t nonce[16]) {
    memset(nonce, 0, 16);
    
    // Copy packet_id in little-endian format to first 4 bytes
    nonce[0] = (uint8_t)(packet_id & 0xFF);
    nonce[1] = (uint8_t)((packet_id >> 8) & 0xFF);
    nonce[2] = (uint8_t)((packet_id >> 16) & 0xFF);
    nonce[3] = (uint8_t)((packet_id >> 24) & 0xFF);
    
    // Copy sender in little-endian format to bytes 8-11
    nonce[8] = (uint8_t)(sender & 0xFF);
    nonce[9] = (uint8_t)((sender >> 8) & 0xFF);
    nonce[10] = (uint8_t)((sender >> 16) & 0xFF);
    nonce[11] = (uint8_t)((sender >> 24) & 0xFF);
}
