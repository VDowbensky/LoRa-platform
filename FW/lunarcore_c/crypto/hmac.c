/* ## Translation Notes

### Structural Changes:
1. **Module System to Header Files**: Rust's `use super::sha256` is translated to `#include "sha256.h"`, assuming the parent module's SHA256 implementation is in a header file.

2. **Struct Definition**: Rust's `pub struct HmacSha256` is translated to a C typedef struct. The fields `inner` and `outer_key` are preserved exactly as in the original.

3. **Method to Function Translation**: Rust methods are translated to C functions with the struct as the first parameter:
   - `HmacSha256::new(key)` → `hmac_sha256_new(hmac, key, key_len)`
   - `hmac.update(data)` → `hmac_sha256_update(hmac, data, data_len)`
   - `hmac.finalize()` → `hmac_sha256_finalize(hmac, output)`

### Memory Management:
1. **Ownership and Consumption**: The Rust `finalize()` method consumes `self` (takes ownership). In C, we pass a pointer and the caller is responsible for the lifetime of the struct.

2. **Return Values**: Instead of returning arrays by value (as Rust does), C functions take an output buffer parameter (`uint8_t output[DIGEST_SIZE]`).

3. **Array Initialization**: Rust's `[0u8; BLOCK_SIZE]` is translated to `memset()` calls in C for zero initialization.

### Type Translations:
1. **Slices to Pointers**: Rust slices (`&[u8]`) are translated to `const uint8_t*` with an additional `size_t` length parameter.

2. **Boolean**: Rust's `bool` maps directly to C's `bool` from `<stdbool.h>`.

3. **Fixed-size Arrays**: Rust's `[u8; DIGEST_SIZE]` maps to `uint8_t[DIGEST_SIZE]` in C.

### Function-Specific Notes:

#### `new()` function:
- In Rust, this returns a new instance by value. In C, we initialize a struct passed by pointer.
- The key processing logic (hashing if too long, padding if too short) is preserved exactly.
- XOR operations with ipad (0x36) and opad (0x5c) are preserved.

#### `update()` function:
- Simple delegation to the inner SHA256 update function, preserved exactly.

#### `finalize()` function:
- In Rust, this consumes `self`. In C, we accept a pointer and output buffer.
- The two-step hashing process (inner hash, then outer hash) is preserved exactly.

#### `mac()` function:
- One-shot computation function, preserved exactly.
- Creates temporary HMAC instance, updates it, and finalizes.

#### `verify()` function:
- Calls `constant_time_eq()` which is assumed to be defined in sha256.h.
- The function signature assumes `constant_time_eq` takes a length parameter (not present in the original Rust code, but necessary in C).

#### `clone()` function:
- Implemented as a separate function rather than a trait implementation.
- Assumes `sha256_clone()` exists for cloning the inner SHA256 state.
- Uses `memcpy()` for the outer_key array.

### Dependencies Assumed:
The translation assumes the following exist in "sha256.h":
- `Sha256` type definition
- `BLOCK_SIZE` constant (typically 64 for SHA-256)
- `DIGEST_SIZE` constant (typically 32 for SHA-256)
- `sha256_init()` function
- `sha256_update()` function
- `sha256_finalize()` function
- `sha256_hash()` one-shot function
- `sha256_clone()` function
- `constant_time_eq()` function for constant-time comparison

### Security Considerations:
- The constant-time comparison in `verify()` is preserved to prevent timing attacks.
- All HMAC operations follow the standard HMAC construction: H((K ⊕ opad) || H((K ⊕ ipad) || message))
- Key derivation (hashing if too long) is preserved.

### Potential Issues:
1. The caller must ensure that output buffers have sufficient space (DIGEST_SIZE bytes).
2. The `hmac_sha256_finalize()` function modifies the HMAC state; if the caller needs to reuse it, they should clone it first.
3. Thread safety depends on the underlying SHA256 implementation.
 */

#include "sha256.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// HmacSha256 structure definition
typedef struct {
    Sha256 inner;
    uint8_t outer_key[BLOCK_SIZE];
} HmacSha256;

// Initialize a new HmacSha256 instance with the given key
void hmac_sha256_new(HmacSha256* hmac, const uint8_t* key, size_t key_len) {
    uint8_t key_block[BLOCK_SIZE];
    memset(key_block, 0, BLOCK_SIZE);

    // If key is longer than BLOCK_SIZE, hash it first
    if (key_len > BLOCK_SIZE) {
        uint8_t hashed[DIGEST_SIZE];
        sha256_hash(hashed, key, key_len);
        memcpy(key_block, hashed, DIGEST_SIZE);
    } else {
        memcpy(key_block, key, key_len);
    }

    uint8_t inner_key[BLOCK_SIZE];
    memset(inner_key, 0, BLOCK_SIZE);
    memset(hmac->outer_key, 0, BLOCK_SIZE);

    // XOR key_block with ipad (0x36) and opad (0x5c)
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
        inner_key[i] = key_block[i] ^ 0x36;
        hmac->outer_key[i] = key_block[i] ^ 0x5c;
    }

    // Initialize inner hash with inner_key
    sha256_init(&hmac->inner);
    sha256_update(&hmac->inner, inner_key, BLOCK_SIZE);
}

// Update the HMAC with additional data
void hmac_sha256_update(HmacSha256* hmac, const uint8_t* data, size_t data_len) {
    sha256_update(&hmac->inner, data, data_len);
}

// Finalize the HMAC computation and return the MAC
void hmac_sha256_finalize(HmacSha256* hmac, uint8_t output[DIGEST_SIZE]) {
    uint8_t inner_hash[DIGEST_SIZE];
    
    // Finalize the inner hash
    sha256_finalize(&hmac->inner, inner_hash);

    // Compute outer hash: H(outer_key || inner_hash)
    Sha256 outer;
    sha256_init(&outer);
    sha256_update(&outer, hmac->outer_key, BLOCK_SIZE);
    sha256_update(&outer, inner_hash, DIGEST_SIZE);
    sha256_finalize(&outer, output);
}

// One-shot function to compute HMAC-SHA256
void hmac_sha256_mac(uint8_t output[DIGEST_SIZE], const uint8_t* key, size_t key_len,
                     const uint8_t* data, size_t data_len) {
    HmacSha256 hmac;
    hmac_sha256_new(&hmac, key, key_len);
    hmac_sha256_update(&hmac, data, data_len);
    hmac_sha256_finalize(&hmac, output);
}

// Verify an HMAC-SHA256 MAC
bool hmac_sha256_verify(const uint8_t* key, size_t key_len,
                        const uint8_t* data, size_t data_len,
                        const uint8_t expected[DIGEST_SIZE]) {
    uint8_t computed[DIGEST_SIZE];
    hmac_sha256_mac(computed, key, key_len, data, data_len);
    return constant_time_eq(computed, expected, DIGEST_SIZE);
}

// Clone an HmacSha256 instance
void hmac_sha256_clone(HmacSha256* dest, const HmacSha256* src) {
    sha256_clone(&dest->inner, &src->inner);
    memcpy(dest->outer_key, src->outer_key, BLOCK_SIZE);
}
