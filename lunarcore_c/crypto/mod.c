/* ## Translation Decisions and Notes

### Module System Translation
1. **Rust modules to C headers**: Rust's `pub mod` declarations are translated to `#include` statements for corresponding header files (e.g., `pub mod aes;` becomes `#include "aes.h"`)
2. **Re-exports**: Rust's `pub use` statements are handled by:
   - Including the necessary headers
   - Using typedefs to re-expose types at this level (for clarity and consistency)
   - Noting that function re-exports (x25519, x25519_base) are already declared in their respective headers

### Function Translations

#### constant_time_eq
- **Rust slice parameters**: Translated `&[u8]` to `const uint8_t *` with separate `size_t` length parameters
- **Functionality**: Preserved exact constant-time comparison behavior
- **Return type**: Maintained as `bool`

#### secure_zero and secure_zero_u32
- **Volatile writes**: Translated `core::ptr::write_volatile` using volatile-qualified pointers in C
- **Compiler fence**: Translated `core::sync::atomic::compiler_fence(Ordering::SeqCst)` to platform-specific implementations:
  - C11: `atomic_thread_fence(memory_order_seq_cst)`
  - GCC/Clang: `__asm__ __volatile__("" ::: "memory")`
  - MSVC: `_ReadWriteBarrier()`
  - Fallback: The volatile accesses provide basic protection
- **Slice parameters**: Translated to pointer + length pattern
- **Inline preservation**: Maintained `static inline` for header-defined functions

#### Random Functions (random_bytes, random_bytes_checked, rng_is_healthy)
- **Delegated to rng module**: These functions call into the `rng` module (fill_random, fill_random_checked, is_healthy)
- **Implementation in .c file**: Since these are not inline, they're implemented in the .c file
- **Return types preserved**: bool for checked functions

### Type System Differences
1. **Slices**: All Rust slice types (`&[u8]`, `&mut [u8]`) converted to pointer + length pattern
2. **Inline functions**: Rust's `#[inline]` attribute translated to C's `static inline` for header-defined functions
3. **Boolean type**: Using C99's `stdbool.h` for native bool type

### File Structure
- **Header file (crypto.h)**: Contains all type definitions, includes, and inline function definitions
- **Implementation file (crypto.c)**: Contains non-inline function implementations that delegate to the rng module

### Security Considerations
1. **Volatile writes**: Properly implemented to prevent compiler optimization of security-critical zeroing
2. **Memory barriers**: Compiler fences ensure memory operations complete in the correct order
3. **Constant-time comparison**: Maintains timing-attack resistance from original code

### Dependencies
The translated code requires:
- Standard C headers: `stdint.h`, `stddef.h`, `stdbool.h`, `stdatomic.h` (for C11)
- Crypto module headers: `aes.h`, `sha256.h`, `hmac.h`, `x25519.h`, `ed25519.h`, `chacha20.h`, `poly1305.h`, `hkdf.h`
- RNG module header: `rng.h` with functions `fill_random()`, `fill_random_checked()`, `is_healthy()`

### Portability Notes
- The compiler fence implementation provides multiple fallback options for different compilers
- C11 support is optional but recommended for the standard atomic fence
- The code will work with C99 or later (C11 recommended for atomic operations)
 */

// crypto.h
#ifndef CRYPTO_H
#define CRYPTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Include all crypto module headers
#include "aes.h"
#include "sha256.h"
#include "hmac.h"
#include "x25519.h"
#include "ed25519.h"
#include "chacha20.h"
#include "poly1305.h"
#include "hkdf.h"

// Re-export types (using typedefs for clarity)
typedef Aes128 Aes128;
typedef Aes256 Aes256;
typedef AesMode AesMode;
typedef Sha256 Sha256;
typedef HmacSha256 HmacSha256;
typedef Ed25519 Ed25519;
typedef Signature Signature;
typedef ChaCha20 ChaCha20;
typedef Poly1305 Poly1305;
typedef Hkdf Hkdf;

// X25519 functions (re-exported)
// Note: These are declared in x25519.h and we're just re-exposing them here
// void x25519(uint8_t out[32], const uint8_t scalar[32], const uint8_t point[32]);
// void x25519_base(uint8_t out[32], const uint8_t scalar[32]);

// Constant-time comparison function
static inline bool constant_time_eq(const uint8_t *a, size_t a_len, const uint8_t *b, size_t b_len) {
    // Check if lengths are equal
    if (a_len != b_len) {
        return false;
    }
    
    // Perform constant-time comparison
    uint8_t result = 0;
    for (size_t i = 0; i < a_len; i++) {
        result |= a[i] ^ b[i];
    }
    
    return result == 0;
}

// Securely zero memory (prevents compiler optimization)
static inline void secure_zero(uint8_t *data, size_t len) {
    // Use volatile pointer to prevent compiler optimization
    volatile uint8_t *volatile_ptr = (volatile uint8_t *)data;
    
    for (size_t i = 0; i < len; i++) {
        volatile_ptr[i] = 0;
    }
    
    // Compiler fence to ensure writes complete before continuing
    #if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
        // C11 atomic fence
        #include <stdatomic.h>
        atomic_thread_fence(memory_order_seq_cst);
    #elif defined(__GNUC__) || defined(__clang__)
        // GCC/Clang builtin
        __asm__ __volatile__("" ::: "memory");
    #elif defined(_MSC_VER)
        // MSVC intrinsic
        _ReadWriteBarrier();
    #else
        // Fallback: volatile access should still work
        (void)volatile_ptr;
    #endif
}

// Securely zero u32 array (prevents compiler optimization)
static inline void secure_zero_u32(uint32_t *data, size_t len) {
    // Use volatile pointer to prevent compiler optimization
    volatile uint32_t *volatile_ptr = (volatile uint32_t *)data;
    
    for (size_t i = 0; i < len; i++) {
        volatile_ptr[i] = 0;
    }
    
    // Compiler fence to ensure writes complete before continuing
    #if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
        // C11 atomic fence
        #include <stdatomic.h>
        atomic_thread_fence(memory_order_seq_cst);
    #elif defined(__GNUC__) || defined(__clang__)
        // GCC/Clang builtin
        __asm__ __volatile__("" ::: "memory");
    #elif defined(_MSC_VER)
        // MSVC intrinsic
        _ReadWriteBarrier();
    #else
        // Fallback: volatile access should still work
        (void)volatile_ptr;
    #endif
}

// Fill buffer with random bytes (assumes rng module exists)
void random_bytes(uint8_t *dest, size_t len);

// Fill buffer with random bytes with health check (assumes rng module exists)
bool random_bytes_checked(uint8_t *dest, size_t len);

// Check if RNG is healthy (assumes rng module exists)
bool rng_is_healthy(void);

#endif // CRYPTO_H

// crypto.c
#include "crypto.h"
#include "rng.h"  // Include the RNG module header

// Fill buffer with random bytes
void random_bytes(uint8_t *dest, size_t len) {
    fill_random(dest, len);
}

// Fill buffer with random bytes with health check
bool random_bytes_checked(uint8_t *dest, size_t len) {
    return fill_random_checked(dest, len);
}

// Check if RNG is healthy
bool rng_is_healthy(void) {
    return is_healthy();
}
