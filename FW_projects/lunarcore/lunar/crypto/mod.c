#include "mod.h"
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
static inline bool constant_time_eq(const uint8_t *a, size_t a_len, const uint8_t *b, size_t b_len) 
{
  // Check if lengths are equal
  if (a_len != b_len) return false;
  // Perform constant-time comparison
  uint8_t result = 0;
  for (size_t i = 0; i < a_len; i++) result |= a[i] ^ b[i];
  return result == 0;
}

// Securely zero memory (prevents compiler optimization)
static inline void secure_zero(uint8_t *data, size_t len) 
{
  // Use volatile pointer to prevent compiler optimization
  volatile uint8_t *volatile_ptr = (volatile uint8_t *)data;
  for (size_t i = 0; i < len; i++) volatile_ptr[i] = 0;
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
static inline void secure_zero_u32(uint32_t *data, size_t len) 
{
  // Use volatile pointer to prevent compiler optimization
  volatile uint32_t *volatile_ptr = (volatile uint32_t *)data;
    for (size_t i = 0; i < len; i++) volatile_ptr[i] = 0;
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
void random_bytes(uint8_t *dest, size_t len) 
{
  fill_random(dest, len);
}

// Fill buffer with random bytes with health check
bool random_bytes_checked(uint8_t *dest, size_t len) 
{
  return fill_random_checked(dest, len);
}

// Check if RNG is healthy
bool rng_is_healthy(void) 
{
  return is_healthy();
}
