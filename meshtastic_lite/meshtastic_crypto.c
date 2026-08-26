/**
 * meshtastic_crypto.h — AES-CTR encryption/decryption for Meshtastic channel PSK.
 *
 * Nonce layout from CryptoEngine.cpp:253-261:
 *   [0..7]   packetId    (uint64_t LE — upper 32 bits are always 0 for current firmware)
 *   [8..11]  fromNode    (uint32_t LE)
 *   [12..15] blockCounter (starts at 0, incremented per 16-byte AES block)
 *
 * AES-CTR works identically for encrypt and decrypt (XOR with keystream).
 *
 * Part of meshtastic-lite.
 */

#include "meshtastic_crypto.h"

// ─── Key Structures ────────────────────────────────────────────────────────────

MeshCryptoKey_t MeshCryptoKey;

// ─── Nonce Construction ────────────────────────────────────────────────────────

/**
 * Build the 16-byte nonce/IV for AES-CTR.
 * Matches CryptoEngine::initNonce() exactly.
 */
void meshBuildNonce(uint8_t nonce[16], uint32_t fromNode, uint32_t packetId) 
{
  memset(nonce, 0, 16);
  // packetId as uint64_t LE in bytes [0..7]
  uint64_t pid64 = (uint64_t)packetId;
  memcpy(nonce, &pid64, sizeof(uint64_t));
  // fromNode as uint32_t LE in bytes [8..11]
  memcpy(nonce + 8, &fromNode, sizeof(uint32_t));
  // bytes [12..15] = block counter, starts at 0 (handled by CTR mode)
}

// ─── Platform-specific AES-CTR ─────────────────────────────────────────────────

/**
 * Software AES-CTR encrypt/decrypt in-place.
 */
bool meshCryptCtr(const MeshCryptoKey *key, uint8_t nonce[16], uint8_t *data, size_t len)
{
  if (key->length <= 0 || len == 0) return true;
  int key_bits = key->length * 8;
  uint8_t counter_block[16];
  uint8_t keystream[16];
  
  memcpy(counter_block, nonce, 16);
  for (size_t offset = 0; offset < len; offset += 16) 
  {
    mesh_aes_block_encrypt(key->bytes, key_bits, counter_block, keystream);
    size_t block_len = (len - offset < 16) ? (len - offset) : 16;
    for (size_t i = 0; i < block_len; i++) data[offset + i] ^= keystream[i];
    // Increment the 32-bit counter in bytes [12..15] (big-endian)
    for (int i = 15; i >= 12; i--) 
    {
      if (++counter_block[i] != 0) break;
    }
  }
  return true;
}

// ─── Convenience Wrappers ──────────────────────────────────────────────────────

/**
 * Decrypt a Meshtastic payload in-place.
 * `data` points to the encrypted payload (after the 16-byte header).
 */
static inline bool meshDecrypt(const MeshCryptoKey *key, uint32_t fromNode, uint32_t packetId, uint8_t *data, size_t len)
{
  uint8_t nonce[16];
  meshBuildNonce(nonce, fromNode, packetId);
  return meshCryptCtr(key, nonce, data, len);
}

/**
 * Encrypt a Meshtastic payload in-place. (CTR is symmetric.)
 */
static inline bool meshEncrypt(const MeshCryptoKey *key, uint32_t fromNode, uint32_t packetId, uint8_t *data, size_t len)
{
  return meshDecrypt(key, fromNode, packetId, data, len);
}
