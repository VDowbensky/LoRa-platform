#ifndef _MESHTASTIC_CRYPTO_H_
#define _MESHTASTIC_CRYPTO_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

// ─── Key Structures ────────────────────────────────────────────────────────────

typedef struct MeshCryptoKey 
{
    uint8_t bytes[32];
    int8_t  length;     // 0 = no encryption, 16 = AES-128, 32 = AES-256, -1 = invalid
}MeshCryptoKey_t;

// ─── Nonce Construction ────────────────────────────────────────────────────────

/**
 * Build the 16-byte nonce/IV for AES-CTR.
 * Matches CryptoEngine::initNonce() exactly.
 */
void meshBuildNonce(uint8_t nonce[16],uint32_t fromNode,uint32_t packetId);

// ─── Platform-specific AES-CTR ─────────────────────────────────────────────────
/**
 * External AES block encrypt function.
 * Must encrypt a single 16-byte block: out = AES_encrypt(key, in).
 * key_bits is 128 or 256.
 */
extern void mesh_aes_block_encrypt(const uint8_t *key,int key_bits,const uint8_t in[16],uint8_t out[16]);
/**
 * Software AES-CTR encrypt/decrypt in-place.
 */
bool meshCryptCtr(const MeshCryptoKey_t *key,uint8_t nonce[16],uint8_t *data,uint16_t len);

// ─── Convenience Wrappers ──────────────────────────────────────────────────────
/**
 * Decrypt a Meshtastic payload in-place.
 * `data` points to the encrypted payload (after the 16-byte header).
 */
bool meshDecrypt(const MeshCryptoKey_t *key,uint32_t fromNode,uint32_t packetId,uint8_t *data,uint16_t len);
/**
 * Encrypt a Meshtastic payload in-place. (CTR is symmetric.)
 */
bool meshEncrypt(const MeshCryptoKey_t *key,uint32_t fromNode,uint32_t packetId,uint8_t *data,uint16_t len);

#endif


