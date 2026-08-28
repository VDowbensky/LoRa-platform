#ifndef __MESHTASTIC_CHANNEL_H__
#define __MESHTASTIC_CHANNEL_H__

#include "meshtastic_defs.h"

// ─── Constants ─────────────────────────────────────────────────────────────────

#define MESH_MAX_CHANNELS 8

/**
 * The well-known 16-byte AES-128 default PSK (Channels.h:144).
 * This is the expanded form of the single-byte PSK index 1 ("AQ==").
 * All nodes on the default "LongFast" channel share this key.
 */
extern const uint8_t MESH_DEFAULT_PSK[];

// ─── Channel Configuration ─────────────────────────────────────────────────────

typedef struct
{
  char         name[32];       // Empty string "" means use preset name
  uint8_t      psk_raw[34];    // Raw PSK as configured (may be 0, 1, 16, or 32 bytes)
  uint8_t      psk_raw_len;
  bool         enabled;
  bool         is_primary;

  // Computed at init:
  MeshCryptoKey_t key;           // Expanded AES key
  uint8_t       hash;          // Channel hash for wire matching
}MeshChannel_t;

// ─── XOR Hash (Channels.cpp:27-33) ────────────────────────────────────────────

uint8_t meshXorHash(const uint8_t *p,uint16_t len);

// ─── PSK Expansion (Channels.cpp:208-256) ──────────────────────────────────────

/**
 * Expand a raw PSK into a full AES key, following Meshtastic's rules:
 *   - 0 bytes  → no encryption (key.length = 0)
 *   - 1 byte   → "short PSK index": 0 = no encrypt, 1+ = defaultpsk with last byte bumped
 *   - 2-15     → pad to 16 bytes with zeros (AES-128)
 *   - 16 bytes → AES-128 as-is
 *   - 17-31    → pad to 32 bytes with zeros (AES-256)
 *   - 32 bytes → AES-256 as-is
 */
MeshCryptoKey_t meshExpandPsk(const uint8_t *raw,uint8_t raw_len);

// ─── Channel Hash Computation (Channels.cpp:39-52) ─────────────────────────────

/**
 * Compute the single-byte channel hash.
 * hash = XOR(name_bytes) ^ XOR(expanded_key_bytes)
 *
 * `name` is the effective channel name: if the channel name is empty "",
 * use the modem preset name (e.g., "LongFast").
 */
uint8_t meshComputeChannelHash(const char *effective_name,const MeshCryptoKey_t *key);
// ─── Channel Table ─────────────────────────────────────────────────────────────

typedef struct
{
    MeshChannel_t channels[MESH_MAX_CHANNELS];
    uint8_t count;
    MeshModemPreset_t preset;
/*  void init(MeshModemPreset p);
    const char* effectiveName(uint8_t idx);
    int addChannel(const char *name, const uint8_t *psk, uint8_t psk_len, bool primary);
    int addDefaultChannel();
    int findByHash(uint8_t wire_hash, MeshCryptoKey *out_key);
    int tryDecrypt(const uint8_t *payload, uint16_t payload_len,
                   uint8_t wire_hash,
                   uint32_t from_node, uint32_t packet_id,
                   uint8_t *out_plaintext,
                   MeshCryptoKey *out_key,
                   bool (*validate_fn)(const uint8_t *, uint16_t) = NULL);  */
}MeshChannelTable_t;

void MeshChannelTable_init(MeshModemPreset_t p);
const char* MeshChannelTable_effectiveName(uint8_t idx);
int MeshChannelTable_addChannel(const char *name,const uint8_t *psk,uint8_t psk_len,bool primary);
int MeshChannelTable_addDefaultChannel(void);
int MeshChannelTable_findByHash(uint8_t wire_hash,MeshCryptoKey_t *out_key);
int MeshChannelTable_tryDecrypt(const uint8_t *payload,uint16_t payload_len,uint8_t wire_hash,
                                uint32_t from_node, uint32_t packet_id,uint8_t *out_plaintext,
                                MeshCryptoKey_t *out_key,bool (*validate_fn)(const uint8_t *, uint16_t)); 

#endif

