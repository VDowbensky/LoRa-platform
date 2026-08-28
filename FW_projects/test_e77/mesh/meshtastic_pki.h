#ifndef _MESHTASTIC_PKI_H_
#define _MESHTASTIC_PKI_H_

#include "meshtastic_pb.h"

// ─── Constants ─────────────────────────────────────────────────────────────────

#define MESH_PKI_OVERHEAD   12   // 8-byte CCM auth tag + 4-byte extraNonce
#define MESH_PKI_TAG_SIZE   8    // AES-CCM MAC length (M parameter)
#define MESH_PKI_KEY_SIZE   32   // x25519 key size
#define MESH_PKI_CCM_L      2    // CCM L parameter (fixed in firmware's aes-ccm.cpp)
#define MESH_PKI_NONCE_LEN  13   // 15 - L = 13 bytes used by CCM

// ─── Node Key Entry ────────────────────────────────────────────────────────────

typedef struct MeshNodeKey 
{
  uint32_t node_num;
  uint8_t  public_key[32];
  bool     has_key;
}MeshNodeKey_t;

// ─── Node Key Store ────────────────────────────────────────────────────────────

#ifndef MESH_MAX_NODE_KEYS
#define MESH_MAX_NODE_KEYS 64
#endif

typedef struct
{
  MeshNodeKey_t entries[MESH_MAX_NODE_KEYS];
  uint16_t count;
/*   void init();
  bool setKey(uint32_t node_num, const uint8_t pubkey[32]);
  const uint8_t* getKey(uint32_t node_num); */
}MeshNodeKeyStore_t;
void MeshNodeKeyStore_init(void);
bool MeshNodeKeyStore_setKey(uint32_t node_num,const uint8_t pubkey[32]);
const uint8_t* MeshNodeKeyStore_getKey(uint32_t node_num);

// ─── PKI Nonce Construction ────────────────────────────────────────────────────

/**
 * Build the 13-byte nonce for AES-CCM PKI encryption.
 * Matches CryptoEngine::initNonce(fromNode, packetId, extraNonce).
 *
 * Layout (16 bytes, but CCM only uses first 13):
 *   [0..3]   packetId    (uint32_t LE)
 *   [4..7]   extraNonce  (uint32_t LE — overwrites upper bits)
 *   [8..11]  fromNode    (uint32_t LE)
 *   [12..15] 0x00000000
 */
void meshBuildPkiNonce(uint8_t nonce[16],uint32_t fromNode,uint32_t packetId,uint32_t extraNonce);

// ─── Platform Abstraction ──────────────────────────────────────────────────────
// The PKI module requires three platform-specific crypto primitives:
//   1. x25519 Diffie-Hellman
//   2. SHA-256
//   3. AES-256-CCM
// Implement the extern "C" functions declared below.

/// x25519 DH: shared_out = DH(our_private, their_public). Return true on success.
extern bool mesh_x25519_dh(const uint8_t our_private[32],const uint8_t their_public[32],uint8_t shared_out[32]);
/// SHA-256: hash `data` of `len` bytes into `out` (32 bytes).
extern void mesh_sha256(const uint8_t *data, uint16_t len, uint8_t out[32]);
/// AES-256-CCM encrypt. nonce is 13 bytes. tag_len = 8.
extern bool mesh_ccm_encrypt(const uint8_t key[32],
                              const uint8_t nonce[13],
                              const uint8_t *plain, uint16_t plain_len,
                              uint8_t *cipher, uint8_t *tag, uint16_t tag_len);
/// AES-256-CCM decrypt with auth. Returns false if tag doesn't match.
extern bool mesh_ccm_decrypt(const uint8_t key[32],
                              const uint8_t nonce[13],
                              const uint8_t *cipher, uint16_t cipher_len,
                              const uint8_t *tag, uint16_t tag_len,
                              uint8_t *plain);

/// Generate x25519 keypair. Return true on success.
extern bool mesh_generate_keypair(uint8_t public_key[32], uint8_t private_key[32]);

//General functions
bool meshX25519DH(const uint8_t our_private[32],const uint8_t their_public[32],uint8_t shared_out[32]);
void meshSHA256(const uint8_t *data,uint16_t len,uint8_t out[32]);
void meshSHA256InPlace(uint8_t *data,uint16_t len);
bool meshCcmEncrypt(const uint8_t key[32],const uint8_t nonce[16],const uint8_t *plaintext,uint16_t plain_len,
                    uint8_t *ciphertext,uint8_t *auth_out,uint16_t tag_len);

bool meshCcmDecrypt(const uint8_t key[32],const uint8_t nonce[16],const uint8_t *ciphertext, uint16_t cipher_len,
                    const uint8_t *auth_tag,uint16_t tag_len,uint8_t *plaintext);
bool meshGenerateKeyPair(uint8_t public_key[32], uint8_t private_key[32]);

// ─── PKI Encrypt / Decrypt ─────────────────────────────────────────────────────

/**
 * Compute the DH shared secret for a node, then SHA-256 hash it.
 * This is the key derivation step from CryptoEngine.cpp:91-94.
 */
bool meshDeriveSharedKey(const uint8_t our_private[32],const uint8_t their_public[32],uint8_t derived_key[32]);

/**
 * PKI encrypt a payload for a specific recipient.
 * Matches CryptoEngine::encryptCurve25519() exactly.
 *
 * @param our_private    Our x25519 private key (32 bytes)
 * @param their_public   Recipient's x25519 public key (32 bytes)
 * @param from_node      Our node number
 * @param packet_id      Packet ID
 * @param plaintext      Data to encrypt
 * @param plain_len      Length of plaintext
 * @param out            Output buffer (must be >= plain_len + 12 bytes)
 * @param rand_fn        Random number generator (returns uint32_t)
 *
 * Output layout in `out`: [ciphertext (plain_len)][auth_tag (8)][extraNonce (4)]
 * Returns total output length (plain_len + 12), or 0 on failure.
 */
uint16_t meshPkiEncrypt(const uint8_t our_private[32],const uint8_t their_public[32],uint32_t from_node,
                        uint32_t packet_id,const uint8_t *plaintext, uint16_t plain_len,uint8_t *out,
                        uint32_t (*rand_fn)(void));

/**
 * PKI decrypt a received DM payload.
 * Matches CryptoEngine::decryptCurve25519() exactly.
 *
 * @param our_private    Our x25519 private key (32 bytes)
 * @param their_public   Sender's x25519 public key (32 bytes)
 * @param from_node      Sender's node number (from packet header)
 * @param packet_id      Packet ID (from packet header)
 * @param encrypted      Full encrypted payload including overhead
 * @param encrypted_len  Total length (ciphertext + 8 auth + 4 extraNonce)
 * @param plaintext_out  Output buffer (must be >= encrypted_len - 12)
 *
 * Returns plaintext length on success, or 0 on failure (bad key, auth fail, etc).
 */
uint16_t meshPkiDecrypt(const uint8_t our_private[32],const uint8_t their_public[32],uint32_t from_node,
                        uint32_t packet_id,const uint8_t *encrypted,uint16_t encrypted_len,uint8_t *plaintext_out);

// ─── PKI Identity ──────────────────────────────────────────────────────────────

typedef struct
{
  uint8_t  public_key[32];
  uint8_t  private_key[32];
  bool     initialized;
/*   void init();
  bool generate();
  void load(const uint8_t pub[32], const uint8_t priv[32]); */
}MeshPkiIdentity_t;

void MeshPkiIdentity_init(void);
bool MeshPkiIdentity_generate(void);
void MeshPkiIdentity_load(const uint8_t pub[32],const uint8_t priv[32]);

// ─── Integrated PKI + Channel RX Processing ────────────────────────────────────
//Check if a packet looks like it could be PKI-encrypted.
//Matches the condition at Router.cpp:453-455.
bool meshIsPkiCandidate(const MeshRxPacket_t *pkt,uint32_t our_node);

//Attempt PKI decryption of a packet.
//Returns plaintext length on success, 0 on failure.
//On success, `out_plaintext` contains the decrypted Data protobuf.
uint16_t meshTryPkiDecrypt(const MeshRxPacket_t *pkt,const MeshPkiIdentity_t *identity,const MeshNodeKeyStore_t *key_store,
                          uint8_t *out_plaintext);
// ─── Auto-learn Public Keys from NODEINFO ──────────────────────────────────────
//If a decoded Data message is a NODEINFO_APP, extract the public key
//and store it in the key store. Call this after successful channel-PSK
//decryption of broadcast packets.
//Returns true if a key was learned.
bool meshLearnNodeKey(const MeshRxPacket_t *pkt,const MeshData_t *data,MeshNodeKeyStore_t *key_store);
                                     
#endif
