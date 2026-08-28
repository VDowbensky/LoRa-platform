/**
 * meshtastic_pki.h — Public Key Infrastructure for Meshtastic Direct Messages.
 *
 * Implements x25519 DH key exchange + AES-256-CCM authenticated encryption
 * for unicast DMs, as introduced in Meshtastic v2.5.
 *
 * Extracted from:
 *   - CryptoEngine.cpp:78-142  (encryptCurve25519 / decryptCurve25519)
 *   - CryptoEngine.cpp:144-198 (DH, SHA256, key management)
 *   - CryptoEngine.cpp:253-262 (initNonce with extraNonce)
 *   - aes-ccm.cpp              (AES-CCM with L=2, variable M)
 *   - Router.cpp:451-481       (PKI decrypt path, priority over channel PSK)
 *   - RadioInterface.h:22      (MESHTASTIC_PKC_OVERHEAD = 12)
 *
 * Wire format for PKI-encrypted payload:
 *   [ciphertext (N bytes)][auth_tag (8 bytes)][extraNonce (4 bytes)]
 *   Total overhead: 12 bytes
 *
 * Nonce construction with extraNonce (CryptoEngine.cpp:253-262):
 *   [0..3]   packetId    (uint32_t LE)
 *   [4..7]   extraNonce  (uint32_t LE — overwrites upper 32 bits of packetId)
 *   [8..11]  fromNode    (uint32_t LE)
 *   [12..15] 0x00000000  (block counter / unused — CCM uses only 13 bytes)
 *
 * Part of meshtastic-lite.
 */
#include "meshtastic_pki.h"

MeshNodeKeyStore_t  MeshNodeKeyStore;
MeshNodeKey_t entries[MESH_MAX_NODE_KEYS];
uint16_t count;

void MeshNodeKeyStore_init(void)
{
  memset(entries, 0, sizeof(entries));
  count = 0;
}

//Store or update a node's public key.
//Called when we receive a NODEINFO_APP packet containing a public key.
bool MeshNodeKeyStore_setKey(uint32_t node_num,const uint8_t pubkey[32])
{
  // Update existing
  for (uint16_t i = 0; i < count; i++) 
  {
    if (entries[i].node_num == node_num) 
    {
      memcpy(entries[i].public_key, pubkey, 32);
      entries[i].has_key = true;
      return true;
    }
  }
  // Add new
  if (count < MESH_MAX_NODE_KEYS) 
  {
    entries[count].node_num = node_num;
    memcpy(entries[count].public_key, pubkey, 32);
    entries[count].has_key = true;
    count++;
    return true;
  }
  // Evict oldest (simple FIFO for now)
  memmove(&entries[0], &entries[1], sizeof(MeshNodeKey) * (MESH_MAX_NODE_KEYS - 1));
  count = MESH_MAX_NODE_KEYS - 1;
  entries[count].node_num = node_num;
  memcpy(entries[count].public_key, pubkey, 32);
  entries[count].has_key = true;
  count++;
  return true;
}

//Look up a node's public key. Returns nullptr if not found.
const uint8_t* MeshNodeKeyStore_getKey(uint32_t node_num)
{
  for (uint16_t i = 0; i < count; i++) 
  {
    if (entries[i].node_num == node_num && entries[i].has_key) return entries[i].public_key;
  }
  return NULL;
}


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
void meshBuildPkiNonce(uint8_t nonce[16],uint32_t fromNode,uint32_t packetId,uint32_t extraNonce)
{
  memset(nonce, 0, 16);
  uint64_t pid64 = (uint64_t)packetId;
  memcpy(nonce, &pid64, sizeof(uint64_t));          // [0..7]
  memcpy(nonce + 8, &fromNode, sizeof(uint32_t));   // [8..11]
  memcpy(nonce + 4, &extraNonce, sizeof(uint32_t)); // [4..7] overwrites upper 32 of packetId
}

//General functions
bool meshX25519DH(const uint8_t our_private[32],const uint8_t their_public[32],uint8_t shared_out[32])
{
  return mesh_x25519_dh(our_private, their_public, shared_out);
}

void meshSHA256(const uint8_t *data,uint16_t len,uint8_t out[32])
{
  mesh_sha256(data, len, out);
}

void meshSHA256InPlace(uint8_t *data,uint16_t len)
{
  uint8_t hash[32];
  mesh_sha256(data, len, hash);
  memcpy(data, hash, 32);
}

bool meshCcmEncrypt(const uint8_t key[32],const uint8_t nonce[16],const uint8_t *plaintext,uint16_t plain_len,
                    uint8_t *ciphertext,uint8_t *auth_out,uint16_t tag_len)
{
  return mesh_ccm_encrypt(key, nonce, plaintext, plain_len, ciphertext, auth_out, tag_len);
}

bool meshCcmDecrypt(const uint8_t key[32],const uint8_t nonce[16],const uint8_t *ciphertext, uint16_t cipher_len,
                    const uint8_t *auth_tag,uint16_t tag_len,uint8_t *plaintext)
{
  return mesh_ccm_decrypt(key, nonce, ciphertext, cipher_len, auth_tag, tag_len, plaintext);
}

bool meshGenerateKeyPair(uint8_t public_key[32], uint8_t private_key[32])
{
  return mesh_generate_keypair(public_key, private_key);
}


// ─── PKI Encrypt / Decrypt ─────────────────────────────────────────────────────

//Compute the DH shared secret for a node, then SHA-256 hash it.
//This is the key derivation step from CryptoEngine.cpp:91-94.
bool meshDeriveSharedKey(const uint8_t our_private[32],const uint8_t their_public[32],uint8_t derived_key[32])
{
  if (!meshX25519DH(our_private, their_public, derived_key)) return false;
  meshSHA256InPlace(derived_key, 32);
  return true;
}

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
                        uint32_t (*rand_fn)(void))
{
  // Derive shared key
  uint8_t shared_key[32];
  if (!meshDeriveSharedKey(our_private, their_public, shared_key)) return 0;
  // Generate random extraNonce
  uint32_t extraNonce = rand_fn();
  // Build nonce
  uint8_t nonce[16];
  meshBuildPkiNonce(nonce, from_node, packet_id, extraNonce);
  // Encrypt with AES-256-CCM
  uint8_t *ciphertext = out;
  uint8_t *auth_tag = out + plain_len;
  if (!meshCcmEncrypt(shared_key, nonce, plaintext, plain_len,ciphertext, auth_tag, MESH_PKI_TAG_SIZE)) return 0;
  // Append extraNonce after auth tag
  memcpy(out + plain_len + MESH_PKI_TAG_SIZE, &extraNonce, sizeof(uint32_t));
  return plain_len + MESH_PKI_OVERHEAD;
}

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
                        uint32_t packet_id,const uint8_t *encrypted,uint16_t encrypted_len,uint8_t *plaintext_out)
{
  if (encrypted_len <= MESH_PKI_OVERHEAD) return 0;
  uint16_t cipher_len = encrypted_len - MESH_PKI_OVERHEAD;
  const uint8_t *auth_tag = encrypted + cipher_len;
  // Extract extraNonce from last 4 bytes
  uint32_t extraNonce;
  memcpy(&extraNonce, auth_tag + MESH_PKI_TAG_SIZE, sizeof(uint32_t));
  // Derive shared key
  uint8_t shared_key[32];
  if (!meshDeriveSharedKey(our_private, their_public, shared_key)) return 0;
  // Build nonce
  uint8_t nonce[16];
  meshBuildPkiNonce(nonce, from_node, packet_id, extraNonce);
  // Decrypt with AES-256-CCM (returns false if auth fails)
  if (!meshCcmDecrypt(shared_key, nonce, encrypted, cipher_len,auth_tag, MESH_PKI_TAG_SIZE, plaintext_out)) return 0;
  return cipher_len;
}

// ─── PKI Identity ──────────────────────────────────────────────────────────────

MeshPkiIdentity_t MeshPkiIdentity;

void MeshPkiIdentity_init(void)
{
  memset(MeshPkiIdentity.public_key,0,sizeof(public_key));
  memset(MeshPkiIdentity.private_key,0,sizeof(private_key));
  MeshPkiIdentity.initialized = false;
}

///Generate a new keypair. Typically done once on first boot, then persisted to NVS/flash.
bool MeshPkiIdentity_generate(void)
{
  if (meshGenerateKeyPair(public_key, private_key)) 
  {
    MeshPkiIdentity.initialized = true;
    return true;
  }
  return false;
}

//Load an existing keypair (from NVS/flash).
void MeshPkiIdentity_load(const uint8_t pub[32],const uint8_t priv[32])
{
  memcpy(MeshPkiIdentity.public_key, pub, 32);
  memcpy(MeshPkiIdentity.private_key, priv, 32);
  MeshPkiIdentity.initialized = true;
}


// ─── Integrated PKI + Channel RX Processing ────────────────────────────────────
//Check if a packet looks like it could be PKI-encrypted.
/Matches the condition at Router.cpp:453-455.
bool meshIsPkiCandidate(const MeshRxPacket_t *pkt,uint32_t our_node)
{
  return pkt->channel_hash == 0 && pkt->to == our_node && pkt->to != MESH_ADDR_BROADCAST && pkt->payload_len > MESH_PKI_OVERHEAD;
}

//Attempt PKI decryption of a packet.
//Returns plaintext length on success, 0 on failure.
//On success, `out_plaintext` contains the decrypted Data protobuf.
uint16_t meshTryPkiDecrypt(const MeshRxPacket_t *pkt,const MeshPkiIdentity_t *identity,const MeshNodeKeyStore_t *key_store,
                          uint8_t *out_plaintext)
{
  if (!identity->initialized) return 0;
  const uint8_t *sender_key = key_store->getKey(pkt->from);
  if (!sender_key) return 0;
  return meshPkiDecrypt(identity->private_key,sender_key,pkt->from,pkt->id,pkt->payload,pkt->payload_len,out_plaintext);
}

// ─── Auto-learn Public Keys from NODEINFO ──────────────────────────────────────
//If a decoded Data message is a NODEINFO_APP, extract the public key
//and store it in the key store. Call this after successful channel-PSK
//decryption of broadcast packets.
//Returns true if a key was learned.
bool meshLearnNodeKey(const MeshRxPacket *pkt,const MeshData_t *data,MeshNodeKeyStore_t *key_store)
{
  if (data->portnum != PORT_NODEINFO) return false;
  MeshUser_t user;
  if (!meshDecodeUser(data->payload, data->payload_len, &user)) return false;
  if (user.public_key_len == 32) 
  {
    key_store->setKey(pkt->from, user.public_key);
    return true;
  }
  return false;
}
