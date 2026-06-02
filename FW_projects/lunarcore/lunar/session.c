#include"session.h"

// Forward declarations for crypto functions (assumed to be provided)
// These would typically be in separate header files
void x25519(uint8_t out[32], const uint8_t scalar[32], const uint8_t point[32]);
void x25519_base(uint8_t out[32], const uint8_t scalar[32]);
void hkdf_derive(const uint8_t* input, size_t input_len, const uint8_t* salt, size_t salt_len, 
                 const uint8_t* info, size_t info_len, uint8_t* output, size_t output_len);
void sha256_hash(const uint8_t* data, size_t len, uint8_t hash[32]);

// AES256 structure and functions
typedef struct 
{
  uint8_t key[32];
  // Internal state would go here
} Aes256;

void aes256_new(Aes256* aes, const uint8_t key[32]);
void aes256_encrypt_block(const Aes256* aes, uint8_t block[16]);

// Constants
#define MAX_MESSAGES_BEFORE_RATCHET 100
#define MAX_TIME_BEFORE_RATCHET_SECS 600
#define MAX_SKIPPED_KEYS 100
#define SESSION_SERIALIZED_SIZE 225
#define MAX_PERSISTED_SESSIONS 32
#define HEAPLESS_VEC_CAPACITY 256
#define HASHER_DATA_CAPACITY 512

static const uint8_t SESSION_HINT_INFO[] = "session-hint-v1";
static const uint8_t ROOT_KEY_INFO[] = "lunarpunk-root-key-v2";
static const uint8_t CHAIN_KEY_INFO[] = "lunarpunk-chain-key-v2";
static const uint8_t MESSAGE_KEY_INFO[] = "lunarpunk-message-key-v2";
static const uint8_t NVS_SESSION_NAMESPACE[] = "sessions";

// HeaplessVec implementation
typedef struct 
{
  uint8_t data[HEAPLESS_VEC_CAPACITY];
  size_t len;
} HeaplessVec;

static void heapless_vec_init(HeaplessVec* vec) 
{
  vec->len = 0;
}

static bool heapless_vec_extend_from_slice(HeaplessVec* vec, const uint8_t* data, size_t len) 
{
  if (vec->len + len > HEAPLESS_VEC_CAPACITY) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

// HashMap entry for skipped keys
typedef struct 
{
  uint8_t key_prefix[8];
  uint64_t message_num;
  uint8_t message_key[32];
  bool occupied;
} SkippedKeyEntry;

// HashMap for skipped keys
typedef struct 
{
  SkippedKeyEntry entries[MAX_SKIPPED_KEYS];
  size_t count;
} SkippedKeysMap;

static void skipped_keys_map_init(SkippedKeysMap* map) 
{
  memset(map->entries, 0, sizeof(map->entries));
  map->count = 0;
}

static bool skipped_keys_map_insert(SkippedKeysMap* map, const uint8_t key_prefix[8],uint64_t message_num, const uint8_t message_key[32]) 
{
  // Find empty slot or existing key
  for (size_t i = 0; i < MAX_SKIPPED_KEYS; i++) 
  {
    if (!map->entries[i].occupied) 
    {
      memcpy(map->entries[i].key_prefix, key_prefix, 8);
      map->entries[i].message_num = message_num;
      memcpy(map->entries[i].message_key, message_key, 32);
      map->entries[i].occupied = true;
      map->count++;
      return true;
    }
  }
  return false;
}

static bool skipped_keys_map_remove(SkippedKeysMap* map, const uint8_t key_prefix[8],uint64_t message_num, uint8_t message_key[32]) 
{
  for (size_t i = 0; i < MAX_SKIPPED_KEYS; i++) 
  {
    if (map->entries[i].occupied && memcmp(map->entries[i].key_prefix, key_prefix, 8) == 0 && map->entries[i].message_num == message_num) 
    {
      memcpy(message_key, map->entries[i].message_key, 32);
      map->entries[i].occupied = false;
      map->count--;
      return true;
    }
  }
  return false;
}

// Random number generation
#ifdef __XTENSA__
static void fill_random(uint8_t* dest, size_t len) 
{
  const uint32_t RNG_DATA_REG = 0x3FF75144;
  for (size_t i = 0; i < len; i += 4) 
  {
    size_t chunk_size = (len - i) < 4 ? (len - i) : 4;
    uint32_t random_word = *(volatile uint32_t*)RNG_DATA_REG;
    uint8_t bytes[4];
    bytes[0] = (uint8_t)(random_word & 0xFF);
    bytes[1] = (uint8_t)((random_word >> 8) & 0xFF);
    bytes[2] = (uint8_t)((random_word >> 16) & 0xFF);
    bytes[3] = (uint8_t)((random_word >> 24) & 0xFF);
    for (size_t j = 0; j < chunk_size; j++) dest[i + j] = bytes[j];
  }
}
#else
static uint64_t COUNTER = 0;

static void fill_random(uint8_t* dest, size_t len) 
{
  uint8_t seed[40] = {0};
  // Copy counter
  memcpy(seed, &COUNTER, 8);
  COUNTER = COUNTER + 1;
  // Get stack address
  uintptr_t stack_addr = (uintptr_t)&seed;
  memcpy(seed + 8, &stack_addr, 8);
  // Hash the seed
  uint8_t hash[32];
  sha256_hash(seed, 40, hash);
  size_t copy_len = len < 32 ? len : 32;
  memcpy(dest, hash, copy_len);
  // Recursively fill if needed
  if (len > 32) fill_random(dest + 32, len - 32);
}
#endif

// Constant time comparison
__attribute__((noinline))
static bool constant_time_eq(const uint8_t* a, size_t a_len, const uint8_t* b, size_t b_len) 
{
  if (a_len != b_len) return false;
  volatile uint8_t result = 0;
  for (size_t i = 0; i < a_len; i++) result |= a[i] ^ b[i];
  return result == 0;
}

// Session error codes
typedef enum 
{
  SESSION_ERROR_NONE = 0,
  SESSION_ERROR_NOT_ESTABLISHED,
  SESSION_ERROR_INVALID_FORMAT,
  SESSION_ERROR_DECRYPTION_FAILED,
  SESSION_ERROR_OLD_CHAIN,
  SESSION_ERROR_TOO_MANY_SKIPPED,
  SESSION_ERROR_KEY_DERIVATION_FAILED
} SessionError;

// Message header structure
typedef struct 
{
  uint8_t dh_public[32];
  uint64_t prev_chain_len;
  uint64_t message_num;
} MessageHeader;

// Encode message header to bytes
static void message_header_encode(const MessageHeader* header, uint8_t buf[48]) 
{
  memcpy(buf, header->dh_public, 32);
  memcpy(buf + 32, &header->prev_chain_len, 8);
  memcpy(buf + 40, &header->message_num, 8);
}

// Decode message header from bytes
static bool message_header_decode(const uint8_t* data, size_t len, MessageHeader* header) 
{
  if (len < 48) return false;
  memcpy(header->dh_public, data, 32);
  memcpy(&header->prev_chain_len, data + 32, 8);
  memcpy(&header->message_num, data + 40, 8);
  return true;
}

// Session structure
typedef struct 
{
  uint8_t root_key[32];
  uint8_t send_chain_key[32];
  uint8_t recv_chain_key[32];
  uint8_t send_ratchet_private[32];
  uint8_t send_ratchet_public[32];
  uint8_t recv_ratchet_public[32];
  uint64_t send_count;
  uint64_t recv_count;
  uint64_t prev_recv_chain;
  uint64_t last_ratchet_time;
  SkippedKeysMap skipped_keys;
  bool established;
} Session;

// Session parameters for initialization
typedef struct 
{
  uint8_t shared_secret[32];
  uint8_t our_private[32];
  uint8_t their_public[32];
  bool is_initiator;
} SessionParams;

// Compute HMAC tag for message authentication
static void compute_tag(const uint8_t key[32], const uint8_t* header, size_t header_len,const uint8_t* ciphertext, size_t ct_len, uint8_t tag[16]) 
{
  uint8_t inner[64];
  uint8_t outer[64];
  // Initialize HMAC pads
  memset(inner, 0x36, 64);
  memset(outer, 0x5c, 64);
  // XOR key into pads
  for (size_t i = 0; i < 32; i++) 
  {
    inner[i] ^= key[i];
    outer[i] ^= key[i];
  }
  // Compute inner hash
  uint8_t hasher_data[HASHER_DATA_CAPACITY];
  size_t hasher_len = 0;
  memcpy(hasher_data + hasher_len, inner, 64);
  hasher_len += 64;
  memcpy(hasher_data + hasher_len, header, header_len);
  hasher_len += header_len;
  memcpy(hasher_data + hasher_len, ciphertext, ct_len);
  hasher_len += ct_len;
  uint8_t inner_hash[32];
  sha256_hash(hasher_data, hasher_len, inner_hash);
  // Compute outer hash
  uint8_t outer_data[96];
  memcpy(outer_data, outer, 64);
  memcpy(outer_data + 64, inner_hash, 32);
  uint8_t full_tag[32];
  sha256_hash(outer_data, 96, full_tag);
  // Return first 16 bytes
  memcpy(tag, full_tag, 16);
}

// Check if session should ratchet
static bool should_ratchet(const Session* session) 
{
  return session->send_count >= MAX_MESSAGES_BEFORE_RATCHET;
}

// Advance send ratchet
static void advance_send_ratchet(Session* session) 
{
  // Generate new private key
  uint8_t new_private[32];
  fill_random(new_private, 32);
  // Clamp the key
  new_private[0] &= 248;
  new_private[31] &= 127;
  new_private[31] |= 64;
  // Compute new public key
  uint8_t new_public[32];
  x25519_base(new_public, new_private);
  // Perform DH
  uint8_t dh_output[32];
  x25519(dh_output, new_private, session->recv_ratchet_public);
  // Derive new keys
  uint8_t kdf_input[64];
  memcpy(kdf_input, session->root_key, 32);
  memcpy(kdf_input + 32, dh_output, 32);
  hkdf_derive(kdf_input, 64, (const uint8_t*)"root", 4, ROOT_KEY_INFO,strlen((const char*)ROOT_KEY_INFO), session->root_key, 32);
  hkdf_derive(kdf_input, 64, (const uint8_t*)"chain", 5, CHAIN_KEY_INFO,strlen((const char*)CHAIN_KEY_INFO), session->send_chain_key, 32);
  // Update session state
  memcpy(session->send_ratchet_private, new_private, 32);
  memcpy(session->send_ratchet_public, new_public, 32);
  session->prev_recv_chain = session->recv_count;
  session->send_count = 0;
}

// Advance receive ratchet
static SessionError advance_recv_ratchet(Session* session, const uint8_t their_new_public[32]) 
{
  // Perform DH
  uint8_t dh_output[32];
  x25519(dh_output, session->send_ratchet_private, their_new_public);
  // Derive new keys
  uint8_t kdf_input[64];
  memcpy(kdf_input, session->root_key, 32);
  memcpy(kdf_input + 32, dh_output, 32);
  hkdf_derive(kdf_input, 64, (const uint8_t*)"root", 4, ROOT_KEY_INFO,strlen((const char*)ROOT_KEY_INFO), session->root_key, 32);
  hkdf_derive(kdf_input, 64, (const uint8_t*)"chain", 5, CHAIN_KEY_INFO,strlen((const char*)CHAIN_KEY_INFO), session->recv_chain_key, 32);
  // Update session state
  memcpy(session->recv_ratchet_public, their_new_public, 32);
  session->recv_count = 0;
  return SESSION_ERROR_NONE;
}

// Skip message keys
static SessionError skip_message_keys(Session* session, uint64_t until) 
{
  uint64_t to_skip = (until > session->recv_count) ? (until - session->recv_count) : 0;
  if (to_skip > MAX_SKIPPED_KEYS) return SESSION_ERROR_TOO_MANY_SKIPPED;
  while (session->recv_count < until) 
  {
    // Derive message key
    uint8_t message_key[32];
    uint8_t count_bytes[8];
    memcpy(count_bytes, &session->recv_count, 8);
    hkdf_derive(session->recv_chain_key, 32, count_bytes, 8, MESSAGE_KEY_INFO,strlen((const char*)MESSAGE_KEY_INFO), message_key, 32);
    // Store skipped key
    uint8_t key_prefix[8];
    memcpy(key_prefix, session->recv_ratchet_public, 8);
    skipped_keys_map_insert(&session->skipped_keys, key_prefix, session->recv_count, message_key);
    // Advance chain key
    uint8_t new_chain[32];
    hkdf_derive(session->recv_chain_key, 32, (const uint8_t*)"chain-advance", 13,CHAIN_KEY_INFO, strlen((const char*)CHAIN_KEY_INFO), new_chain, 32);
    memcpy(session->recv_chain_key, new_chain, 32);
    session->recv_count += 1;
  }
  return SESSION_ERROR_NONE;
}

// Get message key for decryption
static SessionError get_message_key(Session* session, const MessageHeader* header, uint8_t message_key[32]) 
{
  // Check skipped keys first
  uint8_t key_prefix[8];
  memcpy(key_prefix, header->dh_public, 8);
  if (skipped_keys_map_remove(&session->skipped_keys, key_prefix, header->message_num, message_key)) return SESSION_ERROR_NONE;
  // Skip keys if needed
  if (header->message_num > session->recv_count) 
  {
    SessionError err = skip_message_keys(session, header->message_num);
        if (err != SESSION_ERROR_NONE) return err;
  }
  // Derive message key
  uint8_t count_bytes[8];
  memcpy(count_bytes, &header->message_num, 8);
  hkdf_derive(session->recv_chain_key, 32, count_bytes, 8, MESSAGE_KEY_INFO,strlen((const char*)MESSAGE_KEY_INFO), message_key, 32);
  // Advance chain key
  uint8_t new_chain[32];
  hkdf_derive(session->recv_chain_key, 32, (const uint8_t*)"chain-advance", 13,CHAIN_KEY_INFO, strlen((const char*)CHAIN_KEY_INFO), new_chain, 32);
  memcpy(session->recv_chain_key, new_chain, 32);
  session->recv_count = header->message_num + 1;
  return SESSION_ERROR_NONE;
}

// Initialize a new session
static void session_new(Session* session, const SessionParams* params) 
{
  memset(session, 0, sizeof(Session));
  // Initialize skipped keys map
  skipped_keys_map_init(&session->skipped_keys);
  // Derive initial keys
  const uint8_t* salt = params->is_initiator ? (const uint8_t*)"initiator-salt-v1" : (const uint8_t*)"responder-salt-v1";
  size_t salt_len = strlen((const char*)salt);
  hkdf_derive(params->shared_secret, 32, salt, salt_len, ROOT_KEY_INFO,strlen((const char*)ROOT_KEY_INFO), session->root_key, 32);
  // Perform initial DH
  uint8_t dh_output[32];
  x25519(dh_output, params->our_private, params->their_public);
  // Derive chain keys
  uint8_t kdf_input[64];
  memcpy(kdf_input, session->root_key, 32);
  memcpy(kdf_input + 32, dh_output, 32);
  if (params->is_initiator) 
  {
    hkdf_derive(kdf_input, 64, (const uint8_t*)"send", 4, CHAIN_KEY_INFO,strlen((const char*)CHAIN_KEY_INFO), session->send_chain_key, 32);
    hkdf_derive(kdf_input, 64, (const uint8_t*)"recv", 4, CHAIN_KEY_INFO,strlen((const char*)CHAIN_KEY_INFO), session->recv_chain_key, 32);
  } 
  else 
  {
    hkdf_derive(kdf_input, 64, (const uint8_t*)"recv", 4, CHAIN_KEY_INFO,strlen((const char*)CHAIN_KEY_INFO), session->send_chain_key, 32);
    hkdf_derive(kdf_input, 64, (const uint8_t*)"send", 4, CHAIN_KEY_INFO,strlen((const char*)CHAIN_KEY_INFO), session->recv_chain_key, 32);
  }
  // Initialize ratchet keys
  memcpy(session->send_ratchet_private, params->our_private, 32);
  x25519_base(session->send_ratchet_public, params->our_private);
  memcpy(session->recv_ratchet_public, params->their_public, 32);
  // Initialize counters
  session->send_count = 0;
  session->recv_count = 0;
  session->prev_recv_chain = 0;
  session->last_ratchet_time = 0;
  session->established = true;
}

// Create uninitialized session
static void session_uninitialized(Session* session) 
{
  memset(session, 0, sizeof(Session));
  skipped_keys_map_init(&session->skipped_keys);
  session->established = false;
}

// Check if session is established
static bool session_is_established(const Session* session) 
{
  return session->established;
}

// Encrypt a message
static SessionError session_encrypt(Session* session, const uint8_t* plaintext, size_t pt_len,MessageHeader* header, HeaplessVec* ciphertext) 
{
  if (!session->established) return SESSION_ERROR_NOT_ESTABLISHED;
  // Derive message key
  uint8_t message_key[32];
  uint8_t count_bytes[8];
  memcpy(count_bytes, &session->send_count, 8);
  hkdf_derive(session->send_chain_key, 32, count_bytes, 8, MESSAGE_KEY_INFO,strlen((const char*)MESSAGE_KEY_INFO), message_key, 32);
  // Advance chain key
  uint8_t new_chain_key[32];
  hkdf_derive(session->send_chain_key, 32, (const uint8_t*)"chain-advance", 13,CHAIN_KEY_INFO, strlen((const char*)CHAIN_KEY_INFO), new_chain_key, 32);
  memcpy(session->send_chain_key, new_chain_key, 32);
  // Create header
  memcpy(header->dh_public, session->send_ratchet_public, 32);
  header->prev_chain_len = session->prev_recv_chain;
  header->message_num = session->send_count;
  // Prepare nonce for CTR mode
  uint8_t nonce[16] = {0};
  memcpy(nonce, &session->send_count, 8);
  // Initialize ciphertext with plaintext
  heapless_vec_init(ciphertext);
  if (!heapless_vec_extend_from_slice(ciphertext, plaintext, pt_len)) return SESSION_ERROR_INVALID_FORMAT;
  // Encrypt using AES-256-CTR
  Aes256 aes;
  aes256_new(&aes, message_key);
  uint8_t keystream[16];
  uint64_t block_counter = 0;
  for (size_t i = 0; i < ciphertext->len; i += 16) 
  {
    size_t chunk_size = (ciphertext->len - i < 16) ? (ciphertext->len - i) : 16;
    // Generate keystream block
    memcpy(keystream, nonce, 8);
    memcpy(keystream + 8, &block_counter, 8);
    aes256_encrypt_block(&aes, keystream);
    // XOR with plaintext
    for (size_t j = 0; j < chunk_size; j++) ciphertext->data[i + j] ^= keystream[j];
    block_counter += 1;
  }
  // Compute and append tag
  uint8_t header_bytes[48];
  message_header_encode(header, header_bytes);
  uint8_t tag[16];
  compute_tag(message_key, header_bytes, 48, ciphertext->data, ciphertext->len, tag);
  if (!heapless_vec_extend_from_slice(ciphertext, tag, 16)) return SESSION_ERROR_INVALID_FORMAT;
  // Update send count
  session->send_count += 1;
  // Check if ratcheting is needed
  if (should_ratchet(session)) advance_send_ratchet(session);
  return SESSION_ERROR_NONE;
}

// Decrypt a message
static SessionError session_decrypt(Session* session, const MessageHeader* header,const uint8_t* ciphertext, size_t ct_len,HeaplessVec* plaintext) 
{
  if (!session->established) return SESSION_ERROR_NOT_ESTABLISHED;
  if (ct_len < 16) return SESSION_ERROR_INVALID_FORMAT;
  // Check if we need to advance receive ratchet
  if (memcmp(header->dh_public, session->recv_ratchet_public, 32) != 0) 
  {
    // Skip message keys from previous chain
    SessionError err = skip_message_keys(session, header->prev_chain_len);
    if (err != SESSION_ERROR_NONE) return err;
    // Advance receive ratchet
    err = advance_recv_ratchet(session, header->dh_public);
    if (err != SESSION_ERROR_NONE) return err;
  }
  // Get message key
  uint8_t message_key[32];
  SessionError err = get_message_key(session, header, message_key);
  if (err != SESSION_ERROR_NONE) return err;
  // Verify tag
  size_t tag_start = ct_len - 16;
  const uint8_t* received_tag = ciphertext + tag_start;
  const uint8_t* ct_without_tag = ciphertext;
  uint8_t header_bytes[48];
  message_header_encode(header, header_bytes);
  uint8_t expected_tag[16];
  compute_tag(message_key, header_bytes, 48, ct_without_tag, tag_start, expected_tag);
  if (!constant_time_eq(received_tag, 16, expected_tag, 16)) return SESSION_ERROR_DECRYPTION_FAILED;
  // Decrypt ciphertext
  heapless_vec_init(plaintext);
  if (!heapless_vec_extend_from_slice(plaintext, ct_without_tag, tag_start)) return SESSION_ERROR_INVALID_FORMAT;
  // Prepare nonce for CTR mode
  uint8_t nonce[16] = {0};
  memcpy(nonce, &header->message_num, 8);
  // Decrypt using AES-256-CTR
  Aes256 aes;
  aes256_new(&aes, message_key);
  uint8_t keystream[16];
  uint64_t block_counter = 0;
  for (size_t i = 0; i < plaintext->len; i += 16) 
  {
    size_t chunk_size = (plaintext->len - i < 16) ? (plaintext->len - i) : 16;
    // Generate keystream block
    memcpy(keystream, nonce, 8);
    memcpy(keystream + 8, &block_counter, 8);
    aes256_encrypt_block(&aes, keystream);
    // XOR with ciphertext
    for (size_t j = 0; j < chunk_size; j++) plaintext->data[i + j] ^= keystream[j]; 
    block_counter += 1;
  }
  return SESSION_ERROR_NONE;
}

// Derive session hint
static void session_derive_session_hint(const Session* session, uint64_t epoch, uint8_t hint[4]) 
{
  uint8_t input[40];
  memcpy(input, session->root_key, 32);
  memcpy(input + 32, &epoch, 8);
  hkdf_derive(input, 40, (const uint8_t*)"hint", 4, SESSION_HINT_INFO,strlen((const char*)SESSION_HINT_INFO), hint, 4);
}

// Serialize session to bytes
static void session_serialize(const Session* session, uint8_t buf[SESSION_SERIALIZED_SIZE]) 
{
  size_t pos = 0;
  memcpy(buf + pos, session->root_key, 32);
  pos += 32;
  memcpy(buf + pos, session->send_chain_key, 32);
  pos += 32;
  memcpy(buf + pos, session->recv_chain_key, 32);
  pos += 32;
  memcpy(buf + pos, session->send_ratchet_private, 32);
  pos += 32;
  memcpy(buf + pos, session->send_ratchet_public, 32);
  pos += 32;
  memcpy(buf + pos, session->recv_ratchet_public, 32);
  pos += 32;
  memcpy(buf + pos, &session->send_count, 8);
  pos += 8;
  memcpy(buf + pos, &session->recv_count, 8);
  pos += 8;
  memcpy(buf + pos, &session->prev_recv_chain, 8);
  pos += 8;
  memcpy(buf + pos, &session->last_ratchet_time, 8);
  pos += 8;
  buf[pos] = session->established ? 1 : 0;
}

// Deserialize session from bytes
static bool session_deserialize(const uint8_t* data, size_t len, Session* session) 
{
  if (len < SESSION_SERIALIZED_SIZE) return false;
  size_t pos = 0;
  memcpy(session->root_key, data + pos, 32);
  pos += 32;
  memcpy(session->send_chain_key, data + pos, 32);
  pos += 32;
  memcpy(session->recv_chain_key, data + pos, 32);
  pos += 32;
  memcpy(session->send_ratchet_private, data + pos, 32);
  pos += 32;
  memcpy(session->send_ratchet_public, data + pos, 32);
  pos += 32;
  memcpy(session->recv_ratchet_public, data + pos, 32);
  pos += 32;
  memcpy(&session->send_count, data + pos, 8);
  pos += 8;
  memcpy(&session->recv_count, data + pos, 8);
  pos += 8;
  memcpy(&session->prev_recv_chain, data + pos, 8);
  pos += 8;
  memcpy(&session->last_ratchet_time, data + pos, 8);
  pos += 8;
  session->established = (data[pos] != 0);
  // Initialize empty skipped keys map
  skipped_keys_map_init(&session->skipped_keys);
  return true;
}

// Session manager entry
typedef struct 
{
  uint8_t peer_key[8];
  Session session;
  bool occupied;
} SessionEntry;

// Session manager structure
typedef struct 
{
  SessionEntry entries[MAX_PERSISTED_SESSIONS];
  size_t count;
} SessionManager;

// Initialize session manager
static void session_manager_new(SessionManager* manager) 
{
  memset(manager->entries, 0, sizeof(manager->entries));
  manager->count = 0;
}

// Get session by peer public key
static Session* session_manager_get_session(SessionManager* manager, const uint8_t peer_public[32]) 
{
  uint8_t key[8];
  memcpy(key, peer_public, 8);
  for (size_t i = 0; i < MAX_PERSISTED_SESSIONS; i++) 
  {
    if (manager->entries[i].occupied && memcmp(manager->entries[i].peer_key, key, 8) == 0) return &manager->entries[i].session;
  }
  return NULL;
}

// Create a new session
static void session_manager_create_session(SessionManager* manager, const SessionParams* params) 
{
  uint8_t key[8];
  memcpy(key, params->their_public, 8);
  // Find existing or empty slot
  for (size_t i = 0; i < MAX_PERSISTED_SESSIONS; i++) 
  {
    if (!manager->entries[i].occupied || memcmp(manager->entries[i].peer_key, key, 8) == 0) 
    {
      memcpy(manager->entries[i].peer_key, key, 8);
      session_new(&manager->entries[i].session, params);
      manager->entries[i].occupied = true;
      if (i >= manager->count) manager->count = i + 1;
      return;
    }
  }
}

// Remove a session
static void session_manager_remove_session(SessionManager* manager, const uint8_t peer_public[32]) 
{
  uint8_t key[8];
  memcpy(key, peer_public, 8);
  for (size_t i = 0; i < MAX_PERSISTED_SESSIONS; i++) 
  {
    if (manager->entries[i].occupied && memcmp(manager->entries[i].peer_key, key, 8) == 0) 
    {
      manager->entries[i].occupied = false;
      if (i == manager->count - 1) 
      {
        // Adjust count if removing last entry
        while (manager->count > 0 && !manager->entries[manager->count - 1].occupied) manager->count--;
      }
      return;
    }
  }
}

// Get session count
static size_t session_manager_session_count(const SessionManager* manager) 
{
  size_t count = 0;
  for (size_t i = 0; i < MAX_PERSISTED_SESSIONS; i++) 
  {
    if (manager->entries[i].occupied) count++;
  }
  return count;
}

// Save sessions to NVS (xtensa architecture)
#ifdef __XTENSA__
static SessionError session_manager_save_to_nvs(const SessionManager* manager) 
{
  nvs_handle_t handle;
  esp_err_t err;
  // Open NVS namespace
  err = nvs_open(NVS_SESSION_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) 
  {
    // Try to initialize NVS
    nvs_flash_init();
    err = nvs_open(NVS_SESSION_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return SESSION_ERROR_KEY_DERIVATION_FAILED;
  }
  // Get actual session count
  size_t active_count = session_manager_session_count(manager);
  // Save session count
  nvs_set_u32(handle, "sess_count", (uint32_t)active_count);
  // Save each session
  uint32_t idx = 0;
  for (size_t i = 0; i < MAX_PERSISTED_SESSIONS && idx < active_count; i++) 
  {
    if (!manager->entries[i].occupied) continue;
    // Create key name
    char key_name[16];
    if (idx < 10) 
    {
      key_name[0] = 's';
      key_name[1] = 'e';
      key_name[2] = 's';
      key_name[3] = 's';
      key_name[4] = '_';
      key_name[5] = '0' + (char)idx;
      key_name[6] = '\0';
    } 
    else 
    {
      key_name[0] = 's';
      key_name[1] = 'e';
      key_name[2] = 's';
      key_name[3] = 's';
      key_name[4] = '_';
      key_name[5] = '0' + (char)(idx / 10);
      key_name[6] = '0' + (char)(idx % 10);
      key_name[7] = '\0';
    }
    // Serialize session with peer key
    uint8_t blob[8 + SESSION_SERIALIZED_SIZE];
    memcpy(blob, manager->entries[i].peer_key, 8);
    session_serialize(&manager->entries[i].session, blob + 8);
    // Save to NVS
    nvs_set_blob(handle, key_name, blob, sizeof(blob));
    idx++;
  }
  // Commit changes
  nvs_commit(handle);
  nvs_close(handle);
  return SESSION_ERROR_NONE;
}

// Load sessions from NVS (xtensa architecture)
static size_t session_manager_load_from_nvs(SessionManager* manager) 
{
  nvs_handle_t handle;
  esp_err_t err;
  // Open NVS namespace
  err = nvs_open(NVS_SESSION_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) return 0;
  // Get session count
  uint32_t count = 0;
  if (nvs_get_u32(handle, "sess_count", &count) != ESP_OK) 
  {
    nvs_close(handle);
    return 0;
  }
  // Limit count to maximum
  if (count > MAX_PERSISTED_SESSIONS) count = MAX_PERSISTED_SESSIONS;
  // Load each session
  size_t loaded = 0;
  for (uint32_t idx = 0; idx < count; idx++) 
  {
    // Create key name
    char key_name[16];
    if (idx < 10) 
    {
      key_name[0] = 's';
      key_name[1] = 'e';
      key_name[2] = 's';
      key_name[3] = 's';
      key_name[4] = '_';
      key_name[5] = '0' + (char)idx;
      key_name[6] = '\0';
    } 
    else 
    {
      key_name[0] = 's';
      key_name[1] = 'e';
      key_name[2] = 's';
      key_name[3] = 's';
      key_name[4] = '_';
      key_name[5] = '0' + (char)(idx / 10);
      key_name[6] = '0' + (char)(idx % 10);
      key_name[7] = '\0';
    }
    // Load blob
    uint8_t blob[8 + SESSION_SERIALIZED_SIZE];
    size_t blob_len = sizeof(blob);
    if (nvs_get_blob(handle, key_name, blob, &blob_len) == ESP_OK && blob_len == sizeof(blob)) 
    {
      // Find empty slot
      for (size_t i = 0; i < MAX_PERSISTED_SESSIONS; i++) 
      {
        if (!manager->entries[i].occupied) 
        {
          // Deserialize session
          memcpy(manager->entries[i].peer_key, blob, 8);
          if (session_deserialize(blob + 8, SESSION_SERIALIZED_SIZE,&manager->entries[i].session)) 
          {
            manager->entries[i].occupied = true;
            loaded++;
            if (i >= manager->count) manager->count = i + 1;
          }
          break;
        }
      }
    }
  }
  nvs_close(handle);
  return loaded;
}
#else
// Stub implementations for non-xtensa platforms
static SessionError session_manager_save_to_nvs(const SessionManager* manager) 
{
  (void)manager;
  return SESSION_ERROR_NONE;
}

static size_t session_manager_load_from_nvs(SessionManager* manager) 
{
  (void)manager;
  return 0;
}
#endif
