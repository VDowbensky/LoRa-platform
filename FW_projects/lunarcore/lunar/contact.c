#include "contact.h"


// ContactHello constructor (new)
ContactHelloError contact_hello_new(
    ContactHello* hello,
    uint64_t timestamp,
    const uint8_t* did,
    size_t did_len,
    const uint8_t ed25519_public[32],
    const uint8_t x25519_public[32],
    const uint8_t* name,
    size_t name_len,
    const uint8_t* avatar_hash)  // NULL means None 
{
  if (did_len > MAX_DID_LENGTH) return CONTACT_HELLO_ERROR_DID_TOO_LONG;
  if (name_len > MAX_NAME_LENGTH) return CONTACT_HELLO_ERROR_NAME_TOO_LONG;
  // Initialize DID vector
  heapless_vec128_init(&hello->did);
  ContactHelloError err = heapless_vec128_extend_from_slice(&hello->did, did, did_len);
  if (err != CONTACT_HELLO_ERROR_NONE) return err;
  // Initialize name vector
  heapless_vec64_init(&hello->name);
  err = heapless_vec64_extend_from_slice(&hello->name, name, name_len);
  if (err != CONTACT_HELLO_ERROR_NONE) return err;
  // Set fields
  hello->version = CONTACT_HELLO_VERSION;
  hello->timestamp = timestamp;
  memcpy(hello->ed25519_public, ed25519_public, 32);
  memcpy(hello->x25519_public, x25519_public, 32);
  // Set avatar hash (unwrap_or default)
  if (avatar_hash != NULL) memcpy(hello->avatar_hash, avatar_hash, 32);
  else memset(hello->avatar_hash, 0, 32);
  // Initialize signature to zero
  memset(hello->signature, 0, 64);
  return CONTACT_HELLO_ERROR_NONE;
}

// Helper function: Encode ContactHello for signing
static void contact_hello_encode_for_signing(const ContactHello* hello, HeaplessVec256* buf) 
{
  heapless_vec256_init(buf);
  // Push version
  heapless_vec256_push(buf, hello->version);
  // Extend timestamp (little-endian)
  uint8_t ts_bytes[8];
  ts_bytes[0] = (uint8_t)(hello->timestamp & 0xFF);
  ts_bytes[1] = (uint8_t)((hello->timestamp >> 8) & 0xFF);
  ts_bytes[2] = (uint8_t)((hello->timestamp >> 16) & 0xFF);
  ts_bytes[3] = (uint8_t)((hello->timestamp >> 24) & 0xFF);
  ts_bytes[4] = (uint8_t)((hello->timestamp >> 32) & 0xFF);
  ts_bytes[5] = (uint8_t)((hello->timestamp >> 40) & 0xFF);
  ts_bytes[6] = (uint8_t)((hello->timestamp >> 48) & 0xFF);
  ts_bytes[7] = (uint8_t)((hello->timestamp >> 56) & 0xFF);
  heapless_vec256_extend_from_slice(buf, ts_bytes, 8);
  // Extend DID length (little-endian u16)
  uint16_t did_len = (uint16_t)hello->did.len;
  uint8_t did_len_bytes[2];
  did_len_bytes[0] = (uint8_t)(did_len & 0xFF);
  did_len_bytes[1] = (uint8_t)((did_len >> 8) & 0xFF);
  heapless_vec256_extend_from_slice(buf, did_len_bytes, 2);
  // Extend DID
  heapless_vec256_extend_from_slice(buf, hello->did.data, hello->did.len);
  // Extend ed25519 public key
  heapless_vec256_extend_from_slice(buf, hello->ed25519_public, 32);
  // Extend x25519 public key
  heapless_vec256_extend_from_slice(buf, hello->x25519_public, 32);
  // Push name length (u8)
  heapless_vec256_push(buf, (uint8_t)hello->name.len);
  // Extend name
  heapless_vec256_extend_from_slice(buf, hello->name.data, hello->name.len);
  // Extend avatar hash
  heapless_vec256_extend_from_slice(buf, hello->avatar_hash, 32);
}

// ContactHello sign method
void contact_hello_sign(ContactHello* hello, const uint8_t private_key[32]) 
{
  HeaplessVec256 data_to_sign;
  contact_hello_encode_for_signing(hello, &data_to_sign);
  Ed25519Signature sig;
  ed25519_sign(&sig, private_key, data_to_sign.data, data_to_sign.len);
  memcpy(hello->signature, sig.bytes, 64);
}

// ContactHello verify method
ContactHelloError contact_hello_verify(const ContactHello* hello, bool* result) 
{
  HeaplessVec256 data;
  contact_hello_encode_for_signing(hello, &data);
  Ed25519Signature sig;
  memcpy(sig.bytes, hello->signature, 64);
  *result = ed25519_verify(hello->ed25519_public, data.data, data.len, &sig);
  return CONTACT_HELLO_ERROR_NONE;
}

// ContactHello encode method
void contact_hello_encode(const ContactHello* hello, HeaplessVec512* buf) 
{
  HeaplessVec256 signed_data;
  contact_hello_encode_for_signing(hello, &signed_data);
  heapless_vec512_init(buf);
  heapless_vec512_extend_from_slice(buf, signed_data.data, signed_data.len);
  heapless_vec512_extend_from_slice(buf, hello->signature, 64);
}

// ContactHello decode method
ContactHelloError contact_hello_decode(ContactHello* hello, const uint8_t* data, size_t data_len) 
{
  // Minimum size check
  if (data_len < 1 + 8 + 2 + 32 + 32 + 1 + 32 + 64) return CONTACT_HELLO_ERROR_INVALID_FORMAT;
  size_t pos = 0;
  // Read version
  uint8_t version = data[pos];
  if (version != CONTACT_HELLO_VERSION) return CONTACT_HELLO_ERROR_INVALID_VERSION;
  pos += 1;
  // Read timestamp (little-endian)
  uint8_t ts_bytes[8];
  memcpy(ts_bytes, &data[pos], 8);
  uint64_t timestamp = ((uint64_t)ts_bytes[0]) |
          ((uint64_t)ts_bytes[1] << 8) |
          ((uint64_t)ts_bytes[2] << 16) |
          ((uint64_t)ts_bytes[3] << 24) |
          ((uint64_t)ts_bytes[4] << 32) |
          ((uint64_t)ts_bytes[5] << 40) |
          ((uint64_t)ts_bytes[6] << 48) |
          ((uint64_t)ts_bytes[7] << 56);
  pos += 8;
  // Read DID length (little-endian u16)
  uint8_t did_len_bytes[2];
  memcpy(did_len_bytes, &data[pos], 2);
  uint16_t did_len = ((uint16_t)did_len_bytes[0]) | ((uint16_t)did_len_bytes[1] << 8);
  pos += 2;
  if (did_len > MAX_DID_LENGTH || pos + did_len > data_len) return CONTACT_HELLO_ERROR_DID_TOO_LONG;
  // Read DID
  HeaplessVec128 did;
  heapless_vec128_init(&did);
  ContactHelloError err = heapless_vec128_extend_from_slice(&did, &data[pos], did_len);
  if (err != CONTACT_HELLO_ERROR_NONE) return err;
  pos += did_len;
  // Read ed25519 public key
  if (pos + 32 > data_len) return CONTACT_HELLO_ERROR_INVALID_FORMAT;
  uint8_t ed25519_public[32];
  memcpy(ed25519_public, &data[pos], 32);
  pos += 32;
  // Read x25519 public key
  if (pos + 32 > data_len) return CONTACT_HELLO_ERROR_INVALID_FORMAT;
  uint8_t x25519_public[32];
  memcpy(x25519_public, &data[pos], 32);
  pos += 32;
  // Read name length
  if (pos >= data_len) return CONTACT_HELLO_ERROR_INVALID_FORMAT;
  uint8_t name_len = data[pos];
  pos += 1;
  if (name_len > MAX_NAME_LENGTH || pos + name_len > data_len) return CONTACT_HELLO_ERROR_NAME_TOO_LONG;
  // Read name
  HeaplessVec64 name;
  heapless_vec64_init(&name);
  err = heapless_vec64_extend_from_slice(&name, &data[pos], name_len);
  if (err != CONTACT_HELLO_ERROR_NONE) return err;
  pos += name_len;
  // Read avatar hash
  if (pos + 32 > data_len) return CONTACT_HELLO_ERROR_INVALID_FORMAT;
  uint8_t avatar_hash[32];
  memcpy(avatar_hash, &data[pos], 32);
  pos += 32;
  // Read signature
  if (pos + 64 > data_len) return CONTACT_HELLO_ERROR_INVALID_FORMAT;
  uint8_t signature[64];
  memcpy(signature, &data[pos], 64);
  // Populate result
  hello->version = version;
  hello->timestamp = timestamp;
  hello->did = did;
  memcpy(hello->ed25519_public, ed25519_public, 32);
  memcpy(hello->x25519_public, x25519_public, 32);
  hello->name = name;
  memcpy(hello->avatar_hash, avatar_hash, 32);
  memcpy(hello->signature, signature, 64);
  return CONTACT_HELLO_ERROR_NONE;
}

// ContactHello to_qr_data method
void contact_hello_to_qr_data(const ContactHello* hello, HeaplessVec512* buf) 
{
  heapless_vec512_init(buf);
  // Add prefix "YCH:"
  const uint8_t prefix[] = {'Y', 'C', 'H', ':'};
  heapless_vec512_extend_from_slice(buf, prefix, 4);
  // Encode and append
  HeaplessVec512 encoded;
  contact_hello_encode(hello, &encoded);
  heapless_vec512_extend_from_slice(buf, encoded.data, encoded.len);
}

// ContactHello from_qr_data method
ContactHelloError contact_hello_from_qr_data(ContactHello* hello, const uint8_t* data, size_t data_len) 
{
  if (data_len < 4 || memcmp(data, "YCH:", 4) != 0) return CONTACT_HELLO_ERROR_INVALID_FORMAT;
  return contact_hello_decode(hello, &data[4], data_len - 4);
}

// ContactHello fingerprint method
void contact_hello_fingerprint(const ContactHello* hello, uint8_t fingerprint[8]) 
{
  uint8_t hash[32];
  sha256_hash(hash, hello->ed25519_public, 32);
  memcpy(fingerprint, hash, 8);
}

// Contact from_hello constructor
void contact_from_hello(Contact* contact, const ContactHello* hello) 
{
  contact->hello = *hello;
  contact->trust = TRUST_LEVEL_UNKNOWN;
  heapless_vec64_init(&contact->petname);
  contact->last_seen = 0;
  contact->message_count = 0;
}

// Contact set_petname method
void contact_set_petname(Contact* contact, const uint8_t* name, size_t name_len) 
{
  heapless_vec64_clear(&contact->petname);
  heapless_vec64_extend_from_slice(&contact->petname, name, name_len);
}

// Contact display_name method
const uint8_t* contact_display_name(const Contact* contact, size_t* len) 
{
  if (contact->petname.len > 0) 
  {
    *len = contact->petname.len;
    return contact->petname.data;
  } 
  else 
  {
    *len = contact->hello.name.len;
    return contact->hello.name.data;
  }
}

// Contact serialize method
void contact_serialize(const Contact* contact, HeaplessVec512* buf) 
{
  heapless_vec512_init(buf);
  // Encode hello
  HeaplessVec512 hello_encoded;
  contact_hello_encode(&contact->hello, &hello_encoded);
  // Write hello length (little-endian u16)
  uint16_t hello_len = (uint16_t)hello_encoded.len;
  uint8_t hello_len_bytes[2];
  hello_len_bytes[0] = (uint8_t)(hello_len & 0xFF);
  hello_len_bytes[1] = (uint8_t)((hello_len >> 8) & 0xFF);
  heapless_vec512_extend_from_slice(buf, hello_len_bytes, 2);
  heapless_vec512_extend_from_slice(buf, hello_encoded.data, hello_encoded.len);
  // Write trust level
  heapless_vec512_push(buf, (uint8_t)contact->trust);
  // Write petname length and data
  heapless_vec512_push(buf, (uint8_t)contact->petname.len);
  heapless_vec512_extend_from_slice(buf, contact->petname.data, contact->petname.len);
  // Write last_seen (little-endian u64)
  uint8_t last_seen_bytes[8];
  last_seen_bytes[0] = (uint8_t)(contact->last_seen & 0xFF);
  last_seen_bytes[1] = (uint8_t)((contact->last_seen >> 8) & 0xFF);
  last_seen_bytes[2] = (uint8_t)((contact->last_seen >> 16) & 0xFF);
  last_seen_bytes[3] = (uint8_t)((contact->last_seen >> 24) & 0xFF);
  last_seen_bytes[4] = (uint8_t)((contact->last_seen >> 32) & 0xFF);
  last_seen_bytes[5] = (uint8_t)((contact->last_seen >> 40) & 0xFF);
  last_seen_bytes[6] = (uint8_t)((contact->last_seen >> 48) & 0xFF);
  last_seen_bytes[7] = (uint8_t)((contact->last_seen >> 56) & 0xFF);
  heapless_vec512_extend_from_slice(buf, last_seen_bytes, 8);
  // Write message_count (little-endian u32)
  uint8_t count_bytes[4];
  count_bytes[0] = (uint8_t)(contact->message_count & 0xFF);
  count_bytes[1] = (uint8_t)((contact->message_count >> 8) & 0xFF);
  count_bytes[2] = (uint8_t)((contact->message_count >> 16) & 0xFF);
  count_bytes[3] = (uint8_t)((contact->message_count >> 24) & 0xFF);
  heapless_vec512_extend_from_slice(buf, count_bytes, 4);
}

// Contact deserialize method
bool contact_deserialize(Contact* contact, const uint8_t* data, size_t data_len) 
{
  if (data_len < 2) return false;
  size_t pos = 0;
  // Read hello length (little-endian u16)
  uint8_t hello_len_bytes[2];
  memcpy(hello_len_bytes, &data[pos], 2);
  uint16_t hello_len = ((uint16_t)hello_len_bytes[0]) | ((uint16_t)hello_len_bytes[1] << 8);
  pos += 2;
  if (pos + hello_len > data_len) return false;
  // Decode hello
  ContactHello hello;
  if (contact_hello_decode(&hello, &data[pos], hello_len) != CONTACT_HELLO_ERROR_NONE) return false;
  pos += hello_len;
  // Read trust level
  if (pos >= data_len) return false;
  TrustLevel trust;
  switch (data[pos]) 
  {
    case 0: trust = TRUST_LEVEL_UNKNOWN; break;
    case 1: trust = TRUST_LEVEL_SEEN; break;
    case 2: trust = TRUST_LEVEL_VERIFIED; break;
    case 3: trust = TRUST_LEVEL_TRUSTED; break;
    default: trust = TRUST_LEVEL_UNKNOWN; break;
  }
  pos += 1;
  // Read petname length
  if (pos >= data_len) return false;
  uint8_t petname_len = data[pos];
  pos += 1;
  if (pos + petname_len > data_len) return false;
  // Read petname
  HeaplessVec64 petname;
  heapless_vec64_init(&petname);
  heapless_vec64_extend_from_slice(&petname, &data[pos], petname_len);
  pos += petname_len;
  // Read last_seen (little-endian u64)
  if (pos + 8 > data_len) return false;
  uint8_t last_seen_bytes[8];
  memcpy(last_seen_bytes, &data[pos], 8);
  uint64_t last_seen = ((uint64_t)last_seen_bytes[0]) |
                        ((uint64_t)last_seen_bytes[1] << 8) |
                        ((uint64_t)last_seen_bytes[2] << 16) |
                        ((uint64_t)last_seen_bytes[3] << 24) |
                        ((uint64_t)last_seen_bytes[4] << 32) |
                        ((uint64_t)last_seen_bytes[5] << 40) |
                        ((uint64_t)last_seen_bytes[6] << 48) |
                        ((uint64_t)last_seen_bytes[7] << 56);
  pos += 8;
  // Read message_count (little-endian u32)
  if (pos + 4 > data_len) return false;
  uint8_t count_bytes[4];
  memcpy(count_bytes, &data[pos], 4);
  uint32_t message_count = ((uint32_t)count_bytes[0]) |
                            ((uint32_t)count_bytes[1] << 8) |
                            ((uint32_t)count_bytes[2] << 16) |
                            ((uint32_t)count_bytes[3] << 24);
  // Populate result
  contact->hello = hello;
  contact->trust = trust;
  contact->petname = petname;
  contact->last_seen = last_seen;
  contact->message_count = message_count;
  return true;
}

// Contact key method
void contact_key(const Contact* contact, uint8_t key[8]) 
{
  contact_hello_fingerprint(&contact->hello, key);
}

// NVS namespace constant
static const char NVS_CONTACT_NAMESPACE[] = "contacts";

// ContactStore constructor
void contact_store_new(ContactStore* store) 
{
  store->count = 0;
  for (size_t i = 0; i < MAX_CONTACTS; i++) store->entries[i].occupied = false;
}

// Helper function: Simple FNV-1a hash for 8-byte keys
static size_t hash_key(const uint8_t key[8]) 
{
  size_t hash = 2166136261u;
  for (int i = 0; i < 8; i++) 
  {
    hash ^= key[i];
    hash *= 16777619u;
  }
  return hash % MAX_CONTACTS;
}

// Helper function: Compare 8-byte keys
static bool keys_equal(const uint8_t key1[8], const uint8_t key2[8]) 
{
  return memcmp(key1, key2, 8) == 0;
}

// ContactStore add method
ContactHelloError contact_store_add(ContactStore* store, const Contact* contact) 
{
  uint8_t key[8];
  contact_key(contact, key);
  // Check if key already exists (update case)
  for (size_t i = 0; i < MAX_CONTACTS; i++) 
  {
    if (store->entries[i].occupied && keys_equal(store->entries[i].key, key)) 
    {
      store->entries[i].value = *contact;
      return CONTACT_HELLO_ERROR_NONE;
    }
  }
  // Find empty slot using hash-based probing
  size_t idx = hash_key(key);
  for (size_t i = 0; i < MAX_CONTACTS; i++) 
  {
    size_t probe_idx = (idx + i) % MAX_CONTACTS;
    if (!store->entries[probe_idx].occupied) 
    {
      memcpy(store->entries[probe_idx].key, key, 8);
      store->entries[probe_idx].value = *contact;
      store->entries[probe_idx].occupied = true;
      store->count++;
      return CONTACT_HELLO_ERROR_NONE;
    }
  }
  return CONTACT_HELLO_ERROR_BUFFER_TOO_SMALL;
}

// ContactStore get method
const Contact* contact_store_get(const ContactStore* store, const uint8_t fingerprint[8]) 
{
  for (size_t i = 0; i < MAX_CONTACTS; i++) 
  {
    if (store->entries[i].occupied && keys_equal(store->entries[i].key, fingerprint)) return &store->entries[i].value;
  }
  return NULL;
}

// ContactStore get_mut method
Contact* contact_store_get_mut(ContactStore* store, const uint8_t fingerprint[8]) 
{
  for (size_t i = 0; i < MAX_CONTACTS; i++) 
  {
    if (store->entries[i].occupied && keys_equal(store->entries[i].key, fingerprint)) return &store->entries[i].value;
  }
  return NULL;
}

// ContactStore remove method
bool contact_store_remove(ContactStore* store, const uint8_t fingerprint[8], Contact* out_contact) 
{
  for (size_t i = 0; i < MAX_CONTACTS; i++) 
  {
    if (store->entries[i].occupied && keys_equal(store->entries[i].key, fingerprint)) 
    {
      if (out_contact != NULL) *out_contact = store->entries[i].value;
      store->entries[i].occupied = false;
      store->count--;
      return true;
    }
  }
  return false;
}

// ContactStore len method
size_t contact_store_len(const ContactStore* store) 
{
  return store->count;
}

// ContactStore is_empty method
bool contact_store_is_empty(const ContactStore* store) 
{
  return store->count == 0;
}

// ContactStore find_by_pubkey method
const Contact* contact_store_find_by_pubkey(const ContactStore* store, const uint8_t ed25519_public[32]) 
{
  for (size_t i = 0; i < MAX_CONTACTS; i++) 
  {
    if (store->entries[i].occupied) 
    {
      if (memcmp(store->entries[i].value.hello.ed25519_public, ed25519_public, 32) == 0) return &store->entries[i].value;
    }
  }
  return NULL;
}

// ContactStore save_to_nvs method (xtensa only)
#ifdef __XTENSA__
ContactHelloError contact_store_save_to_nvs(const ContactStore* store) 
{
  nvs_handle_t handle = 0;
  // Open NVS namespace
  esp_err_t err = nvs_open(NVS_CONTACT_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) 
  {
    // Try to initialize NVS flash
    nvs_flash_init();
    err = nvs_open(NVS_CONTACT_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return CONTACT_HELLO_ERROR_BUFFER_TOO_SMALL;
  }
  // Save count
  nvs_set_u32(handle, "cnt_count", (uint32_t)store->count);
  // Save each contact
  uint32_t idx = 0;
  for (size_t i = 0; i < MAX_CONTACTS; i++) 
  {
    if (!store->entries[i].occupied) continue;
    // Generate key name "cnt_0", "cnt_1", etc.
    char key_name[16] = {0};
    key_name[0] = 'c';
    key_name[1] = 'n';
    key_name[2] = 't';
    key_name[3] = '_';
    if (idx < 10) 
    {
      key_name[4] = '0' + (char)idx;
      key_name[5] = '\0';
    } 
    else 
    {
      key_name[4] = '0' + (char)(idx / 10);
      key_name[5] = '0' + (char)(idx % 10);
      key_name[6] = '\0';
    }
    // Serialize contact
    HeaplessVec512 blob;
    contact_serialize(&store->entries[i].value, &blob);
    // Save to NVS
    nvs_set_blob(handle, key_name, blob.data, blob.len);
    idx++;
    }

    // Commit and close
    nvs_commit(handle);
    nvs_close(handle);

    // Log info (assuming log.h is available)
    // printf("Saved %zu contacts to NVS\n", store->count);

    return CONTACT_HELLO_ERROR_NONE;
  }
#else
// Non-xtensa stub
ContactHelloError contact_store_save_to_nvs(const ContactStore* store) 
{
  (void)store;  // Unused parameter
  return CONTACT_HELLO_ERROR_NONE;
}
#endif

// ContactStore load_from_nvs method (xtensa only)
#ifdef __XTENSA__
ContactHelloError contact_store_load_from_nvs(ContactStore* store, size_t* loaded_count) 
{
  nvs_handle_t handle = 0;
  // Open NVS namespace (read-only)
  esp_err_t err = nvs_open(NVS_CONTACT_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) 
  {
    *loaded_count = 0;
    return CONTACT_HELLO_ERROR_NONE;
  }
  // Read count
  uint32_t count = 0;
  if (nvs_get_u32(handle, "cnt_count", &count) != ESP_OK) 
  {
    nvs_close(handle);
    *loaded_count = 0;
    return CONTACT_HELLO_ERROR_NONE;
  }
  // Limit count to MAX_CONTACTS
  if (count > MAX_CONTACTS) count = MAX_CONTACTS;
  // Load each contact
  size_t loaded = 0;
  for (uint32_t idx = 0; idx < count; idx++) 
  {
    // Generate key name "cnt_0", "cnt_1", etc.
    char key_name[16] = {0};
    key_name[0] = 'c';
    key_name[1] = 'n';
    key_name[2] = 't';
    key_name[3] = '_';
    if (idx < 10) 
    {
      key_name[4] = '0' + (char)idx;
      key_name[5] = '\0';
    } 
    else 
    {
      key_name[4] = '0' + (char)(idx / 10);
      key_name[5] = '0' + (char)(idx % 10);
      key_name[6] = '\0';
    }
    // Read blob
    uint8_t blob[512];
    size_t blob_len = sizeof(blob);
    if (nvs_get_blob(handle, key_name, blob, &blob_len) == ESP_OK && blob_len > 0) 
    {
      // Deserialize contact
      Contact contact;
      if (contact_deserialize(&contact, blob, blob_len)) 
      {
        uint8_t key[8];
        contact_key(&contact, key);
        // Add to store
        bool found = false;
        for (size_t i = 0; i < MAX_CONTACTS; i++) 
        {
          if (!store->entries[i].occupied) 
          {
            memcpy(store->entries[i].key, key, 8);
            store->entries[i].value = contact;
            store->entries[i].occupied = true;
            found = true;
            break;
          }
        }
        if (found) 
        {
          store->count++;
          loaded++;
        }
      }
    }
  }
  nvs_close(handle);
  // Log info
  // printf("Loaded %zu contacts from NVS\n", loaded);
  *loaded_count = loaded;
  return CONTACT_HELLO_ERROR_NONE;
}
#else
// Non-xtensa stub
ContactHelloError contact_store_load_from_nvs(ContactStore* store, size_t* loaded_count) 
{
  (void)store;  // Unused parameter
  *loaded_count = 0;
  return CONTACT_HELLO_ERROR_NONE;
}
#endif


