#include "onion.h"

// Dependencies from crypto module
extern void x25519(const uint8_t private_key[32], const uint8_t public_key[32], uint8_t shared[32]);
extern void x25519_base(const uint8_t private_key[32], uint8_t public_key[32]);
extern void hkdf_derive(const uint8_t *input_key, size_t input_key_len,
                        const uint8_t *salt, size_t salt_len,
                        const uint8_t *info, size_t info_len,
                        uint8_t *output, size_t output_len);
extern void sha256_hash(const uint8_t *data, size_t len, uint8_t output[32]);
extern void aes256_new(const uint8_t key[32], void **aes_ctx);
extern void aes256_encrypt_block(void *aes_ctx, uint8_t block[16]);
extern void aes256_free(void *aes_ctx);

// Dependencies from transport module
#define NODE_HINT_SIZE 2
#define AUTH_TAG_SIZE 16

// HeaplessVec implementation for uint8_t with max capacity 256
typedef struct 
{
  uint8_t data[256];
  size_t len;
  size_t capacity;
} HeaplessVec_u8_256;

static inline bool heapless_vec_u8_256_init(HeaplessVec_u8_256 *vec) 
{
  vec->len = 0;
  vec->capacity = 256;
  memset(vec->data, 0, 256);
  return true;
}

static inline bool heapless_vec_u8_256_push(HeaplessVec_u8_256 *vec, uint8_t val) 
{
  if (vec->len >= vec->capacity) return false;
  vec->data[vec->len] = val;
  vec->len++;
  return true;
}

static inline bool heapless_vec_u8_256_extend_from_slice(HeaplessVec_u8_256 *vec, const uint8_t *data, size_t len) 
{
  if (vec->len + len > vec->capacity) return false;
  memcpy(&vec->data[vec->len], data, len);
  vec->len += len;
  return true;
}

static inline uint8_t* heapless_vec_u8_256_as_slice(HeaplessVec_u8_256 *vec) 
{
  return vec->data;
}

static inline size_t heapless_vec_u8_256_len(const HeaplessVec_u8_256 *vec) 
{
  return vec->len;
}

// HeaplessVec implementation for uint8_t with max capacity 320
typedef struct 
{
  uint8_t data[320];
  size_t len;
  size_t capacity;
} HeaplessVec_u8_320;

static inline bool heapless_vec_u8_320_init(HeaplessVec_u8_320 *vec) 
{
  vec->len = 0;
  vec->capacity = 320;
  memset(vec->data, 0, 320);
  return true;
}

static inline bool heapless_vec_u8_320_extend_from_slice(HeaplessVec_u8_320 *vec, const uint8_t *data, size_t len) 
{
  if (vec->len + len > vec->capacity) return false;
  memcpy(&vec->data[vec->len], data, len);
  vec->len += len;
  return true;
}

// Constant time equality check
static inline bool constant_time_eq(const uint8_t *a, size_t a_len, const uint8_t *b, size_t b_len) 
{
  if (a_len != b_len) return false;
  uint8_t result = 0;
  for (size_t i = 0; i < a_len; i++) result |= a[i] ^ b[i];
  // Force volatile read to prevent optimization
  volatile uint8_t volatile_result = result;
  return volatile_result == 0;
}

// Constants
#define MAX_HOPS 7
#define MIN_HOPS 3
#define HOP_OVERHEAD (NODE_HINT_SIZE + AUTH_TAG_SIZE)

static const uint8_t ONION_KEY_INFO[] = "lunarpunk-onion-key-v1";
static const size_t ONION_KEY_INFO_LEN = 22;

static const uint8_t ONION_BLIND_INFO[] = "lunarpunk-onion-blind-v1";
static const size_t ONION_BLIND_INFO_LEN = 24;

// RouteHop structure
typedef struct 
{
  uint16_t hint;
  uint8_t public_key[32];
} RouteHop;

// HeaplessVec for RouteHop with max capacity MAX_HOPS
typedef struct 
{
  RouteHop data[MAX_HOPS];
  size_t len;
  size_t capacity;
} HeaplessVec_RouteHop_MAX_HOPS;

static inline bool heapless_vec_routehop_init(HeaplessVec_RouteHop_MAX_HOPS *vec) 
{
  vec->len = 0;
  vec->capacity = MAX_HOPS;
  memset(vec->data, 0, sizeof(RouteHop) * MAX_HOPS);
  return true;
}

static inline bool heapless_vec_routehop_push(HeaplessVec_RouteHop_MAX_HOPS *vec, const RouteHop *hop) 
{
  if (vec->len >= vec->capacity) return false;
  memcpy(&vec->data[vec->len], hop, sizeof(RouteHop));
  vec->len++;
  return true;
}

static inline const RouteHop* heapless_vec_routehop_first(const HeaplessVec_RouteHop_MAX_HOPS *vec) 
{
  if (vec->len == 0) return NULL;
  return &vec->data[0];
}

static inline const RouteHop* heapless_vec_routehop_last(const HeaplessVec_RouteHop_MAX_HOPS *vec) 
{
  if (vec->len == 0) return NULL;
  return &vec->data[vec->len - 1];
}

// OnionRoute structure
typedef struct 
{
  HeaplessVec_RouteHop_MAX_HOPS hops;
} OnionRoute;

// OnionRoute functions
static inline bool onion_route_new(OnionRoute *route, const RouteHop *hops, size_t hops_len) 
{
  if (hops_len < MIN_HOPS || hops_len > MAX_HOPS) return false;
  heapless_vec_routehop_init(&route->hops);
  for (size_t i = 0; i < hops_len; i++) 
  {
    if (!heapless_vec_routehop_push(&route->hops, &hops[i])) return false;
  }
  return true;
}

static inline size_t onion_route_len(const OnionRoute *route) 
{
  return route->hops.len;
}

static inline bool onion_route_is_empty(const OnionRoute *route) 
{
  return route->hops.len == 0;
}

static inline uint16_t onion_route_entry_hint(const OnionRoute *route) 
{
  const RouteHop *first = heapless_vec_routehop_first(&route->hops);
  if (first == NULL) return 0;
  return first->hint;
}

static inline uint16_t onion_route_exit_hint(const OnionRoute *route) 
{
  const RouteHop *last = heapless_vec_routehop_last(&route->hops);
  if (last == NULL) return 0;
  return last->hint;
}

static inline size_t onion_route_overhead(const OnionRoute *route) 
{
  return route->hops.len * HOP_OVERHEAD;
}

// OnionPacket structure
typedef struct 
{
  HeaplessVec_u8_256 data;
  uint8_t num_layers;
} OnionPacket;

// OnionError enumeration
typedef enum 
{
  ONION_ERROR_INVALID_ROUTE,
  ONION_ERROR_PACKET_TOO_LARGE,
  ONION_ERROR_AUTHENTICATION_FAILED,
  ONION_ERROR_DECRYPTION_FAILED,
  ONION_ERROR_NO_MORE_LAYERS
} OnionError;

// OnionRouter structure
typedef struct 
{
  uint8_t our_private[32];
  uint8_t our_public[32];
  uint16_t our_hint;
} OnionRouter;

// OnionRouter constructor
static inline void onion_router_new(OnionRouter *router, const uint8_t private_key[32]) 
{
  memcpy(router->our_private, private_key, 32);
  x25519_base(private_key, router->our_public);
  uint8_t hint_hash[32];
  sha256_hash(router->our_public, 32, hint_hash);
  router->our_hint = ((uint16_t)hint_hash[0] << 8) | (uint16_t)hint_hash[1];
}

// Forward declarations for OnionRouter helper functions
static bool onion_router_encrypt_layer(OnionRouter *self, const uint8_t key[32], 
                                       const uint8_t *data, size_t data_len,
                                       HeaplessVec_u8_256 *output);
static bool onion_router_decrypt_layer(OnionRouter *self, const uint8_t key[32],
                                       const uint8_t *data, size_t data_len,
                                       HeaplessVec_u8_256 *output);
static void onion_router_compute_tag(OnionRouter *self, const uint8_t key[32],
                                     const uint8_t *data, size_t data_len,
                                     uint8_t tag[16]);

// OnionRouter wrap function
static inline bool onion_router_wrap(OnionRouter *self, const uint8_t *payload, size_t payload_len,
                                     const OnionRoute *route, OnionPacket *packet,
                                     OnionError *error) 
{
  if (route->hops.len < MIN_HOPS || route->hops.len > MAX_HOPS) 
  {
    if (error) *error = ONION_ERROR_INVALID_ROUTE;
    return false;
  }
  size_t total_overhead = onion_route_overhead(route);
  if (payload_len + total_overhead > 256) 
  {
    if (error) *error = ONION_ERROR_PACKET_TOO_LARGE;
    return false;
  }
  HeaplessVec_u8_256 current;
  heapless_vec_u8_256_init(&current);
  if (!heapless_vec_u8_256_extend_from_slice(&current, payload, payload_len)) 
  {
    if (error) *error = ONION_ERROR_PACKET_TOO_LARGE;
    return false;
  }
  // Process hops in reverse order
  for (int i = (int)route->hops.len - 1; i >= 0; i--) 
  {
    const RouteHop *hop = &route->hops.data[i];
    // Compute shared secret
    uint8_t shared[32];
    x25519(self->our_private, hop->public_key, shared);
    // Compute layer index
    uint8_t layer_index = (uint8_t)(route->hops.len - 1 - i);
    uint8_t key_input[33];
    memcpy(key_input, shared, 32);
    key_input[32] = layer_index;
    // Derive layer key
    uint8_t layer_key[32];
    uint8_t salt[1] = { layer_index };
    hkdf_derive(key_input, 33, salt, 1, ONION_KEY_INFO, ONION_KEY_INFO_LEN, layer_key, 32);
    // Encrypt current layer
    HeaplessVec_u8_256 encrypted;
    heapless_vec_u8_256_init(&encrypted);
    if (!onion_router_encrypt_layer(self, layer_key, current.data, current.len, &encrypted)) 
    {
      if (error) *error = ONION_ERROR_PACKET_TOO_LARGE;
      return false;
    }
    // Compute authentication tag
    uint8_t tag[16];
    onion_router_compute_tag(self, layer_key, encrypted.data, encrypted.len, tag);
    // Build new layer
    HeaplessVec_u8_256 new_layer;
    heapless_vec_u8_256_init(&new_layer);
    // Determine next hint
    uint16_t next_hint;
    if (i == (int)route->hops.len - 1) next_hint = 0;
    else next_hint = route->hops.data[i + 1].hint;
    // Add next hint (2 bytes)
    if (!heapless_vec_u8_256_push(&new_layer, (uint8_t)(next_hint >> 8))) 
    {
      if (error) *error = ONION_ERROR_PACKET_TOO_LARGE;
      return false;
    }
    if (!heapless_vec_u8_256_push(&new_layer, (uint8_t)next_hint)) 
    {
      if (error) *error = ONION_ERROR_PACKET_TOO_LARGE;
      return false;
    }
    // Add authentication tag
    if (!heapless_vec_u8_256_extend_from_slice(&new_layer, tag, 16)) 
    {
      if (error) *error = ONION_ERROR_PACKET_TOO_LARGE;
      return false;
    }
    // Add encrypted data
    if (!heapless_vec_u8_256_extend_from_slice(&new_layer, encrypted.data, encrypted.len)) 
    {
      if (error) *error = ONION_ERROR_PACKET_TOO_LARGE;
      return false;
    }
    current = new_layer;
  }
  packet->data = current;
  packet->num_layers = (uint8_t)route->hops.len;
  return true;
}

// OnionRouter unwrap function
static inline bool onion_router_unwrap(OnionRouter *self, const OnionPacket *packet,
                                      const uint8_t sender_public[32],
                                      uint16_t *next_hint_out, OnionPacket *inner_packet_out,
                                      OnionError *error) 
{
  if (packet->data.len < HOP_OVERHEAD) 
  {
    if (error) *error = ONION_ERROR_DECRYPTION_FAILED;
    return false;
  }
  uint16_t next_hint = ((uint16_t)packet->data.data[0] << 8) | (uint16_t)packet->data.data[1];
  const uint8_t *tag = &packet->data.data[2];
  const uint8_t *encrypted = &packet->data.data[18];
  size_t encrypted_len = packet->data.len - 18;
  // Compute shared secret
  uint8_t shared[32];
  x25519(self->our_private, sender_public, shared);
  // Compute layer index
  uint8_t layer_index = packet->num_layers > 0 ? packet->num_layers - 1 : 0;
  uint8_t key_input[33];
  memcpy(key_input, shared, 32);
  key_input[32] = layer_index;
  // Derive layer key
  uint8_t layer_key[32];
  uint8_t salt[1] = { layer_index };
  hkdf_derive(key_input, 33, salt, 1, ONION_KEY_INFO, ONION_KEY_INFO_LEN, layer_key, 32);
  // Verify authentication tag
  uint8_t expected_tag[16];
  onion_router_compute_tag(self, layer_key, encrypted, encrypted_len, expected_tag);
  if (!constant_time_eq(tag, 16, expected_tag, 16)) 
  {
    if (error) *error = ONION_ERROR_AUTHENTICATION_FAILED;
    return false;
  }
  // Decrypt layer
  HeaplessVec_u8_256 inner;
  heapless_vec_u8_256_init(&inner);
  if (!onion_router_decrypt_layer(self, layer_key, encrypted, encrypted_len, &inner)) 
  {
    if (error) *error = ONION_ERROR_DECRYPTION_FAILED;
    return false;
  }
  // Check if this is the final layer
  if (next_hint == 0) 
  {
    if (error) *error = ONION_ERROR_NO_MORE_LAYERS;
    return false;
  }
  // Return results
  *next_hint_out = next_hint;
  inner_packet_out->data = inner;
  inner_packet_out->num_layers = packet->num_layers > 0 ? packet->num_layers - 1 : 0;
  return true;
}

// OnionRouter unwrap_final function
static inline bool onion_router_unwrap_final(OnionRouter *self, const OnionPacket *packet,
                                             const uint8_t sender_public[32],
                                             HeaplessVec_u8_256 *payload_out,
                                             OnionError *error) 
{
  if (packet->data.len < HOP_OVERHEAD) 
  {
    if (error) *error = ONION_ERROR_DECRYPTION_FAILED;
    return false;
  }
  const uint8_t *tag = &packet->data.data[2];
  const uint8_t *encrypted = &packet->data.data[18];
  size_t encrypted_len = packet->data.len - 18;
  // Compute shared secret
  uint8_t shared[32];
  x25519(self->our_private, sender_public, shared);
  // Compute layer index
  uint8_t layer_index = packet->num_layers > 0 ? packet->num_layers - 1 : 0;
  uint8_t key_input[33];
  memcpy(key_input, shared, 32);
  key_input[32] = layer_index;
  // Derive layer key
  uint8_t layer_key[32];
  uint8_t salt[1] = { layer_index };
  hkdf_derive(key_input, 33, salt, 1, ONION_KEY_INFO, ONION_KEY_INFO_LEN, layer_key, 32);
  // Verify authentication tag
  uint8_t expected_tag[16];
  onion_router_compute_tag(self, layer_key, encrypted, encrypted_len, expected_tag);
  if (!constant_time_eq(tag, 16, expected_tag, 16)) 
  {
    if (error) *error = ONION_ERROR_AUTHENTICATION_FAILED;
    return false;
  }
  // Decrypt layer
  heapless_vec_u8_256_init(payload_out);
  if (!onion_router_decrypt_layer(self, layer_key, encrypted, encrypted_len, payload_out)) 
  {
    if (error) *error = ONION_ERROR_DECRYPTION_FAILED;
    return false;
  }
  return true;
}

// OnionRouter encrypt_layer implementation
static bool onion_router_encrypt_layer(OnionRouter *self, const uint8_t key[32],
                                       const uint8_t *data, size_t data_len,
                                       HeaplessVec_u8_256 *output) 
{
  heapless_vec_u8_256_init(output);
  if (!heapless_vec_u8_256_extend_from_slice(output, data, data_len)) return false;
  void *aes_ctx;
  aes256_new(key, &aes_ctx);
  uint8_t counter[16];
  memset(counter, 0, 16);
  uint8_t keystream[16];
  uint64_t block_num = 0;
  // Process data in 16-byte chunks
  for (size_t offset = 0; offset < output->len; offset += 16) 
  {
    size_t chunk_size = (output->len - offset < 16) ? (output->len - offset) : 16;
    // Prepare keystream block
    memcpy(keystream, counter, 16);
    memcpy(&keystream[8], &block_num, 8);
    // Encrypt the counter to generate keystream
    aes256_encrypt_block(aes_ctx, keystream);
    // XOR data with keystream
    for (size_t i = 0; i < chunk_size; i++) output->data[offset + i] ^= keystream[i];
    block_num++;
  }
  aes256_free(aes_ctx);
  return true;
}

// OnionRouter decrypt_layer implementation (same as encrypt due to CTR mode)
static bool onion_router_decrypt_layer(OnionRouter *self, const uint8_t key[32],
                                       const uint8_t *data, size_t data_len,
                                       HeaplessVec_u8_256 *output) 
{
  // CTR mode encryption and decryption are the same operation
  return onion_router_encrypt_layer(self, key, data, data_len, output);
}

// OnionRouter compute_tag implementation (HMAC-SHA256 truncated to 16 bytes)
static void onion_router_compute_tag(OnionRouter *self, const uint8_t key[32],
                                     const uint8_t *data, size_t data_len,
                                     uint8_t tag[16]) 
{
  // HMAC-SHA256 implementation
  uint8_t inner[64];
  uint8_t outer[64];
  // Initialize with padding
  memset(inner, 0x36, 64);
  memset(outer, 0x5c, 64);
  // XOR key with padding
  for (size_t i = 0; i < 32; i++) 
  {
    inner[i] ^= key[i];
    outer[i] ^= key[i];
  }
  // Inner hash: H(K XOR ipad || data)
  HeaplessVec_u8_320 inner_input;
  heapless_vec_u8_320_init(&inner_input);
  heapless_vec_u8_320_extend_from_slice(&inner_input, inner, 64);
  heapless_vec_u8_320_extend_from_slice(&inner_input, data, data_len);
  uint8_t inner_hash[32];
  sha256_hash(inner_input.data, inner_input.len, inner_hash);
  // Outer hash: H(K XOR opad || inner_hash)
  uint8_t outer_input[96];
  memcpy(outer_input, outer, 64);
  memcpy(&outer_input[64], inner_hash, 32);
  uint8_t full[32];
  sha256_hash(outer_input, 96, full);
  // Truncate to 16 bytes
  memcpy(tag, full, 16);
}

// OnionRouter getter functions
static inline uint16_t onion_router_our_hint(const OnionRouter *router) 
{
  return router->our_hint;
}

static inline const uint8_t* onion_router_our_public(const OnionRouter *router) 
{
  return router->our_public;
}

// OnionRouter derive_blinded_hint function
static inline uint16_t onion_router_derive_blinded_hint(const OnionRouter *router, uint64_t epoch) 
{
  uint8_t input[40];
  memcpy(input, router->our_public, 32);
  memcpy(&input[32], &epoch, 8);
  uint8_t blinded[2];
  uint8_t epoch_bytes[8];
  memcpy(epoch_bytes, &epoch, 8);
  hkdf_derive(input, 40, epoch_bytes, 8, ONION_BLIND_INFO, ONION_BLIND_INFO_LEN, blinded, 2);
  return ((uint16_t)blinded[0] << 8) | (uint16_t)blinded[1];
}

// HeaplessVec for RouteHop with max capacity 32
typedef struct 
{
  RouteHop data[32];
  size_t len;
  size_t capacity;
} HeaplessVec_RouteHop_32;

static inline bool heapless_vec_routehop_32_init(HeaplessVec_RouteHop_32 *vec) 
{
  vec->len = 0;
  vec->capacity = 32;
  memset(vec->data, 0, sizeof(RouteHop) * 32);
  return true;
}

static inline bool heapless_vec_routehop_32_push(HeaplessVec_RouteHop_32 *vec, const RouteHop *hop) 
{
  if (vec->len >= vec->capacity) return false;
  memcpy(&vec->data[vec->len], hop, sizeof(RouteHop));
  vec->len++;
  return true;
}

// RouteBuilder structure
typedef struct 
{
  HeaplessVec_RouteHop_32 relays;
} RouteBuilder;

// RouteBuilder constructor
static inline void route_builder_new(RouteBuilder *builder) 
{
  heapless_vec_routehop_32_init(&builder->relays);
}

// RouteBuilder add_relay function
static inline bool route_builder_add_relay(RouteBuilder *builder, uint16_t hint, const uint8_t public_key[32]) 
{
  RouteHop hop;
  hop.hint = hint;
  memcpy(hop.public_key, public_key, 32);
  return heapless_vec_routehop_32_push(&builder->relays, &hop);
}

// RouteBuilder build_route function
static inline bool route_builder_build_route(const RouteBuilder *builder, const RouteHop *destination,
                                             size_t num_hops, OnionRoute *route_out) 
{
  if (num_hops < MIN_HOPS || num_hops > MAX_HOPS) return false;
  size_t relay_count = num_hops - 1;
  if (builder->relays.len < relay_count) return false;
  HeaplessVec_RouteHop_MAX_HOPS hops;
  heapless_vec_routehop_init(&hops);
  // Add relay hops
  for (size_t i = 0; i < relay_count; i++) 
  {
    size_t relay_index = i % builder->relays.len;
    if (!heapless_vec_routehop_push(&hops, &builder->relays.data[relay_index])) return false;
  }
  // Add destination hop
  if (!heapless_vec_routehop_push(&hops, destination)) return false;
  route_out->hops = hops;
  return true;
}

// RouteBuilder relay_count function
static inline size_t route_builder_relay_count(const RouteBuilder *builder) 
{
  return builder->relays.len;
}

// RouteBuilder default constructor
static inline void route_builder_default(RouteBuilder *builder) 
{
  route_builder_new(builder);
}
