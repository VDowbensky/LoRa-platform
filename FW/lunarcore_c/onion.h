#ifndef _ONION_H_
#define _ONION_H_

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

// Constants
#define MAX_HOPS 7
#define MIN_HOPS 3
#define HOP_OVERHEAD (NODE_HINT_SIZE + AUTH_TAG_SIZE)

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

// OnionRoute structure
typedef struct 
{
  HeaplessVec_RouteHop_MAX_HOPS hops;
} OnionRoute;

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

// HeaplessVec for RouteHop with max capacity 32
typedef struct 
{
  RouteHop data[32];
  size_t len;
  size_t capacity;
} HeaplessVec_RouteHop_32;

// RouteBuilder structure
typedef struct 
{
  HeaplessVec_RouteHop_32 relays;
} RouteBuilder;

// HeaplessVec implementation for uint8_t with max capacity 320
typedef struct 
{
  uint8_t data[320];
  size_t len;
  size_t capacity;
} HeaplessVec_u8_320;

#endif
