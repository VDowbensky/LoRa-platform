#ifndef _CONTACT_H_
#define _CONTACT_H_

// Include crypto headers (assuming these are available)
// These would need to be implemented or linked from your crypto library
#include "crypto/sha256.h"
#include "crypto/ed25519.h"

// Constants
#define CONTACT_HELLO_VERSION 0x03
#define ED25519_PK_SIZE 32
#define X25519_PK_SIZE 32
#define ED25519_SIG_SIZE 64
#define HASH_SIZE 32
#define MAX_DID_LENGTH 128
#define MAX_NAME_LENGTH 64
#define DILITHIUM_PK_SIZE 1952
#define DILITHIUM_SIG_SIZE 2420
#define MAX_CONTACTS 64

// Heapless vector type for DID (max 128 bytes)
typedef struct 
{
  uint8_t data[MAX_DID_LENGTH];
  size_t len;
} HeaplessVec128;

// Heapless vector type for name (max 64 bytes)
typedef struct 
{
  uint8_t data[MAX_NAME_LENGTH];
  size_t len;
} HeaplessVec64;

// Heapless vector type for encoding (max 256 bytes)
typedef struct 
{
  uint8_t data[256];
  size_t len;
} HeaplessVec256;

// Heapless vector type for encoding (max 512 bytes)
typedef struct 
{
  uint8_t data[512];
  size_t len;
} HeaplessVec512;

// Error enum
typedef enum 
{
  CONTACT_HELLO_ERROR_NONE = 0,
  CONTACT_HELLO_ERROR_INVALID_VERSION,
  CONTACT_HELLO_ERROR_INVALID_FORMAT,
  CONTACT_HELLO_ERROR_SIGNATURE_INVALID,
  CONTACT_HELLO_ERROR_BUFFER_TOO_SMALL,
  CONTACT_HELLO_ERROR_DID_TOO_LONG,
  CONTACT_HELLO_ERROR_NAME_TOO_LONG,
} ContactHelloError;

// Trust level enum
typedef enum 
{
  TRUST_LEVEL_UNKNOWN = 0,
  TRUST_LEVEL_SEEN = 1,
  TRUST_LEVEL_VERIFIED = 2,
  TRUST_LEVEL_TRUSTED = 3,
} TrustLevel;

// ContactHello struct
typedef struct 
{
  uint8_t version;
  uint64_t timestamp;
  HeaplessVec128 did;
  uint8_t ed25519_public[ED25519_PK_SIZE];
  uint8_t x25519_public[X25519_PK_SIZE];
  HeaplessVec64 name;
  uint8_t avatar_hash[HASH_SIZE];
  uint8_t signature[ED25519_SIG_SIZE];
} ContactHello;

// Contact struct
typedef struct 
{
  ContactHello hello;
  TrustLevel trust;
  HeaplessVec64 petname;
  uint64_t last_seen;
  uint32_t message_count;
} Contact;

// Contact map entry
typedef struct 
{
  uint8_t key[8];
  Contact value;
  bool occupied;
} ContactMapEntry;

// ContactStore struct
typedef struct 
{
  ContactMapEntry entries[MAX_CONTACTS];
  size_t count;
} ContactStore;

#endif
