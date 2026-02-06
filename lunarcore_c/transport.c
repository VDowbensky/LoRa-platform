/* # Comprehensive Conversion Notes

## Overall Translation Strategy
This translation converts Rust code to C while maintaining complete functionality. Key adaptations were necessary due to fundamental language differences between Rust and C.

## Language-Specific Feature Adaptations

### 1. HeaplessVec Implementation
- **Rust Feature**: `heapless::Vec` provides a stack-allocated vector with compile-time size bounds
- **C Implementation**: Created separate struct types for each capacity (128, 214, 237, 32, 16, and UniversalAddress[16])
- **Rationale**: C doesn't support generic types, so each distinct capacity requires its own type definition
- **Implementation Details**:
  - Each HeaplessVec type contains: data array, length tracker, capacity field
  - Separate initialization, push, and extend_from_slice functions for each type
  - Return values changed from Result to bool for error handling

### 2. Option and Result Types
- **Rust Feature**: `Option<T>` and `Result<T, E>` for safe error handling
- **C Implementation**: 
  - Functions that return Option use bool return value (false = None, true = Some)
  - Output parameters used for returning values
  - Result types implemented as structs with `is_ok` flag and union containing either success or error value
- **Specific Result Types Created**:
  - `ResultMessageId`: For send_message return value
  - `ResultDeviceInfo`: For get_device_info return value
  - `ResultU32`: For ping_peer return value
  - `ResultVoid`: For connect return value

### 3. Trait Implementation (UniversalMeshTransport)
- **Rust Feature**: Traits define shared behavior across types
- **C Implementation**: Function pointer struct (vtable pattern)
- **Structure**:
  - Contains `void* instance` for storing the implementing object
  - Function pointers for each trait method
  - First parameter of each function is the instance pointer
  - Const-correctness preserved where applicable

### 4. Enums with Associated Data (ProtocolAddress)
- **Rust Feature**: Enums can contain different types of data per variant
- **C Implementation**: Tagged union pattern
- **Structure**:
  - `ProtocolAddressTag` enum for identifying the variant
  - Union containing all possible value types
  - Equality comparison function implemented separately

### 5. Hash Maps (FnvIndexMap)
- **Rust Feature**: `heapless::FnvIndexMap` provides hash map with compile-time size limit
- **C Implementation**: Open addressing hash table with linear probing
- **Implementation Details**:
  - Three separate map types for different key types (u16, u32, u8[8])
  - FNV-1a hash function implemented for consistent hashing
  - Separate entry structures with occupied flag
  - Insert, get, and remove operations for each map type
  - Handles collisions through linear probing
  - No dynamic allocation - fixed size arrays

### 6. Constants and Static Methods
- **Rust Feature**: `pub const` for public constants, associated functions with `pub fn`
- **C Implementation**: 
  - `#define` for numeric constants
  - Regular functions prefixed with module name for namespacing
  - Example: `AddressTranslator::from_public_key` becomes `address_translator_from_public_key`

### 7. String Literals and Byte Slices
- **Rust Feature**: `b"string"` for byte string literals
- **C Implementation**: Cast string literals to `const uint8_t*`
- **Note**: Used `strlen` for dynamic length where needed

### 8. Crypto Module Dependency
- **Rust Feature**: `use crate::crypto::sha256::Sha256;`
- **C Implementation**: External function declaration for SHA-256
- **Declaration**: `extern void sha256_hash(const uint8_t* data, size_t len, uint8_t out[32]);`
- **Note**: Implementation must be provided separately by including the crypto module

### 9. Method Implementations
- **Rust Feature**: `impl StructName` blocks for methods
- **C Implementation**: Functions prefixed with lowercase struct name
- **Examples**:
  - `WirePacket::encode` → `wire_packet_encode`
  - `SignalQuality::new` → `signal_quality_new`
  - `Protocol::detect` → `protocol_detect`

### 10. Clone Operations
- **Rust Feature**: `#[derive(Clone)]` and `.clone()` method
- **C Implementation**: Explicit clone functions where deep copying is needed
- **Implementation**: Manual field-by-field copying with proper handling of HeaplessVec copying

### 11. Debug Trait
- **Rust Feature**: `#[derive(Debug)]` for automatic debug formatting
- **C Implementation**: Not implemented (would require custom print functions)
- **Rationale**: Debug printing is environment-specific and not essential for functionality

### 12. Default Trait
- **Rust Feature**: `impl Default` for default initialization
- **C Implementation**: Standard initialization functions with `_init` suffix
- **Example**: `AddressLookupTable::default()` → `address_lookup_table_init()`

## Type System Differences

### Integer Types
- All Rust integer types mapped directly to C stdint.h equivalents
- `u8` → `uint8_t`
- `u16` → `uint16_t`
- `u32` → `uint32_t`
- `u64` → `uint64_t`
- `i8` → `int8_t`
- `i16` → `int16_t`
- `usize` → `size_t`

### Boolean Type
- Rust `bool` → C99 `bool` (requires `stdbool.h`)

### Array Types
- Fixed-size arrays preserved exactly
- `[u8; 32]` → `uint8_t[32]`

## Memory Management
- **No Dynamic Allocation**: All data structures use fixed-size arrays
- **Stack Allocation**: All structs designed for stack allocation
- **No Ownership System**: C doesn't have Rust's ownership rules
- **Manual Memory Management**: Caller responsible for struct lifetime
- **Const Correctness**: Maintained where data is read-only

## Error Handling Approach
- **Pattern Matching**: Replaced with switch statements
- **Unwrapping**: Replaced with explicit bool checks
- **Result Propagation**: Manual error checking with early returns
- **None Values**: Represented as NULL pointers or false bool returns

## Specific Implementation Notes

### UniversalAddress
- DID field remains as HeaplessVec128 for variable-length data
- Initialize with "did:offgrid:z" prefix as per original

### WirePacket Encoding/Decoding
- Bit manipulation preserved exactly
- Packet type encoded in upper 2 bits
- Hop count in lower 4 bits
- Big-endian byte order maintained for multi-byte fields

### SignalQuality Calculation
- Clamping logic preserved exactly
- Integer arithmetic used throughout
- Quality calculation averages normalized RSSI and SNR

### AddressTranslator Functions
- SHA-256 hashing delegated to external crypto module
- Combined hash calculation for Reticulum exactly matches original
- Default app name "yours.messaging" used in `from_public_key`

### Hash Map Collision Handling
- Linear probing ensures all slots are checked
- Prevents infinite loops with start_index tracking
- Updates existing values on duplicate key insertion

## Portability Considerations
- Standard C headers only: `stdint.h`, `stdbool.h`, `string.h`, `stdlib.h`
- No platform-specific code
- Fixed-size integers ensure consistent behavior across platforms
- External SHA-256 dependency clearly documented

## Potential Issues and Limitations

1. **Thread Safety**: No thread synchronization (none in original either)
2. **Error Granularity**: Some Result types simplified to bool
3. **Debug Output**: No debug printing implementation provided
4. **Hash Collisions**: Linear probing may degrade with high load factors
5. **Static Analysis**: C lacks Rust's compile-time guarantees
6. **Crypto Dependency**: SHA-256 implementation must be provided externally
7. **Memory Initialization**: Caller must call init functions before use

## Deviations from Original

### Deliberate Changes
1. **Trait to VTable**: Necessary adaptation for C
2. **Generic Types**: Replaced with type-specific implementations
3. **Result/Option**: Replaced with bool + output parameters or Result structs
4. **Const String**: DID string stored in HeaplessVec, not concatenated with key

### Preserved Behavior
1. **All Constants**: Exact values maintained
2. **Bit Operations**: Identical logic for encoding/decoding
3. **Hash Functions**: FNV-1a algorithm correctly implemented
4. **Address Derivation**: All calculations match exactly
5. **Size Limits**: All capacity constraints preserved

## Testing Recommendations
1. Verify SHA-256 integration with known test vectors
2. Test hash map operations with collision scenarios
3. Validate packet encoding/decoding round-trips
4. Test address translation against reference implementation
5. Verify signal quality calculations with edge cases
6. Test HeaplessVec boundary conditions

## Usage Example Pattern
```c
// Initialize a lookup table
AddressLookupTable table;
address_lookup_table_init(&table);

// Register a public key
uint8_t pubkey[32] = { /* ... */ };
address_lookup_table_register(&table, pubkey);

// Lookup by MeshCore address
uint16_t addr = 0x1234;
const uint8_t* found_key = address_lookup_table_lookup_meshcore(&table, addr);
if (found_key != NULL) {
    // Use found_key
}
```
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// Forward declarations for crypto functions
extern void sha256_hash(const uint8_t* data, size_t len, uint8_t out[32]);

// Constants
#define MAX_PACKET_SIZE 237
#define NODE_HINT_SIZE 2
#define SESSION_HINT_SIZE 4
#define AUTH_TAG_SIZE 16
#define FLAGS_SIZE 1
#define DATA_OVERHEAD (FLAGS_SIZE + NODE_HINT_SIZE + SESSION_HINT_SIZE + AUTH_TAG_SIZE)
#define DATA_MAX_PAYLOAD (MAX_PACKET_SIZE - DATA_OVERHEAD)
#define PADDED_MESSAGE_SIZE 200
#define MAX_KNOWN_ADDRESSES 64

// HeaplessVec implementation for various sizes
typedef struct {
    uint8_t data[128];
    size_t len;
    size_t capacity;
} HeaplessVec128;

typedef struct {
    uint8_t data[214];
    size_t len;
    size_t capacity;
} HeaplessVec214;

typedef struct {
    uint8_t data[237];
    size_t len;
    size_t capacity;
} HeaplessVec237;

typedef struct {
    uint8_t data[32];
    size_t len;
    size_t capacity;
} HeaplessVec32;

typedef struct {
    uint8_t data[16];
    size_t len;
    size_t capacity;
} HeaplessVec16;

// Forward declaration for UniversalAddress
typedef struct UniversalAddress UniversalAddress;

typedef struct {
    UniversalAddress data[16];
    size_t len;
    size_t capacity;
} HeaplessVecUniversalAddress16;

// HeaplessVec initialization functions
static inline void heapless_vec128_init(HeaplessVec128* vec) {
    vec->len = 0;
    vec->capacity = 128;
    memset(vec->data, 0, 128);
}

static inline void heapless_vec214_init(HeaplessVec214* vec) {
    vec->len = 0;
    vec->capacity = 214;
    memset(vec->data, 0, 214);
}

static inline void heapless_vec237_init(HeaplessVec237* vec) {
    vec->len = 0;
    vec->capacity = 237;
    memset(vec->data, 0, 237);
}

static inline void heapless_vec32_init(HeaplessVec32* vec) {
    vec->len = 0;
    vec->capacity = 32;
    memset(vec->data, 0, 32);
}

static inline void heapless_vec16_init(HeaplessVec16* vec) {
    vec->len = 0;
    vec->capacity = 16;
    memset(vec->data, 0, 16);
}

static inline void heapless_vec_universal_address16_init(HeaplessVecUniversalAddress16* vec) {
    vec->len = 0;
    vec->capacity = 16;
    memset(vec->data, 0, sizeof(UniversalAddress) * 16);
}

// HeaplessVec push functions
static inline bool heapless_vec128_push(HeaplessVec128* vec, uint8_t val) {
    if (vec->len >= vec->capacity) {
        return false;
    }
    vec->data[vec->len++] = val;
    return true;
}

static inline bool heapless_vec237_push(HeaplessVec237* vec, uint8_t val) {
    if (vec->len >= vec->capacity) {
        return false;
    }
    vec->data[vec->len++] = val;
    return true;
}

// HeaplessVec extend_from_slice functions
static inline bool heapless_vec128_extend_from_slice(HeaplessVec128* vec, const uint8_t* data, size_t len) {
    if (vec->len + len > vec->capacity) {
        return false;
    }
    memcpy(&vec->data[vec->len], data, len);
    vec->len += len;
    return true;
}

static inline bool heapless_vec214_extend_from_slice(HeaplessVec214* vec, const uint8_t* data, size_t len) {
    if (vec->len + len > vec->capacity) {
        return false;
    }
    memcpy(&vec->data[vec->len], data, len);
    vec->len += len;
    return true;
}

static inline bool heapless_vec237_extend_from_slice(HeaplessVec237* vec, const uint8_t* data, size_t len) {
    if (vec->len + len > vec->capacity) {
        return false;
    }
    memcpy(&vec->data[vec->len], data, len);
    vec->len += len;
    return true;
}

// UniversalAddress structure
struct UniversalAddress {
    HeaplessVec128 did;
    uint8_t public_key[32];
    uint16_t meshcore_addr;
    uint32_t meshtastic_id;
    uint8_t reticulum_hash[16];
};

// UniversalAddress initialization
static inline void universal_address_init(UniversalAddress* addr) {
    heapless_vec128_init(&addr->did);
    memset(addr->public_key, 0, 32);
    addr->meshcore_addr = 0;
    addr->meshtastic_id = 0;
    memset(addr->reticulum_hash, 0, 16);
}

// UniversalAddress clone function
static inline UniversalAddress universal_address_clone(const UniversalAddress* addr) {
    UniversalAddress result;
    result.did = addr->did;
    memcpy(result.public_key, addr->public_key, 32);
    result.meshcore_addr = addr->meshcore_addr;
    result.meshtastic_id = addr->meshtastic_id;
    memcpy(result.reticulum_hash, addr->reticulum_hash, 16);
    return result;
}

// AddressTranslator functions
UniversalAddress address_translator_from_public_key(const uint8_t public_key[32]) {
    uint8_t pubkey_hash[32];
    sha256_hash(public_key, 32, pubkey_hash);

    uint16_t meshcore_addr = ((uint16_t)pubkey_hash[0] << 8) | (uint16_t)pubkey_hash[1];

    uint32_t meshtastic_id = ((uint32_t)pubkey_hash[0] << 24)
        | ((uint32_t)pubkey_hash[1] << 16)
        | ((uint32_t)pubkey_hash[2] << 8)
        | (uint32_t)pubkey_hash[3];

    const uint8_t app_name[] = "yours.messaging";
    uint8_t app_hash[32];
    sha256_hash(app_name, strlen((const char*)app_name), app_hash);
    
    uint8_t combined[64];
    memcpy(combined, app_hash, 32);
    memcpy(combined + 32, public_key, 32);
    
    uint8_t reticulum_full[32];
    sha256_hash(combined, 64, reticulum_full);
    
    uint8_t reticulum_hash[16];
    memcpy(reticulum_hash, reticulum_full, 16);

    UniversalAddress addr;
    heapless_vec128_init(&addr.did);
    heapless_vec128_extend_from_slice(&addr.did, (const uint8_t*)"did:offgrid:z", 13);
    memcpy(addr.public_key, public_key, 32);
    addr.meshcore_addr = meshcore_addr;
    addr.meshtastic_id = meshtastic_id;
    memcpy(addr.reticulum_hash, reticulum_hash, 16);

    return addr;
}

uint16_t address_translator_derive_meshcore_address(const uint8_t public_key[32]) {
    uint8_t hash[32];
    sha256_hash(public_key, 32, hash);
    return ((uint16_t)hash[0] << 8) | (uint16_t)hash[1];
}

uint32_t address_translator_derive_meshtastic_id(const uint8_t public_key[32]) {
    uint8_t hash[32];
    sha256_hash(public_key, 32, hash);
    return ((uint32_t)hash[0] << 24)
        | ((uint32_t)hash[1] << 16)
        | ((uint32_t)hash[2] << 8)
        | (uint32_t)hash[3];
}

void address_translator_derive_reticulum_hash(const uint8_t public_key[32], 
                                              const uint8_t* app_name, 
                                              size_t app_name_len,
                                              uint8_t result[16]) {
    uint8_t app_hash[32];
    sha256_hash(app_name, app_name_len, app_hash);
    
    uint8_t combined[64];
    memcpy(combined, app_hash, 32);
    memcpy(combined + 32, public_key, 32);
    
    uint8_t full_hash[32];
    sha256_hash(combined, 64, full_hash);
    
    memcpy(result, full_hash, 16);
}

// PacketType enum
typedef enum {
    PACKET_TYPE_DATA = 0b00,
    PACKET_TYPE_HANDSHAKE = 0b01,
    PACKET_TYPE_CONTROL = 0b10,
    PACKET_TYPE_COVER = 0b11
} PacketType;

PacketType packet_type_from_flags(uint8_t flags) {
    uint8_t type_bits = (flags >> 6) & 0b11;
    switch (type_bits) {
        case 0b00: return PACKET_TYPE_DATA;
        case 0b01: return PACKET_TYPE_HANDSHAKE;
        case 0b10: return PACKET_TYPE_CONTROL;
        case 0b11: return PACKET_TYPE_COVER;
        default: return PACKET_TYPE_DATA; // unreachable in practice
    }
}

// WirePacket structure
typedef struct {
    PacketType packet_type;
    uint8_t hop_count;
    uint16_t next_hop_hint;
    uint32_t session_hint;
    HeaplessVec214 payload;
} WirePacket;

// WirePacket initialization
static inline void wire_packet_init(WirePacket* packet) {
    packet->packet_type = PACKET_TYPE_DATA;
    packet->hop_count = 0;
    packet->next_hop_hint = 0;
    packet->session_hint = 0;
    heapless_vec214_init(&packet->payload);
}

// WirePacket clone function
static inline WirePacket wire_packet_clone(const WirePacket* packet) {
    WirePacket result;
    result.packet_type = packet->packet_type;
    result.hop_count = packet->hop_count;
    result.next_hop_hint = packet->next_hop_hint;
    result.session_hint = packet->session_hint;
    result.payload = packet->payload;
    return result;
}

// WirePacket constructor for data packets
bool wire_packet_new_data(WirePacket* packet, uint16_t next_hop, uint32_t session, const uint8_t* payload, size_t payload_len) {
    if (payload_len > DATA_MAX_PAYLOAD) {
        return false;
    }
    
    wire_packet_init(packet);
    packet->packet_type = PACKET_TYPE_DATA;
    packet->hop_count = 0;
    packet->next_hop_hint = next_hop;
    packet->session_hint = session;
    
    if (!heapless_vec214_extend_from_slice(&packet->payload, payload, payload_len)) {
        return false;
    }
    
    return true;
}

// WirePacket encode
HeaplessVec237 wire_packet_encode(const WirePacket* packet) {
    HeaplessVec237 buf;
    heapless_vec237_init(&buf);

    uint8_t flags = ((uint8_t)packet->packet_type << 6) | (packet->hop_count & 0x0F);
    heapless_vec237_push(&buf, flags);

    heapless_vec237_push(&buf, (uint8_t)(packet->next_hop_hint >> 8));
    heapless_vec237_push(&buf, (uint8_t)packet->next_hop_hint);

    heapless_vec237_push(&buf, (uint8_t)(packet->session_hint >> 24));
    heapless_vec237_push(&buf, (uint8_t)(packet->session_hint >> 16));
    heapless_vec237_push(&buf, (uint8_t)(packet->session_hint >> 8));
    heapless_vec237_push(&buf, (uint8_t)packet->session_hint);

    heapless_vec237_extend_from_slice(&buf, packet->payload.data, packet->payload.len);

    return buf;
}

// WirePacket decode
bool wire_packet_decode(WirePacket* packet, const uint8_t* data, size_t data_len) {
    if (data_len < 7) {
        return false;
    }

    uint8_t flags = data[0];
    packet->packet_type = packet_type_from_flags(flags);
    packet->hop_count = flags & 0x0F;

    packet->next_hop_hint = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
    packet->session_hint = ((uint32_t)data[3] << 24)
        | ((uint32_t)data[4] << 16)
        | ((uint32_t)data[5] << 8)
        | (uint32_t)data[6];

    heapless_vec214_init(&packet->payload);
    if (data_len > 7) {
        if (!heapless_vec214_extend_from_slice(&packet->payload, &data[7], data_len - 7)) {
            return false;
        }
    }

    return true;
}

// WirePacket increment_hop
bool wire_packet_increment_hop(WirePacket* packet) {
    if (packet->hop_count < 15) {
        packet->hop_count += 1;
        return true;
    } else {
        return false;
    }
}

// MessagePriority enum
typedef enum {
    MESSAGE_PRIORITY_LOW = 0,
    MESSAGE_PRIORITY_NORMAL = 1,
    MESSAGE_PRIORITY_HIGH = 2,
    MESSAGE_PRIORITY_CRITICAL = 3
} MessagePriority;

// UniversalMessage structure
typedef struct {
    uint8_t id[8];
    UniversalAddress recipient;
    HeaplessVec237 payload;
    MessagePriority priority;
    uint64_t timestamp;
} UniversalMessage;

// UniversalMessage initialization
static inline void universal_message_init(UniversalMessage* msg) {
    memset(msg->id, 0, 8);
    universal_address_init(&msg->recipient);
    heapless_vec237_init(&msg->payload);
    msg->priority = MESSAGE_PRIORITY_NORMAL;
    msg->timestamp = 0;
}

// UniversalMessage clone function
static inline UniversalMessage universal_message_clone(const UniversalMessage* msg) {
    UniversalMessage result;
    memcpy(result.id, msg->id, 8);
    result.recipient = universal_address_clone(&msg->recipient);
    result.payload = msg->payload;
    result.priority = msg->priority;
    result.timestamp = msg->timestamp;
    return result;
}

// ConnectionState enum
typedef enum {
    CONNECTION_STATE_DISCONNECTED,
    CONNECTION_STATE_CONNECTING,
    CONNECTION_STATE_CONNECTED,
    CONNECTION_STATE_ERROR
} ConnectionState;

// SignalQuality structure
typedef struct {
    int16_t rssi;
    int8_t snr;
    uint8_t quality;
} SignalQuality;

// SignalQuality constructor
SignalQuality signal_quality_new(int16_t rssi, int8_t snr) {
    // Clamp RSSI to [-120, -50] and normalize to [0, 100]
    int16_t clamped_rssi = rssi;
    if (clamped_rssi < -120) clamped_rssi = -120;
    if (clamped_rssi > -50) clamped_rssi = -50;
    uint8_t rssi_norm = (uint8_t)(((uint16_t)(clamped_rssi + 120) * 100) / 70);

    // Clamp SNR to [-20, 10] and normalize to [0, 100]
    int8_t clamped_snr = snr;
    if (clamped_snr < -20) clamped_snr = -20;
    if (clamped_snr > 10) clamped_snr = 10;
    uint8_t snr_norm = (uint8_t)(((uint16_t)(clamped_snr + 20) * 100) / 30);

    uint8_t quality = (rssi_norm + snr_norm) / 2;

    SignalQuality sq;
    sq.rssi = rssi;
    sq.snr = snr;
    sq.quality = quality;
    return sq;
}

// ProtocolSupport structure
typedef struct {
    bool meshcore;
    bool meshtastic;
    bool reticulum;
} ProtocolSupport;

// ProtocolSupport initialization
static inline void protocol_support_init(ProtocolSupport* ps) {
    ps->meshcore = false;
    ps->meshtastic = false;
    ps->reticulum = false;
}

// MeshDeviceInfo structure
typedef struct {
    HeaplessVec32 name;
    HeaplessVec16 firmware_version;
    HeaplessVec32 hardware_model;
    ProtocolSupport protocols;
    uint8_t battery_level;
} MeshDeviceInfo;

// MeshDeviceInfo initialization
static inline void mesh_device_info_init(MeshDeviceInfo* info) {
    heapless_vec32_init(&info->name);
    heapless_vec16_init(&info->firmware_version);
    heapless_vec32_init(&info->hardware_model);
    protocol_support_init(&info->protocols);
    info->battery_level = 0;
}

// MeshDeviceInfo clone function
static inline MeshDeviceInfo mesh_device_info_clone(const MeshDeviceInfo* info) {
    MeshDeviceInfo result;
    result.name = info->name;
    result.firmware_version = info->firmware_version;
    result.hardware_model = info->hardware_model;
    result.protocols = info->protocols;
    result.battery_level = info->battery_level;
    return result;
}

// TransportError enum
typedef enum {
    TRANSPORT_ERROR_NOT_CONNECTED,
    TRANSPORT_ERROR_CONNECTION_FAILED,
    TRANSPORT_ERROR_SEND_FAILED,
    TRANSPORT_ERROR_RECEIVE_TIMEOUT,
    TRANSPORT_ERROR_INVALID_MESSAGE,
    TRANSPORT_ERROR_BUFFER_OVERFLOW,
    TRANSPORT_ERROR_DEVICE_BUSY,
    TRANSPORT_ERROR_PROTOCOL_ERROR,
    TRANSPORT_ERROR_UNKNOWN
} TransportError;

// Result types for functions
typedef struct {
    bool is_ok;
    union {
        uint8_t ok_value[8];
        TransportError err_value;
    } value;
} ResultMessageId;

typedef struct {
    bool is_ok;
    union {
        MeshDeviceInfo ok_value;
        TransportError err_value;
    } value;
} ResultDeviceInfo;

typedef struct {
    bool is_ok;
    union {
        uint32_t ok_value;
        TransportError err_value;
    } value;
} ResultU32;

typedef struct {
    bool is_ok;
    TransportError err_value;
} ResultVoid;

// UniversalMeshTransport trait as function pointer struct (vtable)
typedef struct UniversalMeshTransport UniversalMeshTransport;

struct UniversalMeshTransport {
    void* instance;
    ResultVoid (*connect)(void* instance);
    void (*disconnect)(void* instance);
    ResultMessageId (*send_message)(void* instance, const UniversalMessage* message);
    bool (*poll_message)(void* instance, UniversalMessage* out_message);
    ResultDeviceInfo (*get_device_info)(const void* instance);
    ConnectionState (*connection_state)(const void* instance);
    SignalQuality (*signal_quality)(const void* instance);
    HeaplessVecUniversalAddress16 (*discover_peers)(void* instance, uint32_t timeout_ms);
    ResultU32 (*ping_peer)(void* instance, const UniversalAddress* address);
};

// Protocol enum
typedef enum {
    PROTOCOL_MESHCORE,
    PROTOCOL_MESHTASTIC,
    PROTOCOL_RETICULUM
} Protocol;

// Protocol magic_bytes
void protocol_magic_bytes(Protocol protocol, uint8_t out[2]) {
    switch (protocol) {
        case PROTOCOL_MESHCORE:
            out[0] = 0xAA;
            out[1] = 0x55;
            break;
        case PROTOCOL_MESHTASTIC:
            out[0] = 0x94;
            out[1] = 0xC3;
            break;
        case PROTOCOL_RETICULUM:
            out[0] = 0xC0;
            out[1] = 0x00;
            break;
    }
}

// Protocol detect
bool protocol_detect(const uint8_t* data, size_t data_len, Protocol* out_protocol) {
    if (data_len < 2) {
        return false;
    }
    
    if (data[0] == 0xAA && data[1] == 0x55) {
        *out_protocol = PROTOCOL_MESHCORE;
        return true;
    } else if (data[0] == 0x94 && data[1] == 0xC3) {
        *out_protocol = PROTOCOL_MESHTASTIC;
        return true;
    } else if (data[0] == 0xC0) {
        *out_protocol = PROTOCOL_RETICULUM;
        return true;
    } else if (data[0] == 'A' && data[1] == 'T') {
        return false;
    } else {
        return false;
    }
}

// ProtocolAddress enum (tagged union)
typedef enum {
    PROTOCOL_ADDRESS_MESHCORE,
    PROTOCOL_ADDRESS_MESHTASTIC,
    PROTOCOL_ADDRESS_RETICULUM
} ProtocolAddressTag;

typedef struct {
    ProtocolAddressTag tag;
    union {
        uint16_t meshcore;
        uint32_t meshtastic;
        uint8_t reticulum[16];
    } value;
} ProtocolAddress;

// ProtocolAddress equality comparison
bool protocol_address_eq(const ProtocolAddress* a, const ProtocolAddress* b) {
    if (a->tag != b->tag) {
        return false;
    }
    switch (a->tag) {
        case PROTOCOL_ADDRESS_MESHCORE:
            return a->value.meshcore == b->value.meshcore;
        case PROTOCOL_ADDRESS_MESHTASTIC:
            return a->value.meshtastic == b->value.meshtastic;
        case PROTOCOL_ADDRESS_RETICULUM:
            return memcmp(a->value.reticulum, b->value.reticulum, 16) == 0;
        default:
            return false;
    }
}

// Simple FNV-1a hash function for use in hash maps
static inline uint32_t fnv1a_hash_u16(uint16_t key) {
    uint32_t hash = 2166136261u;
    hash ^= (uint8_t)(key >> 8);
    hash *= 16777619u;
    hash ^= (uint8_t)key;
    hash *= 16777619u;
    return hash;
}

static inline uint32_t fnv1a_hash_u32(uint32_t key) {
    uint32_t hash = 2166136261u;
    hash ^= (uint8_t)(key >> 24);
    hash *= 16777619u;
    hash ^= (uint8_t)(key >> 16);
    hash *= 16777619u;
    hash ^= (uint8_t)(key >> 8);
    hash *= 16777619u;
    hash ^= (uint8_t)key;
    hash *= 16777619u;
    return hash;
}

static inline uint32_t fnv1a_hash_bytes(const uint8_t* data, size_t len) {
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash *= 16777619u;
    }
    return hash;
}

// Hash map entry structures
typedef struct {
    bool occupied;
    uint16_t key;
    uint8_t value[32];
} MeshCoreMapEntry;

typedef struct {
    bool occupied;
    uint32_t key;
    uint8_t value[32];
} MeshtasticMapEntry;

typedef struct {
    bool occupied;
    uint8_t key[8];
    uint8_t value[32];
} ReticulumMapEntry;

// Hash map structures (using open addressing with linear probing)
typedef struct {
    MeshCoreMapEntry entries[MAX_KNOWN_ADDRESSES];
    size_t count;
} MeshCoreIndexMap;

typedef struct {
    MeshtasticMapEntry entries[MAX_KNOWN_ADDRESSES];
    size_t count;
} MeshtasticIndexMap;

typedef struct {
    ReticulumMapEntry entries[MAX_KNOWN_ADDRESSES];
    size_t count;
} ReticulumIndexMap;

// MeshCoreIndexMap functions
static inline void meshcore_index_map_init(MeshCoreIndexMap* map) {
    memset(map->entries, 0, sizeof(map->entries));
    map->count = 0;
}

bool meshcore_index_map_insert(MeshCoreIndexMap* map, uint16_t key, const uint8_t value[32]) {
    if (map->count >= MAX_KNOWN_ADDRESSES) {
        return false;
    }
    
    uint32_t hash = fnv1a_hash_u16(key);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            map->entries[index].occupied = true;
            map->entries[index].key = key;
            memcpy(map->entries[index].value, value, 32);
            map->count++;
            return true;
        } else if (map->entries[index].key == key) {
            // Key already exists, update value
            memcpy(map->entries[index].value, value, 32);
            return true;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return false;
}

const uint8_t* meshcore_index_map_get(const MeshCoreIndexMap* map, uint16_t key) {
    uint32_t hash = fnv1a_hash_u16(key);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            return NULL;
        }
        if (map->entries[index].key == key) {
            return map->entries[index].value;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return NULL;
}

bool meshcore_index_map_remove(MeshCoreIndexMap* map, uint16_t key) {
    uint32_t hash = fnv1a_hash_u16(key);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            return false;
        }
        if (map->entries[index].key == key) {
            map->entries[index].occupied = false;
            map->count--;
            return true;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return false;
}

// MeshtasticIndexMap functions
static inline void meshtastic_index_map_init(MeshtasticIndexMap* map) {
    memset(map->entries, 0, sizeof(map->entries));
    map->count = 0;
}

bool meshtastic_index_map_insert(MeshtasticIndexMap* map, uint32_t key, const uint8_t value[32]) {
    if (map->count >= MAX_KNOWN_ADDRESSES) {
        return false;
    }
    
    uint32_t hash = fnv1a_hash_u32(key);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            map->entries[index].occupied = true;
            map->entries[index].key = key;
            memcpy(map->entries[index].value, value, 32);
            map->count++;
            return true;
        } else if (map->entries[index].key == key) {
            // Key already exists, update value
            memcpy(map->entries[index].value, value, 32);
            return true;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return false;
}

const uint8_t* meshtastic_index_map_get(const MeshtasticIndexMap* map, uint32_t key) {
    uint32_t hash = fnv1a_hash_u32(key);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            return NULL;
        }
        if (map->entries[index].key == key) {
            return map->entries[index].value;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return NULL;
}

bool meshtastic_index_map_remove(MeshtasticIndexMap* map, uint32_t key) {
    uint32_t hash = fnv1a_hash_u32(key);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            return false;
        }
        if (map->entries[index].key == key) {
            map->entries[index].occupied = false;
            map->count--;
            return true;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return false;
}

// ReticulumIndexMap functions
static inline void reticulum_index_map_init(ReticulumIndexMap* map) {
    memset(map->entries, 0, sizeof(map->entries));
    map->count = 0;
}

bool reticulum_index_map_insert(ReticulumIndexMap* map, const uint8_t key[8], const uint8_t value[32]) {
    if (map->count >= MAX_KNOWN_ADDRESSES) {
        return false;
    }
    
    uint32_t hash = fnv1a_hash_bytes(key, 8);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            map->entries[index].occupied = true;
            memcpy(map->entries[index].key, key, 8);
            memcpy(map->entries[index].value, value, 32);
            map->count++;
            return true;
        } else if (memcmp(map->entries[index].key, key, 8) == 0) {
            // Key already exists, update value
            memcpy(map->entries[index].value, value, 32);
            return true;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return false;
}

const uint8_t* reticulum_index_map_get(const ReticulumIndexMap* map, const uint8_t key[8]) {
    uint32_t hash = fnv1a_hash_bytes(key, 8);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            return NULL;
        }
        if (memcmp(map->entries[index].key, key, 8) == 0) {
            return map->entries[index].value;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return NULL;
}

bool reticulum_index_map_remove(ReticulumIndexMap* map, const uint8_t key[8]) {
    uint32_t hash = fnv1a_hash_bytes(key, 8);
    size_t index = hash % MAX_KNOWN_ADDRESSES;
    size_t start_index = index;
    
    do {
        if (!map->entries[index].occupied) {
            return false;
        }
        if (memcmp(map->entries[index].key, key, 8) == 0) {
            map->entries[index].occupied = false;
            map->count--;
            return true;
        }
        
        index = (index + 1) % MAX_KNOWN_ADDRESSES;
    } while (index != start_index);
    
    return false;
}

// AddressLookupTable structure
typedef struct {
    MeshCoreIndexMap meshcore_index;
    MeshtasticIndexMap meshtastic_index;
    ReticulumIndexMap reticulum_index;
} AddressLookupTable;

// AddressLookupTable initialization
void address_lookup_table_init(AddressLookupTable* table) {
    meshcore_index_map_init(&table->meshcore_index);
    meshtastic_index_map_init(&table->meshtastic_index);
    reticulum_index_map_init(&table->reticulum_index);
}

// AddressLookupTable register
void address_lookup_table_register(AddressLookupTable* table, const uint8_t public_key[32]) {
    UniversalAddress addr = address_translator_from_public_key(public_key);

    meshcore_index_map_insert(&table->meshcore_index, addr.meshcore_addr, public_key);

    meshtastic_index_map_insert(&table->meshtastic_index, addr.meshtastic_id, public_key);

    uint8_t ret_key[8];
    memcpy(ret_key, addr.reticulum_hash, 8);
    reticulum_index_map_insert(&table->reticulum_index, ret_key, public_key);
}

// AddressLookupTable unregister
void address_lookup_table_unregister(AddressLookupTable* table, const uint8_t public_key[32]) {
    UniversalAddress addr = address_translator_from_public_key(public_key);
    meshcore_index_map_remove(&table->meshcore_index, addr.meshcore_addr);
    meshtastic_index_map_remove(&table->meshtastic_index, addr.meshtastic_id);
    uint8_t ret_key[8];
    memcpy(ret_key, addr.reticulum_hash, 8);
    reticulum_index_map_remove(&table->reticulum_index, ret_key);
}

// AddressLookupTable lookup_meshcore
const uint8_t* address_lookup_table_lookup_meshcore(const AddressLookupTable* table, uint16_t addr) {
    return meshcore_index_map_get(&table->meshcore_index, addr);
}

// AddressLookupTable lookup_meshtastic
const uint8_t* address_lookup_table_lookup_meshtastic(const AddressLookupTable* table, uint32_t id) {
    return meshtastic_index_map_get(&table->meshtastic_index, id);
}

// AddressLookupTable lookup_reticulum
const uint8_t* address_lookup_table_lookup_reticulum(const AddressLookupTable* table, const uint8_t hash[16]) {
    uint8_t key[8];
    memcpy(key, hash, 8);
    return reticulum_index_map_get(&table->reticulum_index, key);
}

// AddressLookupTable lookup (dispatches to specific lookup based on address type)
const uint8_t* address_lookup_table_lookup(const AddressLookupTable* table, const ProtocolAddress* addr) {
    switch (addr->tag) {
        case PROTOCOL_ADDRESS_MESHCORE:
            return address_lookup_table_lookup_meshcore(table, addr->value.meshcore);
        case PROTOCOL_ADDRESS_MESHTASTIC:
            return address_lookup_table_lookup_meshtastic(table, addr->value.meshtastic);
        case PROTOCOL_ADDRESS_RETICULUM:
            return address_lookup_table_lookup_reticulum(table, addr->value.reticulum);
        default:
            return NULL;
    }
}

// AddressLookupTable len
size_t address_lookup_table_len(const AddressLookupTable* table) {
    return table->meshcore_index.count;
}

// AddressLookupTable is_empty
bool address_lookup_table_is_empty(const AddressLookupTable* table) {
    return table->meshcore_index.count == 0;
}
