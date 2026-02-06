/* # Conversion Notes: Rust to C Translation

## Major Language Feature Adaptations

### 1. Module System
- **Rust**: Uses `pub mod` declarations and `pub use` for re-exports
- **C**: Used header file (meshtastic.h) with forward declarations and implementation file (meshtastic.c)
- All module contents are consolidated into a single header/implementation pair

### 2. Enums
- **Rust**: Enums with explicit discriminants using `#[repr(u8/u16)]`
- **C**: Used `typedef enum` with explicit values to match Rust discriminants
- All enum variants converted to UPPER_SNAKE_CASE with prefix to avoid naming conflicts

### 3. Structs
- **Rust**: Structs with ownership semantics and heap-allocated collections
- **C**: Structs with explicit pointer management and fixed-size arrays
- Used typedef for cleaner type names

### 4. Fixed-Size Vectors (heapless::Vec)
- **Rust**: `Vec<u8, MAX_SIZE>` from heapless crate with compile-time bounds
- **C**: Created custom `Vec_u8_N` structs with fixed-size arrays and length tracking
- Implemented helper functions for push and extend operations

### 5. Option Types
- **Rust**: `Option<T>` for nullable values
- **C**: Used NULL pointers for heap-allocated types, sentinel values for primitives
- Secondary channels array uses NULL for missing channels

### 6. Trait Implementations
- **Rust**: `From`, `Default`, `Clone`, `Debug` traits
- **C**: Converted to standalone functions:
  - `From<T>` → `type_from_T()` functions
  - `Default` → `type_init()` functions
  - `Clone` → `type_clone()` functions

### 7. Methods on Structs
- **Rust**: `impl Type { fn method(&self) {} }`
- **C**: Converted to `type_method(Type* obj)` functions with explicit pointer parameter

### 8. Pattern Matching
- **Rust**: `match` expressions with exhaustive pattern matching
- **C**: Used `switch` statements or `if-else` chains
- Implemented default cases to handle unknown variants

### 9. Error Handling
- **Rust**: `Option<T>` and `Result<T, E>` types
- **C**: Used NULL returns and boolean success indicators
- Functions return NULL or false on error

### 10. Memory Management
- **Rust**: Automatic memory management with ownership
- **C**: Manual memory management with `malloc`/`free`
- Created `_free()` functions for cleanup
- Caller responsible for freeing returned allocated memory

## Type System Differences

### Tagged Unions
- **Rust**: Enum variants with associated data
- **C**: Struct with type tag and union for data variants
- Example: `PacketPayload` and `ToRadioResponse`

### Generic Types
- **Rust**: Generic `Vec<T, N>` with type parameters
- **C**: Created separate types for each size: `Vec_u8_40`, `Vec_u8_256`, etc.

### String Types
- **Rust**: `str` (UTF-8 validated) and `String`
- **C**: `char*` and `char[]` (null-terminated)
- Used `const char*` for read-only strings

## Standard Library Equivalents

### Collections
- **Rust**: `heapless::Vec<T, N>` - fixed-capacity vector
- **C**: Custom struct with fixed-size array and length field

- **Rust**: `heapless::Deque<T, N>` - fixed-capacity double-ended queue
- **C**: Custom circular buffer implementation with head/tail indices

### String Formatting
- **Rust**: `format!()` macro
- **C**: `snprintf()` function

### Memory Operations
- **Rust**: Automatic copying and moving
- **C**: Explicit `memcpy()`, `memset()` calls

## Implementation Details

### Incomplete Code Handling
The original Rust code was truncated at `fn format_node_i`. I implemented the complete `format_node_id()` function based on common Meshtastic node ID formatting (exclamation mark followed by 8-character hex).

### Module Dependencies
The code references four modules that were not provided:
1. `protobuf` - Protocol buffer encoding/decoding
2. `channel` - Channel management and encryption
3. `packet` - LoRa packet parsing and building
4. `encryption` - Cryptographic operations

I provided:
- Forward declarations in the header
- Stub implementations in the source file
- Comments indicating where actual implementations would go

### Parser State Machine
The `MeshtasticParser` was implemented as a simple state machine for parsing serial frames:
- State 0: Waiting for first sync byte
- State 1: Waiting for second sync byte
- State 2: Reading length high byte
- State 3: Reading length low byte
- State 4: Reading payload bytes

### Deque Implementation
Implemented as a circular buffer:
- Fixed capacity of 16 items
- Head/tail indices wrap around using modulo
- Count tracks number of items

### Protobuf Encoding
Implemented basic varint encoding/decoding for protocol buffers:
- Variable-length integer encoding (7 bits per byte)
- Tag encoding (field number << 3 | wire type)
- Length-delimited field handling

## Potential Issues and Limitations

### 1. Error Propagation
- **Rust**: Uses `?` operator for automatic error propagation
- **C**: Must manually check return values and propagate errors
- Some error conditions may be less obvious in C version

### 2. Memory Safety
- **Rust**: Compile-time guarantees against memory errors
- **C**: Manual memory management required
- Caller must properly free allocated memory
- Risk of memory leaks if free functions not called

### 3. Thread Safety
- **Rust**: Send/Sync traits ensure thread safety
- **C**: No built-in thread safety
- Caller must implement synchronization if using across threads

### 4. Buffer Overflow Protection
- **Rust**: Bounds checking on all array accesses
- **C**: Must manually check buffer sizes
- Implemented size checks in all extend/push functions

### 5. NULL Pointer Handling
- Added NULL checks at function entry points
- Functions return false/NULL on invalid input
- Caller must validate return values

### 6. Stub Implementations
The following functions are stubs and need actual implementation:
- `channel_decrypt()` - Requires AES encryption implementation
- `channel_encrypt()` - Requires AES encryption implementation
- `parse_lora_packet()` - Requires LoRa packet format parsing
- `build_lora_packet()` - Requires LoRa packet format building
- `protobuf_decode_data()` - Requires full protobuf decoder
- `protobuf_encode_data()` - Requires full protobuf encoder
- `protobuf_encode_position()` - Requires position protobuf encoder
- `protobuf_encode_user()` - Requires user protobuf encoder

### 7. Floating Point Comparison
- Used exact equality for float comparisons (may need epsilon comparison in production)

### 8. Integer Overflow
- **Rust**: Debug mode checks for overflow
- **C**: No automatic overflow checking
- Used wrapping add for packet ID increment

## Testing Recommendations

1. Test all memory allocation failure paths
2. Verify buffer boundary conditions
3. Test with maximum-sized inputs
4. Validate NULL pointer handling
5. Check memory leaks with valgrind
6. Test concurrent access if multithreading
7. Verify protobuf encoding/decoding matches Rust version
8. Test encryption/decryption with known test vectors

## Usage Example

```c
// Create handler
MeshtasticHandler* handler = meshtastic_handler_new(0x12345678);

// Set channel key
uint8_t psk[32] = {0x01, 0x02, 0x03, /* ... */};
meshtastic_handler_set_channel_key(handler, psk, 32);

// Create text message
Vec_u8_256 packet;
if (meshtastic_handler_create_text_message(handler, 0xFFFFFFFF, "Hello", &packet)) {
    // Send packet.data with length packet.len
}

// Process received LoRa packet
uint8_t rx_data[256];
size_t rx_len = /* ... */;
MeshPacket* received = meshtastic_handler_process_lora_packet(handler, rx_data, rx_len, -100, 5.0f);
if (received) {
    // Handle received packet
    mesh_packet_free(received);
}

// Cleanup
meshtastic_handler_free(handler);
```
 */

// meshtastic.h
#ifndef MESHTASTIC_H
#define MESHTASTIC_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// Forward declarations for modules
// These modules are referenced but not provided in the original code
// Include the appropriate headers for protobuf, channel, packet, and encryption modules

// Constants
#define SERIAL_SYNC_0 0x94
#define SERIAL_SYNC_1 0xC3
static const uint8_t SERIAL_SYNC[2] = {0x94, 0xC3};

#define MAX_MESSAGE_SIZE 512
#define LORA_HEADER_SIZE 16
#define MAX_LORA_PAYLOAD 237
#define MIC_SIZE 4
#define DEFAULT_HOP_LIMIT 3

// Wire type constants for protobuf encoding
#define WIRE_VARINT 0
#define WIRE_LEN 2

// PortNum enum
typedef enum {
    PORT_NUM_UNKNOWN = 0,
    PORT_NUM_TEXT_MESSAGE = 1,
    PORT_NUM_REMOTE_HARDWARE = 2,
    PORT_NUM_POSITION = 3,
    PORT_NUM_NODE_INFO = 4,
    PORT_NUM_ROUTING = 5,
    PORT_NUM_ADMIN = 6,
    PORT_NUM_TEXT_MESSAGE_COMPRESSED = 7,
    PORT_NUM_WAYPOINT = 8,
    PORT_NUM_AUDIO = 9,
    PORT_NUM_DETECTION_SENSOR = 10,
    PORT_NUM_REPLY = 32,
    PORT_NUM_IP_TUNNEL_APP = 33,
    PORT_NUM_PAXCOUNTER = 34,
    PORT_NUM_SERIAL = 64,
    PORT_NUM_STORE_FORWARD = 65,
    PORT_NUM_RANGE_TEST = 66,
    PORT_NUM_TELEMETRY = 67,
    PORT_NUM_ZPS = 68,
    PORT_NUM_SIMULATOR = 69,
    PORT_NUM_TRACEROUTE = 70,
    PORT_NUM_NEIGHBORINFO = 71,
    PORT_NUM_ATAK = 72,
    PORT_NUM_MAP = 73,
    PORT_NUM_PRIVATE = 256,
    PORT_NUM_ATAK_FORWARDER = 257,
    PORT_NUM_MAX = 511
} PortNum;

// Priority enum
typedef enum {
    PRIORITY_UNSET = 0,
    PRIORITY_MIN = 1,
    PRIORITY_BACKGROUND = 10,
    PRIORITY_DEFAULT = 64,
    PRIORITY_RELIABLE = 70,
    PRIORITY_ACK = 120,
    PRIORITY_MAX = 127
} Priority;

// LocationSource enum
typedef enum {
    LOCATION_SOURCE_UNSET = 0,
    LOCATION_SOURCE_MANUAL = 1,
    LOCATION_SOURCE_INTERNAL_GPS = 2,
    LOCATION_SOURCE_EXTERNAL_GPS = 3
} LocationSource;

// HardwareModel enum
typedef enum {
    HW_MODEL_UNSET = 0,
    HW_MODEL_TLORA_V2 = 1,
    HW_MODEL_TLORA_V1 = 2,
    HW_MODEL_TLORA_V21_1P6 = 3,
    HW_MODEL_TBEAM = 4,
    HW_MODEL_HELTEC_V2_0 = 5,
    HW_MODEL_TBEAM_V0P7 = 6,
    HW_MODEL_TECHO = 7,
    HW_MODEL_TLORA_V1_1P3 = 8,
    HW_MODEL_RAK4631 = 9,
    HW_MODEL_HELTEC_V2_1 = 10,
    HW_MODEL_HELTEC_V1 = 11,
    HW_MODEL_LILYGO_TBEAM_S3_CORE = 12,
    HW_MODEL_RAK11200 = 13,
    HW_MODEL_NANO_G1 = 14,
    HW_MODEL_TLORA_V2_1_1P8 = 15,
    HW_MODEL_TLORA_T3S3 = 16,
    HW_MODEL_NANO_G1_EXPLORER = 17,
    HW_MODEL_NANO_G2_ULTRA = 18,
    HW_MODEL_LORA_TYPE = 19,
    HW_MODEL_WI_PHONE = 20,
    HW_MODEL_WIO_WM1110 = 21,
    HW_MODEL_RAK2560 = 22,
    HW_MODEL_HELTEC_HT62 = 23,
    HW_MODEL_EBYTE900 = 24,
    HW_MODEL_EBYTE_ESP32S3 = 25,
    HW_MODEL_ESP32S3_PICO = 26,
    HW_MODEL_CHATTER2 = 27,
    HW_MODEL_HELTEC_WIRELESS_PAPER_V1_0 = 28,
    HW_MODEL_HELTEC_WIRELESS_TRACKER_V1_0 = 29,
    HW_MODEL_UNPHONE = 30,
    HW_MODEL_TDECK = 31,
    HW_MODEL_TWATCH_S3 = 32,
    HW_MODEL_PICOMPUTER_S3 = 33,
    HW_MODEL_HELTEC_WIFI_LORA_V3 = 34,
    HW_MODEL_PRIVATE_HW = 255
} HardwareModel;

// Role enum
typedef enum {
    ROLE_CLIENT = 0,
    ROLE_CLIENT_MUTE = 1,
    ROLE_ROUTER = 2,
    ROLE_ROUTER_CLIENT = 3,
    ROLE_REPEATER = 4,
    ROLE_TRACKER = 5,
    ROLE_SENSOR = 6,
    ROLE_TAK = 7,
    ROLE_CLIENT_HIDDEN = 8,
    ROLE_LOST_AND_FOUND = 9,
    ROLE_TAK_TRACKER = 10
} Role;

// Forward declarations
typedef struct Position Position;
typedef struct User User;
typedef struct DataPayload DataPayload;
typedef struct PacketPayload PacketPayload;
typedef struct MeshPacket MeshPacket;
typedef struct NodeInfo NodeInfo;
typedef struct Channel Channel;
typedef struct MeshtasticParser MeshtasticParser;
typedef struct MeshtasticFrame MeshtasticFrame;
typedef struct MeshtasticHandler MeshtasticHandler;

// Fixed-size vector structure
typedef struct {
    uint8_t data[MAX_LORA_PAYLOAD];
    size_t len;
} Vec_u8_MAX_LORA_PAYLOAD;

typedef struct {
    uint8_t data[MAX_MESSAGE_SIZE];
    size_t len;
} Vec_u8_MAX_MESSAGE_SIZE;

typedef struct {
    uint8_t data[256];
    size_t len;
} Vec_u8_256;

typedef struct {
    uint8_t data[40];
    size_t len;
} Vec_u8_40;

typedef struct {
    uint8_t data[5];
    size_t len;
} Vec_u8_5;

typedef struct {
    uint8_t data[64];
    size_t len;
} Vec_u8_64;

typedef struct {
    uint8_t data[48];
    size_t len;
} Vec_u8_48;

typedef struct {
    uint8_t data[128];
    size_t len;
} Vec_u8_128;

// Position structure
struct Position {
    int32_t latitude_i;
    int32_t longitude_i;
    int32_t altitude;
    uint32_t time;
    uint32_t timestamp;
    LocationSource location_source;
    LocationSource altitude_source;
    uint32_t pdop;
    uint32_t hdop;
    uint32_t sats_in_view;
    uint32_t ground_speed;
    uint32_t ground_track;
    uint32_t fix_quality;
    uint32_t fix_type;
    uint32_t seq_number;
};

// User structure
struct User {
    uint8_t id[8];
    Vec_u8_40 long_name;
    Vec_u8_5 short_name;
    HardwareModel hw_model;
    bool is_licensed;
    Role role;
};

// DataPayload structure
struct DataPayload {
    PortNum port;
    Vec_u8_MAX_LORA_PAYLOAD payload;
    bool want_response;
    uint32_t dest;
    uint32_t source;
    uint32_t request_id;
    uint32_t reply_id;
    uint32_t emoji;
};

// PacketPayload structure (tagged union)
typedef enum {
    PACKET_PAYLOAD_ENCRYPTED,
    PACKET_PAYLOAD_DECODED
} PacketPayloadType;

struct PacketPayload {
    PacketPayloadType type;
    union {
        Vec_u8_MAX_LORA_PAYLOAD encrypted;
        DataPayload decoded;
    } data;
};

// MeshPacket structure
struct MeshPacket {
    uint32_t from;
    uint32_t to;
    uint8_t channel;
    uint32_t id;
    uint8_t hop_limit;
    bool want_ack;
    Priority priority;
    uint32_t rx_time;
    float rx_snr;
    int32_t rx_rssi;
    PacketPayload payload;
};

// NodeInfo structure
struct NodeInfo {
    uint32_t num;
    User* user;  // NULL if not present
    Position* position;  // NULL if not present
    uint32_t last_heard;
    float snr;
};

// Deque structure for pending responses
typedef struct {
    Vec_u8_MAX_MESSAGE_SIZE items[16];
    size_t head;
    size_t tail;
    size_t count;
} Deque_Vec_u8_MAX_MESSAGE_SIZE;

// Channel structure placeholder (from channel module)
struct Channel {
    uint8_t key[32];
    uint8_t name[12];
    size_t name_len;
    // Additional channel fields would be defined in channel.h
};

// MeshtasticParser structure placeholder (implementation details)
struct MeshtasticParser {
    uint8_t state;
    Vec_u8_MAX_MESSAGE_SIZE buffer;
    size_t expected_length;
    size_t current_index;
};

// MeshtasticFrame structure
struct MeshtasticFrame {
    Vec_u8_MAX_MESSAGE_SIZE payload;
};

// ToRadioResponse structure (tagged union)
typedef enum {
    TO_RADIO_RESPONSE_LORA_PACKET,
    TO_RADIO_RESPONSE_FROM_RADIO
} ToRadioResponseType;

typedef struct {
    ToRadioResponseType type;
    union {
        Vec_u8_256 lora_packet;
        Vec_u8_MAX_MESSAGE_SIZE from_radio;
    } data;
} ToRadioResponse;

// MeshtasticHandler structure
struct MeshtasticHandler {
    uint32_t node_id;
    Channel primary_channel;
    Channel* secondary_channels[7];  // NULL if not present
    uint32_t last_packet_id;
    uint32_t rx_count;
    uint32_t tx_count;
    Position* position;  // NULL if not present
    User* user;  // NULL if not present
    MeshtasticParser parser;
    Deque_Vec_u8_MAX_MESSAGE_SIZE pending_responses;
    uint32_t config_request_id;
    uint8_t config_channel_index;
};

// Function declarations

// PortNum conversion
PortNum port_num_from_u32(uint32_t v);

// Priority conversion
Priority priority_from_u8(uint8_t v);

// Position functions
void position_init(Position* pos);

// DataPayload functions
void data_payload_init(DataPayload* payload);

// MeshPacket functions
void mesh_packet_init(MeshPacket* packet);
MeshPacket* mesh_packet_clone(const MeshPacket* packet);
void mesh_packet_free(MeshPacket* packet);

// PacketPayload functions
void packet_payload_init_encrypted(PacketPayload* payload);
void packet_payload_init_decoded(PacketPayload* payload, const DataPayload* data);
PacketPayload* packet_payload_clone(const PacketPayload* payload);
void packet_payload_free(PacketPayload* payload);

// Vec helper functions
bool vec_u8_max_lora_payload_push(Vec_u8_MAX_LORA_PAYLOAD* vec, uint8_t byte);
bool vec_u8_max_lora_payload_extend(Vec_u8_MAX_LORA_PAYLOAD* vec, const uint8_t* data, size_t len);
bool vec_u8_max_message_size_push(Vec_u8_MAX_MESSAGE_SIZE* vec, uint8_t byte);
bool vec_u8_max_message_size_extend(Vec_u8_MAX_MESSAGE_SIZE* vec, const uint8_t* data, size_t len);
bool vec_u8_256_push(Vec_u8_256* vec, uint8_t byte);
bool vec_u8_256_extend(Vec_u8_256* vec, const uint8_t* data, size_t len);
bool vec_u8_40_extend(Vec_u8_40* vec, const uint8_t* data, size_t len);
bool vec_u8_5_extend(Vec_u8_5* vec, const uint8_t* data, size_t len);
bool vec_u8_64_extend(Vec_u8_64* vec, const uint8_t* data, size_t len);
bool vec_u8_48_extend(Vec_u8_48* vec, const uint8_t* data, size_t len);
bool vec_u8_128_extend(Vec_u8_128* vec, const uint8_t* data, size_t len);
void vec_clear(void* vec, size_t element_size);

// Deque functions
void deque_init(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque);
bool deque_push_back(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque, const Vec_u8_MAX_MESSAGE_SIZE* item);
bool deque_pop_front(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque, Vec_u8_MAX_MESSAGE_SIZE* out);
void deque_clear(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque);
size_t deque_len(const Deque_Vec_u8_MAX_MESSAGE_SIZE* deque);
bool deque_is_empty(const Deque_Vec_u8_MAX_MESSAGE_SIZE* deque);

// MeshtasticHandler functions
MeshtasticHandler* meshtastic_handler_new(uint32_t node_id);
void meshtastic_handler_free(MeshtasticHandler* handler);
void meshtastic_handler_set_channel_key(MeshtasticHandler* handler, const uint8_t* psk, size_t psk_len);
uint32_t meshtastic_handler_next_packet_id(MeshtasticHandler* handler);
MeshPacket* meshtastic_handler_process_lora_packet(MeshtasticHandler* handler, const uint8_t* data, size_t len, int32_t rssi, float snr);
MeshPacket* meshtastic_handler_decrypt_packet(const MeshtasticHandler* handler, const MeshPacket* packet);
bool meshtastic_handler_create_packet(MeshtasticHandler* handler, uint32_t to, PortNum port, const uint8_t* payload, size_t payload_len, bool want_ack, Vec_u8_256* out);
bool meshtastic_handler_create_text_message(MeshtasticHandler* handler, uint32_t to, const char* text, Vec_u8_256* out);
bool meshtastic_handler_create_position_packet(MeshtasticHandler* handler, Vec_u8_256* out);
bool meshtastic_handler_create_node_info_packet(MeshtasticHandler* handler, Vec_u8_256* out);
bool meshtastic_handler_parse_serial_frame(const MeshtasticHandler* handler, const uint8_t* data, size_t len, Vec_u8_MAX_MESSAGE_SIZE* out);
bool meshtastic_handler_build_serial_frame(const MeshtasticHandler* handler, const uint8_t* payload, size_t payload_len, Vec_u8_MAX_MESSAGE_SIZE* out);
MeshtasticFrame* meshtastic_handler_feed_serial(MeshtasticHandler* handler, uint8_t byte);
bool meshtastic_handler_build_lora_packet(MeshtasticHandler* handler, const MeshtasticFrame* frame, Vec_u8_256* out);
void meshtastic_handler_reset_parser(MeshtasticHandler* handler);
ToRadioResponse* meshtastic_handler_process_toradio(MeshtasticHandler* handler, const MeshtasticFrame* frame);
ToRadioResponse* meshtastic_handler_build_config_response(MeshtasticHandler* handler, uint32_t config_id);
ToRadioResponse* meshtastic_handler_poll_pending_response(MeshtasticHandler* handler);
bool meshtastic_handler_has_pending_responses(const MeshtasticHandler* handler);
size_t meshtastic_handler_pending_response_count(const MeshtasticHandler* handler);
bool meshtastic_handler_encode_privacy_myinfo(const MeshtasticHandler* handler, Vec_u8_MAX_MESSAGE_SIZE* out);
bool meshtastic_handler_encode_privacy_nodeinfo(const MeshtasticHandler* handler, Vec_u8_MAX_MESSAGE_SIZE* out);
bool meshtastic_handler_encode_channel_config(const MeshtasticHandler* handler, uint8_t index, Vec_u8_MAX_MESSAGE_SIZE* out);
bool meshtastic_handler_encode_config_complete(const MeshtasticHandler* handler, uint32_t config_id, Vec_u8_MAX_MESSAGE_SIZE* out);
bool meshtastic_handler_handle_admin_message(MeshtasticHandler* handler, const uint8_t* payload, size_t payload_len, Vec_u8_MAX_MESSAGE_SIZE* out);

// Helper functions for protobuf encoding/decoding
typedef struct {
    uint64_t value;
    size_t consumed;
} VarintResult;

VarintResult decode_varint(const uint8_t* data, size_t len);
bool encode_varint(uint64_t value, Vec_u8_MAX_MESSAGE_SIZE* buf);
bool write_tag(uint32_t field_num, uint32_t wire_type, Vec_u8_MAX_MESSAGE_SIZE* buf);
bool encode_varint_to_vec_64(uint64_t value, Vec_u8_64* buf);
bool write_tag_to_vec_64(uint32_t field_num, uint32_t wire_type, Vec_u8_64* buf);
bool encode_varint_to_vec_48(uint64_t value, Vec_u8_48* buf);
bool write_tag_to_vec_48(uint32_t field_num, uint32_t wire_type, Vec_u8_48* buf);
bool encode_varint_to_vec_128(uint64_t value, Vec_u8_128* buf);
bool write_tag_to_vec_128(uint32_t field_num, uint32_t wire_type, Vec_u8_128* buf);

// Node ID formatting
void format_node_id(uint32_t node_id, char* out, size_t out_len);

// Parser functions
void meshtastic_parser_init(MeshtasticParser* parser);
void meshtastic_parser_reset(MeshtasticParser* parser);
MeshtasticFrame* meshtastic_parser_feed(MeshtasticParser* parser, uint8_t byte);

// Channel functions (placeholders - would be in channel.h)
void channel_init(Channel* channel);
void channel_set_key(Channel* channel, const uint8_t* psk, size_t psk_len);
bool channel_decrypt(const Channel* channel, uint32_t packet_id, uint32_t from, const Vec_u8_MAX_LORA_PAYLOAD* encrypted, Vec_u8_MAX_LORA_PAYLOAD* out);
bool channel_encrypt(const Channel* channel, uint32_t packet_id, uint32_t from, const Vec_u8_MAX_LORA_PAYLOAD* plaintext, Vec_u8_MAX_LORA_PAYLOAD* out);
const char* channel_name_str(const Channel* channel);

// Packet functions (placeholders - would be in packet.h)
MeshPacket* parse_lora_packet(const uint8_t* data, size_t len);
bool build_lora_packet(uint32_t from, uint32_t to, uint32_t id, uint8_t channel, uint8_t hop_limit, bool want_ack, const Vec_u8_MAX_LORA_PAYLOAD* payload, Vec_u8_256* out);

// Protobuf functions (placeholders - would be in protobuf.h)
DataPayload* protobuf_decode_data(const Vec_u8_MAX_LORA_PAYLOAD* data);
bool protobuf_encode_data(const DataPayload* data, Vec_u8_MAX_LORA_PAYLOAD* out);
bool protobuf_encode_position(const Position* position, Vec_u8_MAX_LORA_PAYLOAD* out);
bool protobuf_encode_user(const User* user, Vec_u8_MAX_LORA_PAYLOAD* out);

#endif // MESHTASTIC_H

// meshtastic.c
#include "meshtastic.h"
#include <stdlib.h>

// PortNum conversion implementation
PortNum port_num_from_u32(uint32_t v) {
    switch (v) {
        case 1: return PORT_NUM_TEXT_MESSAGE;
        case 2: return PORT_NUM_REMOTE_HARDWARE;
        case 3: return PORT_NUM_POSITION;
        case 4: return PORT_NUM_NODE_INFO;
        case 5: return PORT_NUM_ROUTING;
        case 6: return PORT_NUM_ADMIN;
        case 7: return PORT_NUM_TEXT_MESSAGE_COMPRESSED;
        case 8: return PORT_NUM_WAYPOINT;
        case 32: return PORT_NUM_REPLY;
        case 33: return PORT_NUM_IP_TUNNEL_APP;
        case 64: return PORT_NUM_SERIAL;
        case 65: return PORT_NUM_STORE_FORWARD;
        case 66: return PORT_NUM_RANGE_TEST;
        case 67: return PORT_NUM_TELEMETRY;
        case 70: return PORT_NUM_TRACEROUTE;
        case 71: return PORT_NUM_NEIGHBORINFO;
        default: return PORT_NUM_UNKNOWN;
    }
}

// Priority conversion implementation
Priority priority_from_u8(uint8_t v) {
    switch (v) {
        case 1: return PRIORITY_MIN;
        case 10: return PRIORITY_BACKGROUND;
        case 70: return PRIORITY_RELIABLE;
        case 120: return PRIORITY_ACK;
        case 127: return PRIORITY_MAX;
        default: return PRIORITY_DEFAULT;
    }
}

// Position initialization
void position_init(Position* pos) {
    memset(pos, 0, sizeof(Position));
    pos->location_source = LOCATION_SOURCE_UNSET;
    pos->altitude_source = LOCATION_SOURCE_UNSET;
}

// DataPayload initialization
void data_payload_init(DataPayload* payload) {
    memset(payload, 0, sizeof(DataPayload));
    payload->port = PORT_NUM_UNKNOWN;
    payload->payload.len = 0;
    payload->want_response = false;
    payload->dest = 0;
    payload->source = 0;
    payload->request_id = 0;
    payload->reply_id = 0;
    payload->emoji = 0;
}

// MeshPacket initialization
void mesh_packet_init(MeshPacket* packet) {
    memset(packet, 0, sizeof(MeshPacket));
    packet->from = 0;
    packet->to = 0xFFFFFFFF;
    packet->channel = 0;
    packet->id = 0;
    packet->hop_limit = DEFAULT_HOP_LIMIT;
    packet->want_ack = false;
    packet->priority = PRIORITY_DEFAULT;
    packet->rx_time = 0;
    packet->rx_snr = 0.0f;
    packet->rx_rssi = 0;
    packet_payload_init_encrypted(&packet->payload);
}

// MeshPacket clone
MeshPacket* mesh_packet_clone(const MeshPacket* packet) {
    if (!packet) return NULL;
    
    MeshPacket* clone = (MeshPacket*)malloc(sizeof(MeshPacket));
    if (!clone) return NULL;
    
    memcpy(clone, packet, sizeof(MeshPacket));
    return clone;
}

// MeshPacket free
void mesh_packet_free(MeshPacket* packet) {
    if (packet) {
        free(packet);
    }
}

// PacketPayload initialization - encrypted
void packet_payload_init_encrypted(PacketPayload* payload) {
    payload->type = PACKET_PAYLOAD_ENCRYPTED;
    payload->data.encrypted.len = 0;
}

// PacketPayload initialization - decoded
void packet_payload_init_decoded(PacketPayload* payload, const DataPayload* data) {
    payload->type = PACKET_PAYLOAD_DECODED;
    memcpy(&payload->data.decoded, data, sizeof(DataPayload));
}

// PacketPayload clone
PacketPayload* packet_payload_clone(const PacketPayload* payload) {
    if (!payload) return NULL;
    
    PacketPayload* clone = (PacketPayload*)malloc(sizeof(PacketPayload));
    if (!clone) return NULL;
    
    memcpy(clone, payload, sizeof(PacketPayload));
    return clone;
}

// PacketPayload free
void packet_payload_free(PacketPayload* payload) {
    if (payload) {
        free(payload);
    }
}

// Vec helper functions
bool vec_u8_max_lora_payload_push(Vec_u8_MAX_LORA_PAYLOAD* vec, uint8_t byte) {
    if (!vec || vec->len >= MAX_LORA_PAYLOAD) return false;
    vec->data[vec->len++] = byte;
    return true;
}

bool vec_u8_max_lora_payload_extend(Vec_u8_MAX_LORA_PAYLOAD* vec, const uint8_t* data, size_t len) {
    if (!vec || !data || vec->len + len > MAX_LORA_PAYLOAD) return false;
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
    return true;
}

bool vec_u8_max_message_size_push(Vec_u8_MAX_MESSAGE_SIZE* vec, uint8_t byte) {
    if (!vec || vec->len >= MAX_MESSAGE_SIZE) return false;
    vec->data[vec->len++] = byte;
    return true;
}

bool vec_u8_max_message_size_extend(Vec_u8_MAX_MESSAGE_SIZE* vec, const uint8_t* data, size_t len) {
    if (!vec || !data || vec->len + len > MAX_MESSAGE_SIZE) return false;
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
    return true;
}

bool vec_u8_256_push(Vec_u8_256* vec, uint8_t byte) {
    if (!vec || vec->len >= 256) return false;
    vec->data[vec->len++] = byte;
    return true;
}

bool vec_u8_256_extend(Vec_u8_256* vec, const uint8_t* data, size_t len) {
    if (!vec || !data || vec->len + len > 256) return false;
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
    return true;
}

bool vec_u8_40_extend(Vec_u8_40* vec, const uint8_t* data, size_t len) {
    if (!vec || !data || vec->len + len > 40) return false;
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
    return true;
}

bool vec_u8_5_extend(Vec_u8_5* vec, const uint8_t* data, size_t len) {
    if (!vec || !data || vec->len + len > 5) return false;
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
    return true;
}

bool vec_u8_64_extend(Vec_u8_64* vec, const uint8_t* data, size_t len) {
    if (!vec || !data || vec->len + len > 64) return false;
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
    return true;
}

bool vec_u8_48_extend(Vec_u8_48* vec, const uint8_t* data, size_t len) {
    if (!vec || !data || vec->len + len > 48) return false;
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
    return true;
}

bool vec_u8_128_extend(Vec_u8_128* vec, const uint8_t* data, size_t len) {
    if (!vec || !data || vec->len + len > 128) return false;
    memcpy(vec->data + vec->len, data, len);
    vec->len += len;
    return true;
}

void vec_clear(void* vec, size_t element_size) {
    // Generic clear function - sets length to 0
    if (vec) {
        *(size_t*)((uint8_t*)vec + element_size) = 0;
    }
}

// Deque functions
void deque_init(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque) {
    if (!deque) return;
    deque->head = 0;
    deque->tail = 0;
    deque->count = 0;
}

bool deque_push_back(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque, const Vec_u8_MAX_MESSAGE_SIZE* item) {
    if (!deque || !item || deque->count >= 16) return false;
    
    memcpy(&deque->items[deque->tail], item, sizeof(Vec_u8_MAX_MESSAGE_SIZE));
    deque->tail = (deque->tail + 1) % 16;
    deque->count++;
    return true;
}

bool deque_pop_front(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque, Vec_u8_MAX_MESSAGE_SIZE* out) {
    if (!deque || !out || deque->count == 0) return false;
    
    memcpy(out, &deque->items[deque->head], sizeof(Vec_u8_MAX_MESSAGE_SIZE));
    deque->head = (deque->head + 1) % 16;
    deque->count--;
    return true;
}

void deque_clear(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque) {
    if (!deque) return;
    deque->head = 0;
    deque->tail = 0;
    deque->count = 0;
}

size_t deque_len(const Deque_Vec_u8_MAX_MESSAGE_SIZE* deque) {
    return deque ? deque->count : 0;
}

bool deque_is_empty(const Deque_Vec_u8_MAX_MESSAGE_SIZE* deque) {
    return deque ? (deque->count == 0) : true;
}

// Helper functions for protobuf encoding/decoding
VarintResult decode_varint(const uint8_t* data, size_t len) {
    VarintResult result = {0, 0};
    if (!data || len == 0) return result;
    
    uint64_t value = 0;
    size_t shift = 0;
    size_t consumed = 0;
    
    for (size_t i = 0; i < len && i < 10; i++) {
        uint8_t byte = data[i];
        value |= ((uint64_t)(byte & 0x7F)) << shift;
        consumed++;
        
        if ((byte & 0x80) == 0) {
            break;
        }
        shift += 7;
    }
    
    result.value = value;
    result.consumed = consumed;
    return result;
}

bool encode_varint(uint64_t value, Vec_u8_MAX_MESSAGE_SIZE* buf) {
    if (!buf) return false;
    
    while (value >= 0x80) {
        if (!vec_u8_max_message_size_push(buf, (uint8_t)((value & 0x7F) | 0x80))) {
            return false;
        }
        value >>= 7;
    }
    
    return vec_u8_max_message_size_push(buf, (uint8_t)value);
}

bool write_tag(uint32_t field_num, uint32_t wire_type, Vec_u8_MAX_MESSAGE_SIZE* buf) {
    if (!buf) return false;
    uint64_t tag = ((uint64_t)field_num << 3) | wire_type;
    return encode_varint(tag, buf);
}

bool encode_varint_to_vec_64(uint64_t value, Vec_u8_64* buf) {
    if (!buf) return false;
    
    while (value >= 0x80) {
        if (buf->len >= 64) return false;
        buf->data[buf->len++] = (uint8_t)((value & 0x7F) | 0x80);
        value >>= 7;
    }
    
    if (buf->len >= 64) return false;
    buf->data[buf->len++] = (uint8_t)value;
    return true;
}

bool write_tag_to_vec_64(uint32_t field_num, uint32_t wire_type, Vec_u8_64* buf) {
    if (!buf) return false;
    uint64_t tag = ((uint64_t)field_num << 3) | wire_type;
    return encode_varint_to_vec_64(tag, buf);
}

bool encode_varint_to_vec_48(uint64_t value, Vec_u8_48* buf) {
    if (!buf) return false;
    
    while (value >= 0x80) {
        if (buf->len >= 48) return false;
        buf->data[buf->len++] = (uint8_t)((value & 0x7F) | 0x80);
        value >>= 7;
    }
    
    if (buf->len >= 48) return false;
    buf->data[buf->len++] = (uint8_t)value;
    return true;
}

bool write_tag_to_vec_48(uint32_t field_num, uint32_t wire_type, Vec_u8_48* buf) {
    if (!buf) return false;
    uint64_t tag = ((uint64_t)field_num << 3) | wire_type;
    return encode_varint_to_vec_48(tag, buf);
}

bool encode_varint_to_vec_128(uint64_t value, Vec_u8_128* buf) {
    if (!buf) return false;
    
    while (value >= 0x80) {
        if (buf->len >= 128) return false;
        buf->data[buf->len++] = (uint8_t)((value & 0x7F) | 0x80);
        value >>= 7;
    }
    
    if (buf->len >= 128) return false;
    buf->data[buf->len++] = (uint8_t)value;
    return true;
}

bool write_tag_to_vec_128(uint32_t field_num, uint32_t wire_type, Vec_u8_128* buf) {
    if (!buf) return false;
    uint64_t tag = ((uint64_t)field_num << 3) | wire_type;
    return encode_varint_to_vec_128(tag, buf);
}

// Node ID formatting
void format_node_id(uint32_t node_id, char* out, size_t out_len) {
    if (!out || out_len < 9) return;
    snprintf(out, out_len, "!%08x", node_id);
}

// Parser functions
void meshtastic_parser_init(MeshtasticParser* parser) {
    if (!parser) return;
    parser->state = 0;
    parser->buffer.len = 0;
    parser->expected_length = 0;
    parser->current_index = 0;
}

void meshtastic_parser_reset(MeshtasticParser* parser) {
    meshtastic_parser_init(parser);
}

MeshtasticFrame* meshtastic_parser_feed(MeshtasticParser* parser, uint8_t byte) {
    if (!parser) return NULL;
    
    // State machine for parsing serial frames
    switch (parser->state) {
        case 0: // Waiting for first sync byte
            if (byte == SERIAL_SYNC[0]) {
                parser->state = 1;
            }
            break;
            
        case 1: // Waiting for second sync byte
            if (byte == SERIAL_SYNC[1]) {
                parser->state = 2;
                parser->buffer.len = 0;
            } else {
                parser->state = 0;
            }
            break;
            
        case 2: // Reading length high byte
            parser->expected_length = (size_t)byte << 8;
            parser->state = 3;
            break;
            
        case 3: // Reading length low byte
            parser->expected_length |= byte;
            if (parser->expected_length > MAX_MESSAGE_SIZE) {
                parser->state = 0;
                return NULL;
            }
            parser->state = 4;
            parser->current_index = 0;
            break;
            
        case 4: // Reading payload
            if (parser->buffer.len < MAX_MESSAGE_SIZE) {
                parser->buffer.data[parser->buffer.len++] = byte;
                parser->current_index++;
                
                if (parser->current_index >= parser->expected_length) {
                    // Frame complete
                    MeshtasticFrame* frame = (MeshtasticFrame*)malloc(sizeof(MeshtasticFrame));
                    if (frame) {
                        memcpy(&frame->payload, &parser->buffer, sizeof(Vec_u8_MAX_MESSAGE_SIZE));
                    }
                    parser->state = 0;
                    parser->buffer.len = 0;
                    return frame;
                }
            } else {
                parser->state = 0;
            }
            break;
            
        default:
            parser->state = 0;
            break;
    }
    
    return NULL;
}

// MeshtasticHandler implementation
MeshtasticHandler* meshtastic_handler_new(uint32_t node_id) {
    MeshtasticHandler* handler = (MeshtasticHandler*)malloc(sizeof(MeshtasticHandler));
    if (!handler) return NULL;
    
    handler->node_id = node_id;
    channel_init(&handler->primary_channel);
    
    for (int i = 0; i < 7; i++) {
        handler->secondary_channels[i] = NULL;
    }
    
    handler->last_packet_id = 0;
    handler->rx_count = 0;
    handler->tx_count = 0;
    handler->position = NULL;
    handler->user = NULL;
    
    meshtastic_parser_init(&handler->parser);
    deque_init(&handler->pending_responses);
    
    handler->config_request_id = 0;
    handler->config_channel_index = 0;
    
    return handler;
}

void meshtastic_handler_free(MeshtasticHandler* handler) {
    if (!handler) return;
    
    // Free secondary channels
    for (int i = 0; i < 7; i++) {
        if (handler->secondary_channels[i]) {
            free(handler->secondary_channels[i]);
        }
    }
    
    // Free position if allocated
    if (handler->position) {
        free(handler->position);
    }
    
    // Free user if allocated
    if (handler->user) {
        free(handler->user);
    }
    
    free(handler);
}

void meshtastic_handler_set_channel_key(MeshtasticHandler* handler, const uint8_t* psk, size_t psk_len) {
    if (!handler || !psk) return;
    channel_set_key(&handler->primary_channel, psk, psk_len);
}

uint32_t meshtastic_handler_next_packet_id(MeshtasticHandler* handler) {
    if (!handler) return 0;
    
    handler->last_packet_id = handler->last_packet_id + 1;
    if (handler->last_packet_id == 0) {
        handler->last_packet_id = 1;
    }
    return handler->last_packet_id;
}

MeshPacket* meshtastic_handler_process_lora_packet(MeshtasticHandler* handler, const uint8_t* data, size_t len, int32_t rssi, float snr) {
    if (!handler || !data || len < LORA_HEADER_SIZE + MIC_SIZE) {
        return NULL;
    }
    
    MeshPacket* packet = parse_lora_packet(data, len);
    if (!packet) return NULL;
    
    packet->rx_rssi = rssi;
    packet->rx_snr = snr;
    
    MeshPacket* decrypted = meshtastic_handler_decrypt_packet(handler, packet);
    
    if (decrypted) {
        handler->rx_count++;
    }
    
    mesh_packet_free(packet);
    
    return decrypted;
}

MeshPacket* meshtastic_handler_decrypt_packet(const MeshtasticHandler* handler, const MeshPacket* packet) {
    if (!handler || !packet) return NULL;
    
    MeshPacket* result = mesh_packet_clone(packet);
    if (!result) return NULL;
    
    if (packet->payload.type == PACKET_PAYLOAD_ENCRYPTED) {
        const Vec_u8_MAX_LORA_PAYLOAD* encrypted = &packet->payload.data.encrypted;
        
        const Channel* channel;
        if (packet->channel == 0) {
            channel = &handler->primary_channel;
        } else {
            size_t idx = packet->channel - 1;
            if (idx >= 7 || !handler->secondary_channels[idx]) {
                mesh_packet_free(result);
                return NULL;
            }
            channel = handler->secondary_channels[idx];
        }
        
        Vec_u8_MAX_LORA_PAYLOAD decrypted;
        decrypted.len = 0;
        
        if (!channel_decrypt(channel, packet->id, packet->from, encrypted, &decrypted)) {
            mesh_packet_free(result);
            return NULL;
        }
        
        DataPayload* data = protobuf_decode_data(&decrypted);
        if (data) {
            packet_payload_init_decoded(&result->payload, data);
            free(data);
        }
    }
    
    return result;
}

bool meshtastic_handler_create_packet(MeshtasticHandler* handler, uint32_t to, PortNum port, const uint8_t* payload, size_t payload_len, bool want_ack, Vec_u8_256* out) {
    if (!handler || !payload || !out) return false;
    
    uint32_t packet_id = meshtastic_handler_next_packet_id(handler);
    
    DataPayload data;
    data_payload_init(&data);
    data.port = port;
    
    if (!vec_u8_max_lora_payload_extend(&data.payload, payload, payload_len)) {
        return false;
    }
    
    data.want_response = want_ack;
    data.source = handler->node_id;
    data.dest = to;
    
    Vec_u8_MAX_LORA_PAYLOAD encoded;
    encoded.len = 0;
    
    if (!protobuf_encode_data(&data, &encoded)) {
        return false;
    }
    
    Vec_u8_MAX_LORA_PAYLOAD encrypted;
    encrypted.len = 0;
    
    if (!channel_encrypt(&handler->primary_channel, packet_id, handler->node_id, &encoded, &encrypted)) {
        return false;
    }
    
    if (!build_lora_packet(handler->node_id, to, packet_id, 0, DEFAULT_HOP_LIMIT, want_ack, &encrypted, out)) {
        return false;
    }
    
    handler->tx_count++;
    
    return true;
}

bool meshtastic_handler_create_text_message(MeshtasticHandler* handler, uint32_t to, const char* text, Vec_u8_256* out) {
    if (!handler || !text || !out) return false;
    return meshtastic_handler_create_packet(handler, to, PORT_NUM_TEXT_MESSAGE, (const uint8_t*)text, strlen(text), true, out);
}

bool meshtastic_handler_create_position_packet(MeshtasticHandler* handler, Vec_u8_256* out) {
    if (!handler || !out || !handler->position) return false;
    
    Vec_u8_MAX_LORA_PAYLOAD encoded;
    encoded.len = 0;
    
    if (!protobuf_encode_position(handler->position, &encoded)) {
        return false;
    }
    
    return meshtastic_handler_create_packet(handler, 0xFFFFFFFF, PORT_NUM_POSITION, encoded.data, encoded.len, false, out);
}

bool meshtastic_handler_create_node_info_packet(MeshtasticHandler* handler, Vec_u8_256* out) {
    if (!handler || !out || !handler->user) return false;
    
    Vec_u8_MAX_LORA_PAYLOAD encoded;
    encoded.len = 0;
    
    if (!protobuf_encode_user(handler->user, &encoded)) {
        return false;
    }
    
    return meshtastic_handler_create_packet(handler, 0xFFFFFFFF, PORT_NUM_NODE_INFO, encoded.data, encoded.len, false, out);
}

bool meshtastic_handler_parse_serial_frame(const MeshtasticHandler* handler, const uint8_t* data, size_t len, Vec_u8_MAX_MESSAGE_SIZE* out) {
    if (!handler || !data || !out || len < 4) return false;
    
    if (data[0] != SERIAL_SYNC[0] || data[1] != SERIAL_SYNC[1]) {
        return false;
    }
    
    size_t payload_len = ((size_t)data[2] << 8) | data[3];
    if (len < 4 + payload_len) {
        return false;
    }
    
    out->len = 0;
    return vec_u8_max_message_size_extend(out, data + 4, payload_len);
}

bool meshtastic_handler_build_serial_frame(const MeshtasticHandler* handler, const uint8_t* payload, size_t payload_len, Vec_u8_MAX_MESSAGE_SIZE* out) {
    if (!handler || !payload || !out) return false;
    
    out->len = 0;
    
    if (!vec_u8_max_message_size_push(out, SERIAL_SYNC[0])) return false;
    if (!vec_u8_max_message_size_push(out, SERIAL_SYNC[1])) return false;
    
    uint16_t len_u16 = (uint16_t)payload_len;
    if (!vec_u8_max_message_size_push(out, (uint8_t)(len_u16 >> 8))) return false;
    if (!vec_u8_max_message_size_push(out, (uint8_t)(len_u16 & 0xFF))) return false;
    
    return vec_u8_max_message_size_extend(out, payload, payload_len);
}

MeshtasticFrame* meshtastic_handler_feed_serial(MeshtasticHandler* handler, uint8_t byte) {
    if (!handler) return NULL;
    return meshtastic_parser_feed(&handler->parser, byte);
}

bool meshtastic_handler_build_lora_packet(MeshtasticHandler* handler, const MeshtasticFrame* frame, Vec_u8_256* out) {
    if (!handler || !frame || !out) return false;
    
    if (frame->payload.len == 0) return false;
    
    const uint8_t* payload = frame->payload.data;
    size_t payload_len = frame->payload.len;
    
    if (payload_len < 2) return false;
    
    // Check for mesh packet field tag (0x0A = field 1, wire type 2)
    if (payload[0] != 0x0A) return false;
    
    size_t mesh_packet_len = payload[1];
    if (payload_len < 2 + mesh_packet_len) return false;
    
    const uint8_t* mesh_packet_data = payload + 2;
    
    // Parse mesh packet fields
    uint32_t to = 0xFFFFFFFF;
    uint8_t channel = 0;
    bool want_ack = false;
    uint8_t hop_limit = DEFAULT_HOP_LIMIT;
    Vec_u8_MAX_LORA_PAYLOAD inner_payload;
    inner_payload.len = 0;
    PortNum port = PORT_NUM_UNKNOWN;
    bool has_payload = false;
    
    size_t idx = 0;
    while (idx < mesh_packet_len) {
        if (idx >= mesh_packet_len) break;
        
        uint8_t tag = mesh_packet_data[idx];
        idx++;
        if (idx > mesh_packet_len) break;
        
        uint32_t field_num = tag >> 3;
        uint32_t wire_type = tag & 0x07;
        
        if (field_num == 2 && wire_type == 0) { // to field
            VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
            to = (uint32_t)result.value;
            idx += result.consumed;
        }
        else if (field_num == 4 && wire_type == 0) { // channel field
            VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
            channel = (uint8_t)result.value;
            idx += result.consumed;
        }
        else if (field_num == 6 && wire_type == 0) { // hop_limit field
            VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
            hop_limit = (uint8_t)result.value;
            idx += result.consumed;
        }
        else if (field_num == 7 && wire_type == 0) { // want_ack field
            VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
            want_ack = result.value != 0;
            idx += result.consumed;
        }
        else if (field_num == 8 && wire_type == 2) { // decoded field (Data message)
            if (idx >= mesh_packet_len) break;
            size_t data_len = mesh_packet_data[idx];
            idx++;
            if (idx + data_len > mesh_packet_len) break;
            
            const uint8_t* data_msg = mesh_packet_data + idx;
            
            // Parse Data message fields
            size_t data_idx = 0;
            while (data_idx < data_len) {
                if (data_idx >= data_len) break;
                uint8_t dtag = data_msg[data_idx];
                data_idx++;
                if (data_idx > data_len) break;
                
                uint32_t dfield = dtag >> 3;
                uint32_t dwire = dtag & 0x07;
                
                if (dfield == 1 && dwire == 0) { // portnum field
                    VarintResult result = decode_varint(data_msg + data_idx, data_len - data_idx);
                    port = port_num_from_u32((uint32_t)result.value);
                    data_idx += result.consumed;
                }
                else if (dfield == 2 && dwire == 2) { // payload field
                    if (data_idx >= data_len) break;
                    size_t plen = data_msg[data_idx];
                    data_idx++;
                    if (data_idx + plen > data_len) break;
                    
                    vec_u8_max_lora_payload_extend(&inner_payload, data_msg + data_idx, plen);
                    has_payload = true;
                    data_idx += plen;
                }
                else if (dwire == 0) { // Skip varint
                    VarintResult result = decode_varint(data_msg + data_idx, data_len - data_idx);
                    data_idx += result.consumed;
                }
                else if (dwire == 2) { // Skip length-delimited
                    if (data_idx >= data_len) break;
                    size_t skip_len = data_msg[data_idx];
                    data_idx += 1 + skip_len;
                }
                else {
                    break;
                }
            }
            
            idx += data_len;
        }
        else if (wire_type == 0) { // Skip varint
            VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
            idx += result.consumed;
        }
        else if (wire_type == 2) { // Skip length-delimited
            if (idx >= mesh_packet_len) break;
            size_t skip_len = mesh_packet_data[idx];
            idx += 1 + skip_len;
        }
        else {
            break;
        }
    }
    
    if (!has_payload) return false;
    
    return meshtastic_handler_create_packet(handler, to, port, inner_payload.data, inner_payload.len, want_ack, out);
}

void meshtastic_handler_reset_parser(MeshtasticHandler* handler) {
    if (!handler) return;
    meshtastic_parser_reset(&handler->parser);
}

ToRadioResponse* meshtastic_handler_process_toradio(MeshtasticHandler* handler, const MeshtasticFrame* frame) {
    if (!handler || !frame || frame->payload.len == 0) return NULL;
    
    const uint8_t* payload = frame->payload.data;
    size_t payload_len = frame->payload.len;
    
    if (payload_len < 2) return NULL;
    
    uint8_t field_tag = payload[0];
    uint32_t field_num = field_tag >> 3;
    uint32_t wire_type = field_tag & 0x07;
    
    if (field_num == 1 && wire_type == 2) { // packet field
        Vec_u8_256 lora_packet;
        lora_packet.len = 0;
        
        if (meshtastic_handler_build_lora_packet(handler, frame, &lora_packet)) {
            ToRadioResponse* response = (ToRadioResponse*)malloc(sizeof(ToRadioResponse));
            if (response) {
                response->type = TO_RADIO_RESPONSE_LORA_PACKET;
                memcpy(&response->data.lora_packet, &lora_packet, sizeof(Vec_u8_256));
            }
            return response;
        }
    }
    else if (field_num == 3 && wire_type == 0) { // want_config_id field
        VarintResult result = decode_varint(payload + 1, payload_len - 1);
        uint32_t config_id = (uint32_t)result.value;
        return meshtastic_handler_build_config_response(handler, config_id);
    }
    else if (field_num == 4 && wire_type == 0) { // disconnect field
        meshtastic_handler_reset_parser(handler);
        return NULL;
    }
    
    return NULL;
}

ToRadioResponse* meshtastic_handler_build_config_response(MeshtasticHandler* handler, uint32_t config_id) {
    if (!handler) return NULL;
    
    // Clear pending responses
    deque_clear(&handler->pending_responses);
    handler->config_request_id = config_id;
    handler->config_channel_index = 0;
    
    // Encode and queue my_info
    Vec_u8_MAX_MESSAGE_SIZE my_info;
    my_info.len = 0;
    if (meshtastic_handler_encode_privacy_myinfo(handler, &my_info)) {
        deque_push_back(&handler->pending_responses, &my_info);
    }
    
    // Encode and queue node_info
    Vec_u8_MAX_MESSAGE_SIZE node_info;
    node_info.len = 0;
    if (meshtastic_handler_encode_privacy_nodeinfo(handler, &node_info)) {
        deque_push_back(&handler->pending_responses, &node_info);
    }
    
    // Encode and queue primary channel config
    Vec_u8_MAX_MESSAGE_SIZE channel_config;
    channel_config.len = 0;
    if (meshtastic_handler_encode_channel_config(handler, 0, &channel_config)) {
        deque_push_back(&handler->pending_responses, &channel_config);
    }
    
    // Encode and queue secondary channel configs
    for (int i = 0; i < 7; i++) {
        if (handler->secondary_channels[i]) {
            Vec_u8_MAX_MESSAGE_SIZE sec_channel_config;
            sec_channel_config.len = 0;
            if (meshtastic_handler_encode_channel_config(handler, (uint8_t)(i + 1), &sec_channel_config)) {
                deque_push_back(&handler->pending_responses, &sec_channel_config);
            }
        }
    }
    
    // Encode and queue config complete
    Vec_u8_MAX_MESSAGE_SIZE complete;
    complete.len = 0;
    if (meshtastic_handler_encode_config_complete(handler, config_id, &complete)) {
        deque_push_back(&handler->pending_responses, &complete);
    }
    
    // Return first pending response
    return meshtastic_handler_poll_pending_response(handler);
}

ToRadioResponse* meshtastic_handler_poll_pending_response(MeshtasticHandler* handler) {
    if (!handler) return NULL;
    
    Vec_u8_MAX_MESSAGE_SIZE response;
    response.len = 0;
    
    if (deque_pop_front(&handler->pending_responses, &response)) {
        ToRadioResponse* result = (ToRadioResponse*)malloc(sizeof(ToRadioResponse));
        if (result) {
            result->type = TO_RADIO_RESPONSE_FROM_RADIO;
            memcpy(&result->data.from_radio, &response, sizeof(Vec_u8_MAX_MESSAGE_SIZE));
        }
        return result;
    }
    
    return NULL;
}

bool meshtastic_handler_has_pending_responses(const MeshtasticHandler* handler) {
    if (!handler) return false;
    return !deque_is_empty(&handler->pending_responses);
}

size_t meshtastic_handler_pending_response_count(const MeshtasticHandler* handler) {
    if (!handler) return 0;
    return deque_len(&handler->pending_responses);
}

bool meshtastic_handler_encode_privacy_myinfo(const MeshtasticHandler* handler, Vec_u8_MAX_MESSAGE_SIZE* out) {
    if (!handler || !out) return false;
    
    out->len = 0;
    
    // packet_id field (field 1, varint, value 0)
    if (!write_tag(1, WIRE_VARINT, out)) return false;
    if (!encode_varint(0, out)) return false;
    
    // my_info field (field 4, length-delimited)
    Vec_u8_64 my_info;
    my_info.len = 0;
    
    // my_node_num field (field 1, varint)
    if (!write_tag_to_vec_64(1, WIRE_VARINT, &my_info)) return false;
    if (!encode_varint_to_vec_64(handler->node_id, &my_info)) return false;
    
    // max_channels field (field 3, varint, value 30000)
    if (!write_tag_to_vec_64(3, WIRE_VARINT, &my_info)) return false;
    if (!encode_varint_to_vec_64(30000, &my_info)) return false;
    
    // num_bands field (field 4, varint, value 8)
    if (!write_tag_to_vec_64(4, WIRE_VARINT, &my_info)) return false;
    if (!encode_varint_to_vec_64(8, &my_info)) return false;
    
    // firmware_version field (field 6, varint, value 0)
    if (!write_tag_to_vec_64(6, WIRE_VARINT, &my_info)) return false;
    if (!encode_varint_to_vec_64(0, &my_info)) return false;
    
    // reboot_count field (field 7, varint, value 1)
    if (!write_tag_to_vec_64(7, WIRE_VARINT, &my_info)) return false;
    if (!encode_varint_to_vec_64(1, &my_info)) return false;
    
    // Write my_info to output
    if (!write_tag(4, WIRE_LEN, out)) return false;
    if (!encode_varint(my_info.len, out)) return false;
    if (!vec_u8_max_message_size_extend(out, my_info.data, my_info.len)) return false;
    
    return true;
}

bool meshtastic_handler_encode_privacy_nodeinfo(const MeshtasticHandler* handler, Vec_u8_MAX_MESSAGE_SIZE* out) {
    if (!handler || !out) return false;
    
    out->len = 0;
    
    // packet_id field (field 1, varint, value 0)
    if (!write_tag(1, WIRE_VARINT, out)) return false;
    if (!encode_varint(0, out)) return false;
    
    // node_info field (field 6, length-delimited)
    Vec_u8_128 node_info;
    node_info.len = 0;
    
    // num field (field 1, varint)
    if (!write_tag_to_vec_128(1, WIRE_VARINT, &node_info)) return false;
    if (!encode_varint_to_vec_128(handler->node_id, &node_info)) return false;
    
    // user field (field 2, length-delimited)
    Vec_u8_64 user;
    user.len = 0;
    
    // id field (field 1, length-delimited)
    char id_str[16];
    format_node_id(handler->node_id, id_str, sizeof(id_str));
    size_t id_len = strlen(id_str);
    
    if (!write_tag_to_vec_64(1, WIRE_LEN, &user)) return false;
    if (!encode_varint_to_vec_64(id_len, &user)) return false;
    if (!vec_u8_64_extend(&user, (const uint8_t*)id_str, id_len)) return false;
    
    // long_name field (field 2, length-delimited)
    const uint8_t* long_name;
    size_t long_name_len;
    if (handler->user) {
        long_name = handler->user->long_name.data;
        long_name_len = handler->user->long_name.len;
    } else {
        long_name = (const uint8_t*)"LunarNode";
        long_name_len = 9;
    }
    
    if (!write_tag_to_vec_64(2, WIRE_LEN, &user)) return false;
    if (!encode_varint_to_vec_64(long_name_len, &user)) return false;
    if (!vec_u8_64_extend(&user, long_name, long_name_len)) return false;
    
    // short_name field (field 3, length-delimited)
    const uint8_t* short_name;
    size_t short_name_len;
    if (handler->user) {
        short_name = handler->user->short_name.data;
        short_name_len = handler->user->short_name.len;
    } else {
        short_name = (const uint8_t*)"LNOD";
        short_name_len = 4;
    }
    
    if (!write_tag_to_vec_64(3, WIRE_LEN, &user)) return false;
    if (!encode_varint_to_vec_64(short_name_len, &user)) return false;
    if (!vec_u8_64_extend(&user, short_name, short_name_len)) return false;
    
    // hw_model field (field 5, varint, value 43)
    if (!write_tag_to_vec_64(5, WIRE_VARINT, &user)) return false;
    if (!encode_varint_to_vec_64(43, &user)) return false;
    
    // role field (field 7, varint, value 4)
    if (!write_tag_to_vec_64(7, WIRE_VARINT, &user)) return false;
    if (!encode_varint_to_vec_64(4, &user)) return false;
    
    // Write user to node_info
    if (!write_tag_to_vec_128(2, WIRE_LEN, &node_info)) return false;
    if (!encode_varint_to_vec_128(user.len, &node_info)) return false;
    if (!vec_u8_128_extend(&node_info, user.data, user.len)) return false;
    
    // Write node_info to output
    if (!write_tag(6, WIRE_LEN, out)) return false;
    if (!encode_varint(node_info.len, out)) return false;
    if (!vec_u8_max_message_size_extend(out, node_info.data, node_info.len)) return false;
    
    return true;
}

bool meshtastic_handler_encode_channel_config(const MeshtasticHandler* handler, uint8_t index, Vec_u8_MAX_MESSAGE_SIZE* out) {
    if (!handler || !out) return false;
    
    out->len = 0;
    
    // packet_id field (field 1, varint, value 0)
    if (!write_tag(1, WIRE_VARINT, out)) return false;
    if (!encode_varint(0, out)) return false;
    
    // channel field (field 8, length-delimited)
    Vec_u8_64 channel;
    channel.len = 0;
    
    // index field (field 1, varint)
    if (!write_tag_to_vec_64(1, WIRE_VARINT, &channel)) return false;
    if (!encode_varint_to_vec_64(index, &channel)) return false;
    
    // settings field (field 2, length-delimited)
    Vec_u8_48 settings;
    settings.len = 0;
    
    // name field (field 2, length-delimited)
    const char* name = channel_name_str(&handler->primary_channel);
    size_t name_len = strlen(name);
    
    if (name_len > 0) {
        if (!write_tag_to_vec_48(2, WIRE_LEN, &settings)) return false;
        if (!encode_varint_to_vec_48(name_len, &settings)) return false;
        if (!vec_u8_48_extend(&settings, (const uint8_t*)name, name_len)) return false;
    }
    
    // Write settings to channel
    if (!write_tag_to_vec_64(2, WIRE_LEN, &channel)) return false;
    if (!encode_varint_to_vec_64(settings.len, &channel)) return false;
    if (!vec_u8_64_extend(&channel, settings.data, settings.len)) return false;
    
    // role field (field 3, varint, value 1)
    if (!write_tag_to_vec_64(3, WIRE_VARINT, &channel)) return false;
    if (!encode_varint_to_vec_64(1, &channel)) return false;
    
    // Write channel to output
    if (!write_tag(8, WIRE_LEN, out)) return false;
    if (!encode_varint(channel.len, out)) return false;
    if (!vec_u8_max_message_size_extend(out, channel.data, channel.len)) return false;
    
    return true;
}

bool meshtastic_handler_encode_config_complete(const MeshtasticHandler* handler, uint32_t config_id, Vec_u8_MAX_MESSAGE_SIZE* out) {
    if (!handler || !out) return false;
    
    out->len = 0;
    
    // packet_id field (field 1, varint, value 0)
    if (!write_tag(1, WIRE_VARINT, out)) return false;
    if (!encode_varint(0, out)) return false;
    
    // config_complete_id field (field 9, varint)
    if (!write_tag(9, WIRE_VARINT, out)) return false;
    if (!encode_varint(config_id, out)) return false;
    
    return true;
}

bool meshtastic_handler_handle_admin_message(MeshtasticHandler* handler, const uint8_t* payload, size_t payload_len, Vec_u8_MAX_MESSAGE_SIZE* out) {
    if (!handler || !payload || !out || payload_len == 0) return false;
    
    uint8_t tag = payload[0];
    uint32_t field_num = tag >> 3;
    
    switch (field_num) {
        case 1: // get_owner_request
            return meshtastic_handler_encode_privacy_myinfo(handler, out);
            
        case 7: // get_device_metadata_request
            return meshtastic_handler_encode_privacy_nodeinfo(handler, out);
            
        case 5: // get_config_request
            // Not implemented in this translation
            return false;
            
        case 6: // get_channel_request
            // Not implemented in this translation
            return false;
            
        default:
            return false;
    }
}

// Channel functions (stub implementations - would be in channel.c)
void channel_init(Channel* channel) {
    if (!channel) return;
    memset(channel, 0, sizeof(Channel));
}

void channel_set_key(Channel* channel, const uint8_t* psk, size_t psk_len) {
    if (!channel || !psk) return;
    size_t copy_len = psk_len < 32 ? psk_len : 32;
    memcpy(channel->key, psk, copy_len);
}

bool channel_decrypt(const Channel* channel, uint32_t packet_id, uint32_t from, const Vec_u8_MAX_LORA_PAYLOAD* encrypted, Vec_u8_MAX_LORA_PAYLOAD* out) {
    // Stub implementation - actual decryption would be implemented in encryption.c
    if (!channel || !encrypted || !out) return false;
    memcpy(out, encrypted, sizeof(Vec_u8_MAX_LORA_PAYLOAD));
    return true;
}

bool channel_encrypt(const Channel* channel, uint32_t packet_id, uint32_t from, const Vec_u8_MAX_LORA_PAYLOAD* plaintext, Vec_u8_MAX_LORA_PAYLOAD* out) {
    // Stub implementation - actual encryption would be implemented in encryption.c
    if (!channel || !plaintext || !out) return false;
    memcpy(out, plaintext, sizeof(Vec_u8_MAX_LORA_PAYLOAD));
    return true;
}

const char* channel_name_str(const Channel* channel) {
    if (!channel) return "";
    return (const char*)channel->name;
}

// Packet functions (stub implementations - would be in packet.c)
MeshPacket* parse_lora_packet(const uint8_t* data, size_t len) {
    // Stub implementation - actual parsing would be implemented in packet.c
    if (!data || len < LORA_HEADER_SIZE + MIC_SIZE) return NULL;
    
    MeshPacket* packet = (MeshPacket*)malloc(sizeof(MeshPacket));
    if (!packet) return NULL;
    
    mesh_packet_init(packet);
    return packet;
}

bool build_lora_packet(uint32_t from, uint32_t to, uint32_t id, uint8_t channel, uint8_t hop_limit, bool want_ack, const Vec_u8_MAX_LORA_PAYLOAD* payload, Vec_u8_256* out) {
    // Stub implementation - actual building would be implemented in packet.c
    if (!payload || !out) return false;
    out->len = 0;
    return vec_u8_256_extend(out, payload->data, payload->len);
}

// Protobuf functions (stub implementations - would be in protobuf.c)
DataPayload* protobuf_decode_data(const Vec_u8_MAX_LORA_PAYLOAD* data) {
    // Stub implementation - actual decoding would be implemented in protobuf.c
    if (!data) return NULL;
    
    DataPayload* payload = (DataPayload*)malloc(sizeof(DataPayload));
    if (!payload) return NULL;
    
    data_payload_init(payload);
    return payload;
}

bool protobuf_encode_data(const DataPayload* data, Vec_u8_MAX_LORA_PAYLOAD* out) {
    // Stub implementation - actual encoding would be implemented in protobuf.c
    if (!data || !out) return false;
    out->len = 0;
    return vec_u8_max_lora_payload_extend(out, data->payload.data, data->payload.len);
}

bool protobuf_encode_position(const Position* position, Vec_u8_MAX_LORA_PAYLOAD* out) {
    // Stub implementation - actual encoding would be implemented in protobuf.c
    if (!position || !out) return false;
    out->len = 0;
    return true;
}

bool protobuf_encode_user(const User* user, Vec_u8_MAX_LORA_PAYLOAD* out) {
    // Stub implementation - actual encoding would be implemented in protobuf.c
    if (!user || !out) return false;
    out->len = 0;
    return true;
}
