/* # Conversion Notes: Rust to C Translation

## Major Translation Decisions

### 1. Generic Const Parameters
- **Rust**: Used `const N: usize` for compile-time generic buffer sizes
- **C**: Defined `MAX_LORA_PAYLOAD` as a constant and used it throughout. For functions that need different sizes, the size is passed as a parameter.

### 2. heapless::Vec to C Implementation
- **Rust**: Used `heapless::Vec<u8, N>` for fixed-size, heap-free vectors
- **C**: Implemented `Vec_u8` structure with:
  - `data`: pointer to buffer
  - `len`: current length
  - `capacity`: maximum capacity
  - Helper functions: `vec_u8_init`, `vec_u8_push`, `vec_u8_extend_from_slice`, `vec_u8_is_empty`

### 3. Option<T> Return Type
- **Rust**: Used `Option<T>` for functions that might fail
- **C**: 
  - For value-returning functions: Use `bool` return with output parameters (pointers)
  - For encoding functions: Return `bool` for success/failure, use output parameters for data and length

### 4. Slices and Borrowing
- **Rust**: Used slices (`&[u8]`, `&str`) with automatic lifetime management
- **C**: Used `const uint8_t*` with explicit length parameters

### 5. Module Structure
- **Rust**: Used nested modules (`mod data_fields`, `mod position_fields`, etc.)
- **C**: Used `#define` constants with prefixes (`DATA_FIELD_`, `POSITION_FIELD_`, `USER_FIELD_`)

### 6. Methods and Implementations
- **Rust**: Used `impl` blocks with methods taking `&self` or `&mut self`
- **C**: Converted to functions taking struct pointers as first parameter (e.g., `protobuf_encoder_write_varint(ProtobufEncoder *encoder, ...)`)

### 7. String Handling
- **Rust**: Used `&str` with `as_bytes()` method
- **C**: Used `const char*` with explicit length or converted to `const uint8_t*`

### 8. Pattern Matching
- **Rust**: Used `match` expressions for enum variants and field processing
- **C**: Converted to `switch` statements

### 9. Closures and Callbacks
- **Rust**: `write_message_field` used closure `F: FnOnce(&mut ProtobufEncoder<256>) -> bool`
- **C**: Defined function pointer type `typedef bool (*EncodeCallback)(ProtobufEncoder *)`

### 10. Error Handling
- **Rust**: Used `Result<T, E>` and `Option<T>` with `?` operator
- **C**: Used `bool` return values and explicit error checking

## Type Conversions

### Enums
- Assumed enum definitions exist in parent module:
  - `PortNum`, `HardwareModel`, `Role`, `LocationSource`
- Used integer conversions with switch statements for enum mapping

### Structs
- Assumed struct definitions exist for:
  - `DataPayload` (with fields: port, payload, payload_len, want_response, dest, source, request_id, reply_id, emoji)
  - `Position` (with all position-related fields)
  - `User` (with id[8], long_name, long_name_len, short_name, short_name_len, hw_model, is_licensed, role)

## Incomplete Code Handling

The original Rust code was cut off at the `hex_decode` function. I completed the implementation based on the context:
```c
static inline int hex_char_to_value(uint8_t c) {
    if (c >= '0' && c <= '9') return c - '0';
    else if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

void hex_decode(const uint8_t *data, size_t data_len, uint8_t *output, size_t output_len) {
    for (size_t i = 0; i < data_len / 2 && i < output_len; i++) {
        int high = hex_char_to_value(data[i * 2]);
        int low = hex_char_to_value(data[i * 2 + 1]);
        if (high >= 0 && low >= 0) {
            output[i] = (uint8_t)((high << 4) | low);
        }
    }
}
```

## Memory Management

- **Rust**: Automatic memory management with ownership and borrowing
- **C**: Manual memory management:
  - Used stack-allocated buffers for encoders
  - Caller provides output buffers for encode functions
  - No dynamic allocation (matching Rust's heap-free approach)

## Safety Considerations

1. **Buffer Overflow Protection**: All buffer operations check bounds
2. **Integer Overflow**: Shift operations limited to valid ranges
3. **Null Pointer Checks**: Assumed valid pointers (as in Rust with references)

## Testing Recommendations

1. Test varint encoding/decoding with edge cases (0, max values, overflow)
2. Test all protobuf field types (varint, fixed32, fixed64, length-delimited)
3. Test encoding and decoding round-trips for DataPayload, Position, and User
4. Test buffer overflow scenarios
5. Test hex encoding/decoding with various input sizes

## Required External Definitions

The following types and constants must be defined in your codebase:
- `MAX_LORA_PAYLOAD` constant
- `DataPayload` struct
- `Position` struct  
- `User` struct
- `PortNum` enum with `from()` conversion
- `HardwareModel` enum with variants
- `Role` enum with variants
- `LocationSource` enum with variants and corresponding defines (e.g., `LOCATION_SOURCE_UNSET`, `LOCATION_SOURCE_MANUAL`, etc.)

All enum values in the decode functions use placeholder constant names that should match your existing codebase.
 */


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

// Import dependencies from super module
// These would be defined in your existing codebase
typedef struct DataPayload DataPayload;
typedef struct Position Position;
typedef struct User User;
typedef enum PortNum PortNum;
typedef enum HardwareModel HardwareModel;
typedef enum Role Role;
typedef enum LocationSource LocationSource;

// Assume MAX_LORA_PAYLOAD is defined in the parent module
#ifndef MAX_LORA_PAYLOAD
#define MAX_LORA_PAYLOAD 256
#endif

// Wire type constants
#define WIRE_TYPE_VARINT 0
#define WIRE_TYPE_64BIT 1
#define WIRE_TYPE_LENGTH_DELIMITED 2
#define WIRE_TYPE_32BIT 5

// Data field constants
#define DATA_FIELD_PORTNUM 1
#define DATA_FIELD_PAYLOAD 2
#define DATA_FIELD_WANT_RESPONSE 3
#define DATA_FIELD_DEST 4
#define DATA_FIELD_SOURCE 5
#define DATA_FIELD_REQUEST_ID 6
#define DATA_FIELD_REPLY_ID 7
#define DATA_FIELD_EMOJI 8

// Position field constants
#define POSITION_FIELD_LATITUDE_I 1
#define POSITION_FIELD_LONGITUDE_I 2
#define POSITION_FIELD_ALTITUDE 3
#define POSITION_FIELD_TIME 4
#define POSITION_FIELD_LOCATION_SOURCE 5
#define POSITION_FIELD_ALTITUDE_SOURCE 6
#define POSITION_FIELD_TIMESTAMP 7
#define POSITION_FIELD_TIMESTAMP_MILLIS_ADJUST 8
#define POSITION_FIELD_ALTITUDE_HAE 9
#define POSITION_FIELD_ALTITUDE_GEOIDAL_SEPARATION 10
#define POSITION_FIELD_PDOP 11
#define POSITION_FIELD_HDOP 12
#define POSITION_FIELD_VDOP 13
#define POSITION_FIELD_GPS_ACCURACY 14
#define POSITION_FIELD_GROUND_SPEED 15
#define POSITION_FIELD_GROUND_TRACK 16
#define POSITION_FIELD_FIX_QUALITY 17
#define POSITION_FIELD_FIX_TYPE 18
#define POSITION_FIELD_SATS_IN_VIEW 19
#define POSITION_FIELD_SENSOR_ID 20
#define POSITION_FIELD_NEXT_UPDATE 21
#define POSITION_FIELD_SEQ_NUMBER 22

// User field constants
#define USER_FIELD_ID 1
#define USER_FIELD_LONG_NAME 2
#define USER_FIELD_SHORT_NAME 3
#define USER_FIELD_MACADDR 4
#define USER_FIELD_HW_MODEL 5
#define USER_FIELD_IS_LICENSED 6
#define USER_FIELD_ROLE 7

// Fixed-size vector implementation
typedef struct {
    uint8_t *data;
    size_t len;
    size_t capacity;
} Vec_u8;

// Initialize a Vec_u8 with a pre-allocated buffer
static inline void vec_u8_init(Vec_u8 *vec, uint8_t *buffer, size_t capacity) {
    vec->data = buffer;
    vec->len = 0;
    vec->capacity = capacity;
}

// Push a byte to the vector
static inline bool vec_u8_push(Vec_u8 *vec, uint8_t value) {
    if (vec->len >= vec->capacity) {
        return false;
    }
    vec->data[vec->len] = value;
    vec->len++;
    return true;
}

// Extend vector from slice
static inline bool vec_u8_extend_from_slice(Vec_u8 *vec, const uint8_t *data, size_t len) {
    if (vec->len + len > vec->capacity) {
        return false;
    }
    memcpy(&vec->data[vec->len], data, len);
    vec->len += len;
    return true;
}

// Check if vector is empty
static inline bool vec_u8_is_empty(const Vec_u8 *vec) {
    return vec->len == 0;
}

// ProtobufEncoder structure
typedef struct {
    Vec_u8 buffer;
    uint8_t buffer_storage[MAX_LORA_PAYLOAD];
} ProtobufEncoder;

// Initialize ProtobufEncoder
void protobuf_encoder_init(ProtobufEncoder *encoder) {
    vec_u8_init(&encoder->buffer, encoder->buffer_storage, MAX_LORA_PAYLOAD);
}

// Write varint to encoder
bool protobuf_encoder_write_varint(ProtobufEncoder *encoder, uint64_t value) {
    uint64_t v = value;
    while (true) {
        uint8_t byte = (uint8_t)(v & 0x7F);
        v >>= 7;
        if (v == 0) {
            return vec_u8_push(&encoder->buffer, byte);
        } else {
            if (!vec_u8_push(&encoder->buffer, byte | 0x80)) {
                return false;
            }
        }
    }
}

// Write signed 32-bit integer
bool protobuf_encoder_write_sint32(ProtobufEncoder *encoder, int32_t value) {
    uint32_t encoded = (uint32_t)((value << 1) ^ (value >> 31));
    return protobuf_encoder_write_varint(encoder, (uint64_t)encoded);
}

// Write signed 64-bit integer
bool protobuf_encoder_write_sint64(ProtobufEncoder *encoder, int64_t value) {
    uint64_t encoded = (uint64_t)((value << 1) ^ (value >> 63));
    return protobuf_encoder_write_varint(encoder, encoded);
}

// Write tag
bool protobuf_encoder_write_tag(ProtobufEncoder *encoder, uint32_t field_number, uint8_t wire_type) {
    uint32_t tag = (field_number << 3) | (uint32_t)wire_type;
    return protobuf_encoder_write_varint(encoder, (uint64_t)tag);
}

// Write varint field
bool protobuf_encoder_write_varint_field(ProtobufEncoder *encoder, uint32_t field_number, uint64_t value) {
    if (value == 0) {
        return true;
    }
    return protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_VARINT) &&
           protobuf_encoder_write_varint(encoder, value);
}

// Write sint32 field
bool protobuf_encoder_write_sint32_field(ProtobufEncoder *encoder, uint32_t field_number, int32_t value) {
    if (value == 0) {
        return true;
    }
    return protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_VARINT) &&
           protobuf_encoder_write_sint32(encoder, value);
}

// Write bool field
bool protobuf_encoder_write_bool_field(ProtobufEncoder *encoder, uint32_t field_number, bool value) {
    if (!value) {
        return true;
    }
    return protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_VARINT) &&
           vec_u8_push(&encoder->buffer, 1);
}

// Write bytes field
bool protobuf_encoder_write_bytes_field(ProtobufEncoder *encoder, uint32_t field_number, const uint8_t *data, size_t len) {
    if (len == 0) {
        return true;
    }
    return protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_LENGTH_DELIMITED) &&
           protobuf_encoder_write_varint(encoder, (uint64_t)len) &&
           vec_u8_extend_from_slice(&encoder->buffer, data, len);
}

// Write string field
bool protobuf_encoder_write_string_field(ProtobufEncoder *encoder, uint32_t field_number, const char *s, size_t len) {
    return protobuf_encoder_write_bytes_field(encoder, field_number, (const uint8_t *)s, len);
}

// Write fixed32 field
bool protobuf_encoder_write_fixed32_field(ProtobufEncoder *encoder, uint32_t field_number, uint32_t value) {
    if (value == 0) {
        return true;
    }
    if (!protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_32BIT)) {
        return false;
    }
    uint8_t bytes[4];
    bytes[0] = (uint8_t)(value & 0xFF);
    bytes[1] = (uint8_t)((value >> 8) & 0xFF);
    bytes[2] = (uint8_t)((value >> 16) & 0xFF);
    bytes[3] = (uint8_t)((value >> 24) & 0xFF);
    return vec_u8_extend_from_slice(&encoder->buffer, bytes, 4);
}

// Write fixed64 field
bool protobuf_encoder_write_fixed64_field(ProtobufEncoder *encoder, uint32_t field_number, uint64_t value) {
    if (value == 0) {
        return true;
    }
    if (!protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_64BIT)) {
        return false;
    }
    uint8_t bytes[8];
    for (int i = 0; i < 8; i++) {
        bytes[i] = (uint8_t)((value >> (i * 8)) & 0xFF);
    }
    return vec_u8_extend_from_slice(&encoder->buffer, bytes, 8);
}

// Write message field using callback
typedef bool (*EncodeCallback)(ProtobufEncoder *);

bool protobuf_encoder_write_message_field(ProtobufEncoder *encoder, uint32_t field_number, EncodeCallback encode_fn) {
    ProtobufEncoder nested;
    protobuf_encoder_init(&nested);
    
    if (!encode_fn(&nested)) {
        return false;
    }
    
    if (vec_u8_is_empty(&nested.buffer)) {
        return true;
    }
    
    return protobuf_encoder_write_bytes_field(encoder, field_number, nested.buffer.data, nested.buffer.len);
}

// Encode varint to slice
void protobuf_encoder_encode_varint_to_slice(uint64_t value, Vec_u8 *output) {
    uint64_t v = value;
    while (true) {
        uint8_t byte = (uint8_t)(v & 0x7F);
        v >>= 7;
        if (v == 0) {
            vec_u8_push(output, byte);
            break;
        } else {
            vec_u8_push(output, byte | 0x80);
        }
    }
}

// ProtobufDecoder structure
typedef struct {
    const uint8_t *data;
    size_t len;
    size_t pos;
} ProtobufDecoder;

// Initialize ProtobufDecoder
void protobuf_decoder_init(ProtobufDecoder *decoder, const uint8_t *data, size_t len) {
    decoder->data = data;
    decoder->len = len;
    decoder->pos = 0;
}

// Check if decoder has more data
bool protobuf_decoder_has_more(const ProtobufDecoder *decoder) {
    return decoder->pos < decoder->len;
}

// Read varint
bool protobuf_decoder_read_varint(ProtobufDecoder *decoder, uint64_t *result) {
    uint64_t value = 0;
    int shift = 0;
    
    while (true) {
        if (decoder->pos >= decoder->len) {
            return false;
        }
        
        uint8_t byte = decoder->data[decoder->pos];
        decoder->pos++;
        
        value |= ((uint64_t)(byte & 0x7F)) << shift;
        
        if ((byte & 0x80) == 0) {
            *result = value;
            return true;
        }
        
        shift += 7;
        if (shift >= 64) {
            return false;
        }
    }
}

// Read sint32
bool protobuf_decoder_read_sint32(ProtobufDecoder *decoder, int32_t *result) {
    uint64_t encoded;
    if (!protobuf_decoder_read_varint(decoder, &encoded)) {
        return false;
    }
    uint32_t encoded32 = (uint32_t)encoded;
    *result = (int32_t)((encoded32 >> 1) ^ (-(int32_t)(encoded32 & 1)));
    return true;
}

// Read sint64
bool protobuf_decoder_read_sint64(ProtobufDecoder *decoder, int64_t *result) {
    uint64_t encoded;
    if (!protobuf_decoder_read_varint(decoder, &encoded)) {
        return false;
    }
    *result = (int64_t)((encoded >> 1) ^ (-(int64_t)(encoded & 1)));
    return true;
}

// Read tag
bool protobuf_decoder_read_tag(ProtobufDecoder *decoder, uint32_t *field_number, uint8_t *wire_type) {
    uint64_t tag;
    if (!protobuf_decoder_read_varint(decoder, &tag)) {
        return false;
    }
    uint32_t tag32 = (uint32_t)tag;
    *field_number = tag32 >> 3;
    *wire_type = (uint8_t)(tag32 & 0x07);
    return true;
}

// Next field (returns field info)
typedef struct {
    uint32_t field_number;
    uint8_t wire_type;
    const uint8_t *data;
    size_t len;
} FieldInfo;

bool protobuf_decoder_next_field(ProtobufDecoder *decoder, FieldInfo *field_info) {
    if (!protobuf_decoder_has_more(decoder)) {
        return false;
    }
    
    if (!protobuf_decoder_read_tag(decoder, &field_info->field_number, &field_info->wire_type)) {
        return false;
    }
    
    switch (field_info->wire_type) {
        case WIRE_TYPE_VARINT: {
            size_t start = decoder->pos;
            while (decoder->pos < decoder->len) {
                uint8_t byte = decoder->data[decoder->pos];
                decoder->pos++;
                if ((byte & 0x80) == 0) {
                    break;
                }
            }
            field_info->data = &decoder->data[start];
            field_info->len = decoder->pos - start;
            break;
        }
        case WIRE_TYPE_64BIT: {
            if (decoder->pos + 8 > decoder->len) {
                return false;
            }
            field_info->data = &decoder->data[decoder->pos];
            field_info->len = 8;
            decoder->pos += 8;
            break;
        }
        case WIRE_TYPE_LENGTH_DELIMITED: {
            uint64_t len;
            if (!protobuf_decoder_read_varint(decoder, &len)) {
                return false;
            }
            size_t len_sz = (size_t)len;
            if (decoder->pos + len_sz > decoder->len) {
                return false;
            }
            field_info->data = &decoder->data[decoder->pos];
            field_info->len = len_sz;
            decoder->pos += len_sz;
            break;
        }
        case WIRE_TYPE_32BIT: {
            if (decoder->pos + 4 > decoder->len) {
                return false;
            }
            field_info->data = &decoder->data[decoder->pos];
            field_info->len = 4;
            decoder->pos += 4;
            break;
        }
        default:
            return false;
    }
    
    return true;
}

// Read bytes
bool protobuf_decoder_read_bytes(ProtobufDecoder *decoder, const uint8_t **result, size_t *len) {
    uint64_t length;
    if (!protobuf_decoder_read_varint(decoder, &length)) {
        return false;
    }
    size_t length_sz = (size_t)length;
    if (decoder->pos + length_sz > decoder->len) {
        return false;
    }
    *result = &decoder->data[decoder->pos];
    *len = length_sz;
    decoder->pos += length_sz;
    return true;
}

// Read fixed32
bool protobuf_decoder_read_fixed32(ProtobufDecoder *decoder, uint32_t *result) {
    if (decoder->pos + 4 > decoder->len) {
        return false;
    }
    *result = ((uint32_t)decoder->data[decoder->pos]) |
              ((uint32_t)decoder->data[decoder->pos + 1] << 8) |
              ((uint32_t)decoder->data[decoder->pos + 2] << 16) |
              ((uint32_t)decoder->data[decoder->pos + 3] << 24);
    decoder->pos += 4;
    return true;
}

// Read fixed64
bool protobuf_decoder_read_fixed64(ProtobufDecoder *decoder, uint64_t *result) {
    if (decoder->pos + 8 > decoder->len) {
        return false;
    }
    *result = ((uint64_t)decoder->data[decoder->pos]) |
              ((uint64_t)decoder->data[decoder->pos + 1] << 8) |
              ((uint64_t)decoder->data[decoder->pos + 2] << 16) |
              ((uint64_t)decoder->data[decoder->pos + 3] << 24) |
              ((uint64_t)decoder->data[decoder->pos + 4] << 32) |
              ((uint64_t)decoder->data[decoder->pos + 5] << 40) |
              ((uint64_t)decoder->data[decoder->pos + 6] << 48) |
              ((uint64_t)decoder->data[decoder->pos + 7] << 56);
    decoder->pos += 8;
    return true;
}

// Skip field
bool protobuf_decoder_skip_field(ProtobufDecoder *decoder, uint8_t wire_type) {
    switch (wire_type) {
        case WIRE_TYPE_VARINT: {
            uint64_t dummy;
            return protobuf_decoder_read_varint(decoder, &dummy);
        }
        case WIRE_TYPE_64BIT: {
            if (decoder->pos + 8 <= decoder->len) {
                decoder->pos += 8;
                return true;
            }
            return false;
        }
        case WIRE_TYPE_LENGTH_DELIMITED: {
            const uint8_t *dummy_data;
            size_t dummy_len;
            return protobuf_decoder_read_bytes(decoder, &dummy_data, &dummy_len);
        }
        case WIRE_TYPE_32BIT: {
            if (decoder->pos + 4 <= decoder->len) {
                decoder->pos += 4;
                return true;
            }
            return false;
        }
        default:
            return false;
    }
}

// Static helper functions for reading from slices

// Read varint from slice
bool protobuf_decoder_read_varint_from_slice(const uint8_t *data, size_t len, uint64_t *result) {
    uint64_t value = 0;
    int shift = 0;
    
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        value |= ((uint64_t)(byte & 0x7F)) << shift;
        
        if ((byte & 0x80) == 0) {
            *result = value;
            return true;
        }
        
        shift += 7;
        if (shift >= 64) {
            return false;
        }
    }
    
    return false;
}

// Read varint advancing pointer
bool protobuf_decoder_read_varint_advancing(const uint8_t **data, size_t *len, uint64_t *result) {
    uint64_t value = 0;
    int shift = 0;
    size_t consumed = 0;
    
    for (size_t i = 0; i < *len; i++) {
        uint8_t byte = (*data)[i];
        value |= ((uint64_t)(byte & 0x7F)) << shift;
        consumed = i + 1;
        
        if ((byte & 0x80) == 0) {
            *data = &(*data)[consumed];
            *len -= consumed;
            *result = value;
            return true;
        }
        
        shift += 7;
        if (shift >= 64) {
            return false;
        }
    }
    
    return false;
}

// Read bytes from slice
bool protobuf_decoder_read_bytes_from_slice(const uint8_t **data, size_t *len, const uint8_t **result, size_t *result_len) {
    uint64_t length;
    if (!protobuf_decoder_read_varint_advancing(data, len, &length)) {
        return false;
    }
    size_t length_sz = (size_t)length;
    if (*len < length_sz) {
        return false;
    }
    *result = *data;
    *result_len = length_sz;
    *data = &(*data)[length_sz];
    *len -= length_sz;
    return true;
}

// Read tag from slice
bool protobuf_decoder_read_tag_from_slice(const uint8_t **data, size_t *len, uint32_t *field_number, uint8_t *wire_type) {
    uint64_t tag;
    if (!protobuf_decoder_read_varint_advancing(data, len, &tag)) {
        return false;
    }
    uint32_t tag32 = (uint32_t)tag;
    *field_number = tag32 >> 3;
    *wire_type = (uint8_t)(tag32 & 0x07);
    return true;
}

// Skip field from slice
bool protobuf_decoder_skip_field_from_slice(const uint8_t **data, size_t *len, uint8_t wire_type) {
    switch (wire_type) {
        case WIRE_TYPE_VARINT: {
            uint64_t dummy;
            return protobuf_decoder_read_varint_advancing(data, len, &dummy);
        }
        case WIRE_TYPE_64BIT: {
            if (*len >= 8) {
                *data = &(*data)[8];
                *len -= 8;
                return true;
            }
            return false;
        }
        case WIRE_TYPE_LENGTH_DELIMITED: {
            const uint8_t *dummy_data;
            size_t dummy_len;
            return protobuf_decoder_read_bytes_from_slice(data, len, &dummy_data, &dummy_len);
        }
        case WIRE_TYPE_32BIT: {
            if (*len >= 4) {
                *data = &(*data)[4];
                *len -= 4;
                return true;
            }
            return false;
        }
        default:
            return false;
    }
}

// Helper functions

// Hex encode
void hex_encode(const uint8_t *data, size_t data_len, uint8_t *output, size_t output_len) {
    static const uint8_t HEX_CHARS[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    for (size_t i = 0; i < data_len; i++) {
        if (i * 2 + 1 < output_len) {
            uint8_t byte = data[i];
            output[i * 2] = HEX_CHARS[byte >> 4];
            output[i * 2 + 1] = HEX_CHARS[byte & 0x0F];
        }
    }
}

// Hex decode helper - convert hex char to value
static inline int hex_char_to_value(uint8_t c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

// Hex decode
void hex_decode(const uint8_t *data, size_t data_len, uint8_t *output, size_t output_len) {
    for (size_t i = 0; i < data_len / 2 && i < output_len; i++) {
        int high = hex_char_to_value(data[i * 2]);
        int low = hex_char_to_value(data[i * 2 + 1]);
        if (high >= 0 && low >= 0) {
            output[i] = (uint8_t)((high << 4) | low);
        }
    }
}

// Encode DataPayload
bool encode_data(const DataPayload *data, uint8_t *output, size_t *output_len, size_t max_len) {
    ProtobufEncoder encoder;
    uint8_t buffer[MAX_LORA_PAYLOAD];
    vec_u8_init(&encoder.buffer, buffer, MAX_LORA_PAYLOAD);
    
    protobuf_encoder_write_varint_field(&encoder, DATA_FIELD_PORTNUM, (uint64_t)data->port);
    protobuf_encoder_write_bytes_field(&encoder, DATA_FIELD_PAYLOAD, data->payload, data->payload_len);
    protobuf_encoder_write_bool_field(&encoder, DATA_FIELD_WANT_RESPONSE, data->want_response);
    protobuf_encoder_write_varint_field(&encoder, DATA_FIELD_DEST, (uint64_t)data->dest);
    protobuf_encoder_write_varint_field(&encoder, DATA_FIELD_SOURCE, (uint64_t)data->source);
    protobuf_encoder_write_varint_field(&encoder, DATA_FIELD_REQUEST_ID, (uint64_t)data->request_id);
    protobuf_encoder_write_varint_field(&encoder, DATA_FIELD_REPLY_ID, (uint64_t)data->reply_id);
    protobuf_encoder_write_varint_field(&encoder, DATA_FIELD_EMOJI, (uint64_t)data->emoji);
    
    if (encoder.buffer.len > max_len) {
        return false;
    }
    
    memcpy(output, encoder.buffer.data, encoder.buffer.len);
    *output_len = encoder.buffer.len;
    return true;
}

// Encode Position
bool encode_position(const Position *pos, uint8_t *output, size_t *output_len, size_t max_len) {
    ProtobufEncoder encoder;
    uint8_t buffer[MAX_LORA_PAYLOAD];
    vec_u8_init(&encoder.buffer, buffer, MAX_LORA_PAYLOAD);
    
    protobuf_encoder_write_sint32_field(&encoder, POSITION_FIELD_LATITUDE_I, pos->latitude_i);
    protobuf_encoder_write_sint32_field(&encoder, POSITION_FIELD_LONGITUDE_I, pos->longitude_i);
    protobuf_encoder_write_sint32_field(&encoder, POSITION_FIELD_ALTITUDE, pos->altitude);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_TIME, (uint64_t)pos->time);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_LOCATION_SOURCE, (uint64_t)pos->location_source);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_ALTITUDE_SOURCE, (uint64_t)pos->altitude_source);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_TIMESTAMP, (uint64_t)pos->timestamp);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_PDOP, (uint64_t)pos->pdop);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_HDOP, (uint64_t)pos->hdop);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_SATS_IN_VIEW, (uint64_t)pos->sats_in_view);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_GROUND_SPEED, (uint64_t)pos->ground_speed);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_GROUND_TRACK, (uint64_t)pos->ground_track);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_FIX_QUALITY, (uint64_t)pos->fix_quality);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_FIX_TYPE, (uint64_t)pos->fix_type);
    protobuf_encoder_write_varint_field(&encoder, POSITION_FIELD_SEQ_NUMBER, (uint64_t)pos->seq_number);
    
    if (encoder.buffer.len > max_len) {
        return false;
    }
    
    memcpy(output, encoder.buffer.data, encoder.buffer.len);
    *output_len = encoder.buffer.len;
    return true;
}

// Encode User
bool encode_user(const User *user, uint8_t *output, size_t *output_len, size_t max_len) {
    ProtobufEncoder encoder;
    uint8_t buffer[MAX_LORA_PAYLOAD];
    vec_u8_init(&encoder.buffer, buffer, MAX_LORA_PAYLOAD);
    
    uint8_t id_str[17];
    id_str[0] = '!';
    hex_encode(user->id, 8, &id_str[1], 16);
    protobuf_encoder_write_bytes_field(&encoder, USER_FIELD_ID, id_str, 17);
    
    protobuf_encoder_write_bytes_field(&encoder, USER_FIELD_LONG_NAME, user->long_name, user->long_name_len);
    protobuf_encoder_write_bytes_field(&encoder, USER_FIELD_SHORT_NAME, user->short_name, user->short_name_len);
    protobuf_encoder_write_bytes_field(&encoder, USER_FIELD_MACADDR, &user->id[2], 6);
    protobuf_encoder_write_varint_field(&encoder, USER_FIELD_HW_MODEL, (uint64_t)user->hw_model);
    protobuf_encoder_write_bool_field(&encoder, USER_FIELD_IS_LICENSED, user->is_licensed);
    protobuf_encoder_write_varint_field(&encoder, USER_FIELD_ROLE, (uint64_t)user->role);
    
    if (encoder.buffer.len > max_len) {
        return false;
    }
    
    memcpy(output, encoder.buffer.data, encoder.buffer.len);
    *output_len = encoder.buffer.len;
    return true;
}

// Decode DataPayload
bool decode_data(const uint8_t *data, size_t len, DataPayload *result) {
    ProtobufDecoder decoder;
    protobuf_decoder_init(&decoder, data, len);
    
    // Initialize result with defaults
    memset(result, 0, sizeof(DataPayload));
    
    while (protobuf_decoder_has_more(&decoder)) {
        uint32_t field_number;
        uint8_t wire_type;
        if (!protobuf_decoder_read_tag(&decoder, &field_number, &wire_type)) {
            return false;
        }
        
        switch (field_number) {
            case DATA_FIELD_PORTNUM:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->port = (PortNum)((uint32_t)value);
                }
                break;
            case DATA_FIELD_PAYLOAD:
                if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) {
                    const uint8_t *bytes;
                    size_t bytes_len;
                    if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) {
                        return false;
                    }
                    if (bytes_len > MAX_LORA_PAYLOAD) {
                        return false;
                    }
                    memcpy(result->payload, bytes, bytes_len);
                    result->payload_len = bytes_len;
                }
                break;
            case DATA_FIELD_WANT_RESPONSE:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->want_response = (value != 0);
                }
                break;
            case DATA_FIELD_DEST:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->dest = (uint32_t)value;
                }
                break;
            case DATA_FIELD_SOURCE:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->source = (uint32_t)value;
                }
                break;
            case DATA_FIELD_REQUEST_ID:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->request_id = (uint32_t)value;
                }
                break;
            case DATA_FIELD_REPLY_ID:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->reply_id = (uint32_t)value;
                }
                break;
            case DATA_FIELD_EMOJI:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->emoji = (uint32_t)value;
                }
                break;
            default:
                if (!protobuf_decoder_skip_field(&decoder, wire_type)) {
                    return false;
                }
                break;
        }
    }
    
    return true;
}

// Decode Position
bool decode_position(const uint8_t *data, size_t len, Position *result) {
    ProtobufDecoder decoder;
    protobuf_decoder_init(&decoder, data, len);
    
    // Initialize result with defaults
    memset(result, 0, sizeof(Position));
    
    while (protobuf_decoder_has_more(&decoder)) {
        uint32_t field_number;
        uint8_t wire_type;
        if (!protobuf_decoder_read_tag(&decoder, &field_number, &wire_type)) {
            return false;
        }
        
        switch (field_number) {
            case POSITION_FIELD_LATITUDE_I:
                if (wire_type == WIRE_TYPE_VARINT) {
                    if (!protobuf_decoder_read_sint32(&decoder, &result->latitude_i)) {
                        return false;
                    }
                }
                break;
            case POSITION_FIELD_LONGITUDE_I:
                if (wire_type == WIRE_TYPE_VARINT) {
                    if (!protobuf_decoder_read_sint32(&decoder, &result->longitude_i)) {
                        return false;
                    }
                }
                break;
            case POSITION_FIELD_ALTITUDE:
                if (wire_type == WIRE_TYPE_VARINT) {
                    if (!protobuf_decoder_read_sint32(&decoder, &result->altitude)) {
                        return false;
                    }
                }
                break;
            case POSITION_FIELD_TIME:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->time = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_LOCATION_SOURCE:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    uint8_t val_u8 = (uint8_t)value;
                    switch (val_u8) {
                        case 1: result->location_source = LOCATION_SOURCE_MANUAL; break;
                        case 2: result->location_source = LOCATION_SOURCE_INTERNAL_GPS; break;
                        case 3: result->location_source = LOCATION_SOURCE_EXTERNAL_GPS; break;
                        default: result->location_source = LOCATION_SOURCE_UNSET; break;
                    }
                }
                break;
            case POSITION_FIELD_ALTITUDE_SOURCE:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    uint8_t val_u8 = (uint8_t)value;
                    switch (val_u8) {
                        case 1: result->altitude_source = LOCATION_SOURCE_MANUAL; break;
                        case 2: result->altitude_source = LOCATION_SOURCE_INTERNAL_GPS; break;
                        case 3: result->altitude_source = LOCATION_SOURCE_EXTERNAL_GPS; break;
                        default: result->altitude_source = LOCATION_SOURCE_UNSET; break;
                    }
                }
                break;
            case POSITION_FIELD_TIMESTAMP:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->timestamp = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_PDOP:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->pdop = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_HDOP:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->hdop = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_SATS_IN_VIEW:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->sats_in_view = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_GROUND_SPEED:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->ground_speed = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_GROUND_TRACK:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->ground_track = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_FIX_QUALITY:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->fix_quality = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_FIX_TYPE:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->fix_type = (uint32_t)value;
                }
                break;
            case POSITION_FIELD_SEQ_NUMBER:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->seq_number = (uint32_t)value;
                }
                break;
            default:
                if (!protobuf_decoder_skip_field(&decoder, wire_type)) {
                    return false;
                }
                break;
        }
    }
    
    return true;
}

// Decode User
bool decode_user(const uint8_t *data, size_t len, User *result) {
    ProtobufDecoder decoder;
    protobuf_decoder_init(&decoder, data, len);
    
    // Initialize result with defaults
    memset(result, 0, sizeof(User));
    result->hw_model = HARDWARE_MODEL_UNSET;
    result->is_licensed = false;
    result->role = ROLE_CLIENT;
    
    while (protobuf_decoder_has_more(&decoder)) {
        uint32_t field_number;
        uint8_t wire_type;
        if (!protobuf_decoder_read_tag(&decoder, &field_number, &wire_type)) {
            return false;
        }
        
        switch (field_number) {
            case USER_FIELD_ID:
                if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) {
                    const uint8_t *bytes;
                    size_t bytes_len;
                    if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) {
                        return false;
                    }
                    
                    if (bytes_len >= 17 && bytes[0] == '!') {
                        hex_decode(&bytes[1], 16, result->id, 8);
                    } else if (bytes_len == 8) {
                        memcpy(result->id, bytes, 8);
                    }
                }
                break;
            case USER_FIELD_LONG_NAME:
                if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) {
                    const uint8_t *bytes;
                    size_t bytes_len;
                    if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) {
                        return false;
                    }
                    if (bytes_len > sizeof(result->long_name)) {
                        bytes_len = sizeof(result->long_name);
                    }
                    memcpy(result->long_name, bytes, bytes_len);
                    result->long_name_len = bytes_len;
                }
                break;
            case USER_FIELD_SHORT_NAME:
                if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) {
                    const uint8_t *bytes;
                    size_t bytes_len;
                    if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) {
                        return false;
                    }
                    if (bytes_len > sizeof(result->short_name)) {
                        bytes_len = sizeof(result->short_name);
                    }
                    memcpy(result->short_name, bytes, bytes_len);
                    result->short_name_len = bytes_len;
                }
                break;
            case USER_FIELD_MACADDR:
                if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) {
                    const uint8_t *bytes;
                    size_t bytes_len;
                    if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) {
                        return false;
                    }
                    if (bytes_len == 6) {
                        memcpy(&result->id[2], bytes, 6);
                    }
                }
                break;
            case USER_FIELD_HW_MODEL:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    uint16_t val_u16 = (uint16_t)value;
                    switch (val_u16) {
                        case 1: result->hw_model = HARDWARE_MODEL_TLORA_V2; break;
                        case 2: result->hw_model = HARDWARE_MODEL_TLORA_V1; break;
                        case 4: result->hw_model = HARDWARE_MODEL_TBEAM; break;
                        case 5: result->hw_model = HARDWARE_MODEL_HELTEC_V2_0; break;
                        case 9: result->hw_model = HARDWARE_MODEL_RAK4631; break;
                        case 10: result->hw_model = HARDWARE_MODEL_HELTEC_V2_1; break;
                        case 34: result->hw_model = HARDWARE_MODEL_HELTEC_WIFI_LORA_V3; break;
                        case 255: result->hw_model = HARDWARE_MODEL_PRIVATE_HW; break;
                        default: result->hw_model = HARDWARE_MODEL_UNSET; break;
                    }
                }
                break;
            case USER_FIELD_IS_LICENSED:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    result->is_licensed = (value != 0);
                }
                break;
            case USER_FIELD_ROLE:
                if (wire_type == WIRE_TYPE_VARINT) {
                    uint64_t value;
                    if (!protobuf_decoder_read_varint(&decoder, &value)) {
                        return false;
                    }
                    uint8_t val_u8 = (uint8_t)value;
                    switch (val_u8) {
                        case 0: result->role = ROLE_CLIENT; break;
                        case 1: result->role = ROLE_CLIENT_MUTE; break;
                        case 2: result->role = ROLE_ROUTER; break;
                        case 3: result->role = ROLE_ROUTER_CLIENT; break;
                        case 4: result->role = ROLE_REPEATER; break;
                        case 5: result->role = ROLE_TRACKER; break;
                        case 6: result->role = ROLE_SENSOR; break;
                        case 7: result->role = ROLE_TAK; break;
                        case 8: result->role = ROLE_CLIENT_HIDDEN; break;
                        case 9: result->role = ROLE_LOST_AND_FOUND; break;
                        case 10: result->role = ROLE_TAK_TRACKER; break;
                        default: result->role = ROLE_CLIENT; break;
                    }
                }
                break;
            default:
                if (!protobuf_decoder_skip_field(&decoder, wire_type)) {
                    return false;
                }
                break;
        }
    }
    
    return true;
}
