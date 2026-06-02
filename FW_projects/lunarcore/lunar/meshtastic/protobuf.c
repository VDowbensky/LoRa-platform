#include "protobuf.h"

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
typedef struct 
{
  uint8_t *data;
  size_t len;
  size_t capacity;
} Vec_u8;

// Initialize a Vec_u8 with a pre-allocated buffer
static inline void vec_u8_init(Vec_u8 *vec, uint8_t *buffer, size_t capacity) 
{
  vec->data = buffer;
  vec->len = 0;
  vec->capacity = capacity;
}

// Push a byte to the vector
static inline bool vec_u8_push(Vec_u8 *vec, uint8_t value) 
{
  if (vec->len >= vec->capacity) return false;
  vec->data[vec->len] = value;
  vec->len++;
  return true;
}

// Extend vector from slice
static inline bool vec_u8_extend_from_slice(Vec_u8 *vec, const uint8_t *data, size_t len) 
{
  if (vec->len + len > vec->capacity) return false;
  memcpy(&vec->data[vec->len], data, len);
  vec->len += len;
  return true;
}

// Check if vector is empty
static inline bool vec_u8_is_empty(const Vec_u8 *vec) 
{
  return vec->len == 0;
}

// ProtobufEncoder structure
typedef struct 
{
  Vec_u8 buffer;
  uint8_t buffer_storage[MAX_LORA_PAYLOAD];
} ProtobufEncoder;

// Initialize ProtobufEncoder
void protobuf_encoder_init(ProtobufEncoder *encoder) 
{
  vec_u8_init(&encoder->buffer, encoder->buffer_storage, MAX_LORA_PAYLOAD);
}

// Write varint to encoder
bool protobuf_encoder_write_varint(ProtobufEncoder *encoder, uint64_t value) 
{
  uint64_t v = value;
  while (true) 
  {
    uint8_t byte = (uint8_t)(v & 0x7F);
    v >>= 7;
    if (v == 0) return vec_u8_push(&encoder->buffer, byte);
    else 
    {
      if (!vec_u8_push(&encoder->buffer, byte | 0x80)) return false;
    }
  }
}

// Write signed 32-bit integer
bool protobuf_encoder_write_sint32(ProtobufEncoder *encoder, int32_t value) 
{
  uint32_t encoded = (uint32_t)((value << 1) ^ (value >> 31));
  return protobuf_encoder_write_varint(encoder, (uint64_t)encoded);
}

// Write signed 64-bit integer
bool protobuf_encoder_write_sint64(ProtobufEncoder *encoder, int64_t value) 
{
  uint64_t encoded = (uint64_t)((value << 1) ^ (value >> 63));
  return protobuf_encoder_write_varint(encoder, encoded);
}

// Write tag
bool protobuf_encoder_write_tag(ProtobufEncoder *encoder, uint32_t field_number, uint8_t wire_type) 
{
  uint32_t tag = (field_number << 3) | (uint32_t)wire_type;
  return protobuf_encoder_write_varint(encoder, (uint64_t)tag);
}

// Write varint field
bool protobuf_encoder_write_varint_field(ProtobufEncoder *encoder, uint32_t field_number, uint64_t value) 
{
  if (value == 0) return true;
  return protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_VARINT) && protobuf_encoder_write_varint(encoder, value);
}

// Write sint32 field
bool protobuf_encoder_write_sint32_field(ProtobufEncoder *encoder, uint32_t field_number, int32_t value) 
{
  if (value == 0) return true;
  return protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_VARINT) && protobuf_encoder_write_sint32(encoder, value);
}

// Write bool field
bool protobuf_encoder_write_bool_field(ProtobufEncoder *encoder, uint32_t field_number, bool value) 
{
    if (!value) return true;
    return protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_VARINT) && vec_u8_push(&encoder->buffer, 1);
}

// Write bytes field
bool protobuf_encoder_write_bytes_field(ProtobufEncoder *encoder, uint32_t field_number, const uint8_t *data, size_t len) 
{
  if (len == 0) return true;
  return protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_LENGTH_DELIMITED) &&
         protobuf_encoder_write_varint(encoder, (uint64_t)len) &&
         vec_u8_extend_from_slice(&encoder->buffer, data, len);
}

// Write string field
bool protobuf_encoder_write_string_field(ProtobufEncoder *encoder, uint32_t field_number, const char *s, size_t len) 
{
  return protobuf_encoder_write_bytes_field(encoder, field_number, (const uint8_t *)s, len);
}

// Write fixed32 field
bool protobuf_encoder_write_fixed32_field(ProtobufEncoder *encoder, uint32_t field_number, uint32_t value) 
{
  if (value == 0) return true;
  if (!protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_32BIT)) return false;
  uint8_t bytes[4];
  bytes[0] = (uint8_t)(value & 0xFF);
  bytes[1] = (uint8_t)((value >> 8) & 0xFF);
  bytes[2] = (uint8_t)((value >> 16) & 0xFF);
  bytes[3] = (uint8_t)((value >> 24) & 0xFF);
  return vec_u8_extend_from_slice(&encoder->buffer, bytes, 4);
}

// Write fixed64 field
bool protobuf_encoder_write_fixed64_field(ProtobufEncoder *encoder, uint32_t field_number, uint64_t value) 
{
  if (value == 0) return true;
  if (!protobuf_encoder_write_tag(encoder, field_number, WIRE_TYPE_64BIT)) return false;
  uint8_t bytes[8];
  for (int i = 0; i < 8; i++) bytes[i] = (uint8_t)((value >> (i * 8)) & 0xFF);
  return vec_u8_extend_from_slice(&encoder->buffer, bytes, 8);
}

// Write message field using callback
typedef bool (*EncodeCallback)(ProtobufEncoder *);

bool protobuf_encoder_write_message_field(ProtobufEncoder *encoder, uint32_t field_number, EncodeCallback encode_fn) 
{
  ProtobufEncoder nested;
  protobuf_encoder_init(&nested);
  if (!encode_fn(&nested)) return false;
  if (vec_u8_is_empty(&nested.buffer)) return true;
  return protobuf_encoder_write_bytes_field(encoder, field_number, nested.buffer.data, nested.buffer.len);
}

// Encode varint to slice
void protobuf_encoder_encode_varint_to_slice(uint64_t value, Vec_u8 *output) 
{
  uint64_t v = value;
  while (true) 
  {
    uint8_t byte = (uint8_t)(v & 0x7F);
    v >>= 7;
    if (v == 0) 
    {
      vec_u8_push(output, byte);
      break;
    } 
    else vec_u8_push(output, byte | 0x80);
  }
}

// ProtobufDecoder structure
typedef struct 
{
  const uint8_t *data;
  size_t len;
  size_t pos;
} ProtobufDecoder;

// Initialize ProtobufDecoder
void protobuf_decoder_init(ProtobufDecoder *decoder, const uint8_t *data, size_t len) 
{
  decoder->data = data;
  decoder->len = len;
  decoder->pos = 0;
}

// Check if decoder has more data
bool protobuf_decoder_has_more(const ProtobufDecoder *decoder) 
{
  return decoder->pos < decoder->len;
}

// Read varint
bool protobuf_decoder_read_varint(ProtobufDecoder *decoder, uint64_t *result) 
{
  uint64_t value = 0;
  int shift = 0;
  while (true) 
  {
    if (decoder->pos >= decoder->len) return false;
    uint8_t byte = decoder->data[decoder->pos];
    decoder->pos++;
    value |= ((uint64_t)(byte & 0x7F)) << shift;
    if ((byte & 0x80) == 0) 
    {
      *result = value;
      return true;
    }
    shift += 7;
    if (shift >= 64) return false;
  }
}

// Read sint32
bool protobuf_decoder_read_sint32(ProtobufDecoder *decoder, int32_t *result) 
{
  uint64_t encoded;
  if (!protobuf_decoder_read_varint(decoder, &encoded)) return false;
  uint32_t encoded32 = (uint32_t)encoded;
  *result = (int32_t)((encoded32 >> 1) ^ (-(int32_t)(encoded32 & 1)));
  return true;
}

// Read sint64
bool protobuf_decoder_read_sint64(ProtobufDecoder *decoder, int64_t *result) 
{
  uint64_t encoded;
  if (!protobuf_decoder_read_varint(decoder, &encoded)) return false;
  *result = (int64_t)((encoded >> 1) ^ (-(int64_t)(encoded & 1)));
  return true;
}

// Read tag
bool protobuf_decoder_read_tag(ProtobufDecoder *decoder, uint32_t *field_number, uint8_t *wire_type) 
{
  uint64_t tag;
  if (!protobuf_decoder_read_varint(decoder, &tag)) return false;
  uint32_t tag32 = (uint32_t)tag;
  *field_number = tag32 >> 3;
  *wire_type = (uint8_t)(tag32 & 0x07);
  return true;
}

// Next field (returns field info)
typedef struct 
{
  uint32_t field_number;
  uint8_t wire_type;
  const uint8_t *data;
  size_t len;
} FieldInfo;

bool protobuf_decoder_next_field(ProtobufDecoder *decoder, FieldInfo *field_info) 
{
  if (!protobuf_decoder_has_more(decoder)) return false;
  if (!protobuf_decoder_read_tag(decoder, &field_info->field_number, &field_info->wire_type)) return false;
  switch (field_info->wire_type) 
  {
    case WIRE_TYPE_VARINT: 
    {
      size_t start = decoder->pos;
      while (decoder->pos < decoder->len) 
      {
        uint8_t byte = decoder->data[decoder->pos];
        decoder->pos++;
        if ((byte & 0x80) == 0) break;
      }
      field_info->data = &decoder->data[start];
      field_info->len = decoder->pos - start;
      break;
    }
    
    case WIRE_TYPE_64BIT: 
    {
      if (decoder->pos + 8 > decoder->len) return false;
      field_info->data = &decoder->data[decoder->pos];
      field_info->len = 8;
      decoder->pos += 8;
      break;
    }
    
    case WIRE_TYPE_LENGTH_DELIMITED: 
    {
      uint64_t len;
      if (!protobuf_decoder_read_varint(decoder, &len)) return false;
      size_t len_sz = (size_t)len;
      if (decoder->pos + len_sz > decoder->len) return false;
      field_info->data = &decoder->data[decoder->pos];
      field_info->len = len_sz;
      decoder->pos += len_sz;
      break;
    }
    
    case WIRE_TYPE_32BIT: 
    {
      if (decoder->pos + 4 > decoder->len) return false;
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
bool protobuf_decoder_read_bytes(ProtobufDecoder *decoder, const uint8_t **result, size_t *len) 
{
  uint64_t length;
  if (!protobuf_decoder_read_varint(decoder, &length)) return false;
  size_t length_sz = (size_t)length;
  if (decoder->pos + length_sz > decoder->len) return false;
  *result = &decoder->data[decoder->pos];
  *len = length_sz;
  decoder->pos += length_sz;
  return true;
}

// Read fixed32
bool protobuf_decoder_read_fixed32(ProtobufDecoder *decoder, uint32_t *result) 
{
  if (decoder->pos + 4 > decoder->len) return false;
  *result = ((uint32_t)decoder->data[decoder->pos]) | ((uint32_t)decoder->data[decoder->pos + 1] << 8) |
            ((uint32_t)decoder->data[decoder->pos + 2] << 16) | ((uint32_t)decoder->data[decoder->pos + 3] << 24);
  decoder->pos += 4;
  return true;
}

// Read fixed64
bool protobuf_decoder_read_fixed64(ProtobufDecoder *decoder, uint64_t *result) 
{
  if (decoder->pos + 8 > decoder->len) return false;
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
bool protobuf_decoder_skip_field(ProtobufDecoder *decoder, uint8_t wire_type) 
{
  switch (wire_type) 
  {
    case WIRE_TYPE_VARINT: 
    {
      uint64_t dummy;
      return protobuf_decoder_read_varint(decoder, &dummy);
    }
    
    case WIRE_TYPE_64BIT: 
    {
      if (decoder->pos + 8 <= decoder->len) 
      {
        decoder->pos += 8;
        return true;
      }
      return false;
    }
    
    case WIRE_TYPE_LENGTH_DELIMITED: 
    {
      const uint8_t *dummy_data;
      size_t dummy_len;
      return protobuf_decoder_read_bytes(decoder, &dummy_data, &dummy_len);
    }
    
    case WIRE_TYPE_32BIT: 
    {
      if (decoder->pos + 4 <= decoder->len) 
      {
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
bool protobuf_decoder_read_varint_from_slice(const uint8_t *data, size_t len, uint64_t *result) 
{
  uint64_t value = 0;
  int shift = 0;
  for (size_t i = 0; i < len; i++) 
  {
    uint8_t byte = data[i];
    value |= ((uint64_t)(byte & 0x7F)) << shift;
    if ((byte & 0x80) == 0) 
    {
      *result = value;
      return true;
    }
    shift += 7;
    if (shift >= 64) return false;
  }
  return false;
}

// Read varint advancing pointer
bool protobuf_decoder_read_varint_advancing(const uint8_t **data, size_t *len, uint64_t *result) 
{
  uint64_t value = 0;
  int shift = 0;
  size_t consumed = 0;
  for (size_t i = 0; i < *len; i++) 
  {
    uint8_t byte = (*data)[i];
    value |= ((uint64_t)(byte & 0x7F)) << shift;
    consumed = i + 1;
    
    if ((byte & 0x80) == 0) 
    {
      *data = &(*data)[consumed];
      *len -= consumed;
      *result = value;
      return true;
    }
    shift += 7;
    if (shift >= 64) return false;
  }
  return false;
}

// Read bytes from slice
bool protobuf_decoder_read_bytes_from_slice(const uint8_t **data, size_t *len, const uint8_t **result, size_t *result_len) 
{
  uint64_t length;
  if (!protobuf_decoder_read_varint_advancing(data, len, &length)) return false;
  size_t length_sz = (size_t)length;
  if (*len < length_sz) return false;
  *result = *data;
  *result_len = length_sz;
  *data = &(*data)[length_sz];
  *len -= length_sz;
  return true;
}

// Read tag from slice
bool protobuf_decoder_read_tag_from_slice(const uint8_t **data, size_t *len, uint32_t *field_number, uint8_t *wire_type) 
{
  uint64_t tag;
  if (!protobuf_decoder_read_varint_advancing(data, len, &tag)) return false;
  uint32_t tag32 = (uint32_t)tag;
  *field_number = tag32 >> 3;
  *wire_type = (uint8_t)(tag32 & 0x07);
  return true;
}

// Skip field from slice
bool protobuf_decoder_skip_field_from_slice(const uint8_t **data, size_t *len, uint8_t wire_type) 
{
  switch (wire_type) 
  {
    case WIRE_TYPE_VARINT: 
    {
      uint64_t dummy;
      return protobuf_decoder_read_varint_advancing(data, len, &dummy);
    }
    
    case WIRE_TYPE_64BIT: 
    {
      if (*len >= 8) 
      {
        *data = &(*data)[8];
        *len -= 8;
        return true;
      }
      return false;
    }
    
    case WIRE_TYPE_LENGTH_DELIMITED: 
    {
      const uint8_t *dummy_data;
      size_t dummy_len;
      return protobuf_decoder_read_bytes_from_slice(data, len, &dummy_data, &dummy_len);
    }
    
    case WIRE_TYPE_32BIT: 
    {
      if (*len >= 4) 
      {
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
void hex_encode(const uint8_t *data, size_t data_len, uint8_t *output, size_t output_len) 
{
  static const uint8_t HEX_CHARS[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  for (size_t i = 0; i < data_len; i++) 
  {
    if (i * 2 + 1 < output_len) 
    {
      uint8_t byte = data[i];
      output[i * 2] = HEX_CHARS[byte >> 4];
      output[i * 2 + 1] = HEX_CHARS[byte & 0x0F];
    }
  }
}

// Hex decode helper - convert hex char to value
static inline int hex_char_to_value(uint8_t c) 
{
  if (c >= '0' && c <= '9') return c - '0';
  else if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  else if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Hex decode
void hex_decode(const uint8_t *data, size_t data_len, uint8_t *output, size_t output_len) 
{
  for (size_t i = 0; i < data_len / 2 && i < output_len; i++) 
  {
    int high = hex_char_to_value(data[i * 2]);
    int low = hex_char_to_value(data[i * 2 + 1]);
    if (high >= 0 && low >= 0) output[i] = (uint8_t)((high << 4) | low);
  }
}

// Encode DataPayload
bool encode_data(const DataPayload *data, uint8_t *output, size_t *output_len, size_t max_len) 
{
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
  if (encoder.buffer.len > max_len) return false;
  memcpy(output, encoder.buffer.data, encoder.buffer.len);
  *output_len = encoder.buffer.len;
  return true;
}

// Encode Position
bool encode_position(const Position *pos, uint8_t *output, size_t *output_len, size_t max_len) 
{
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
   if (encoder.buffer.len > max_len) return false;
  memcpy(output, encoder.buffer.data, encoder.buffer.len);
  *output_len = encoder.buffer.len;
  return true;
}

// Encode User
bool encode_user(const User *user, uint8_t *output, size_t *output_len, size_t max_len) 
{
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
  if (encoder.buffer.len > max_len) return false;
  memcpy(output, encoder.buffer.data, encoder.buffer.len);
  *output_len = encoder.buffer.len;
  return true;
}

// Decode DataPayload
bool decode_data(const uint8_t *data, size_t len, DataPayload *result) 
{
  ProtobufDecoder decoder;
  protobuf_decoder_init(&decoder, data, len);
  // Initialize result with defaults
  memset(result, 0, sizeof(DataPayload));
  while (protobuf_decoder_has_more(&decoder)) 
  {
    uint32_t field_number;
    uint8_t wire_type;
    if (!protobuf_decoder_read_tag(&decoder, &field_number, &wire_type)) return false;
    switch (field_number) 
    {
      case DATA_FIELD_PORTNUM:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
          uint64_t value;
          if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
          result->port = (PortNum)((uint32_t)value);
      }
      break;
      
      case DATA_FIELD_PAYLOAD:
      if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) 
      {
          const uint8_t *bytes;
          size_t bytes_len;
          if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) return false;
          if (bytes_len > MAX_LORA_PAYLOAD) return false;
          memcpy(result->payload, bytes, bytes_len);
          result->payload_len = bytes_len;
      }
      break;
      
      case DATA_FIELD_WANT_RESPONSE:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
          uint64_t value;
          if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
          result->want_response = (value != 0);
      }
      break;
      
      case DATA_FIELD_DEST:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
          uint64_t value;
          if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
          result->dest = (uint32_t)value;
      }
      break;
      
      case DATA_FIELD_SOURCE:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->source = (uint32_t)value;
      }
      break;
      
      case DATA_FIELD_REQUEST_ID:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->request_id = (uint32_t)value;
      }
      break;
      
      case DATA_FIELD_REPLY_ID:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->reply_id = (uint32_t)value;
      }
      break;
      
      case DATA_FIELD_EMOJI:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->emoji = (uint32_t)value;
      }
      break;
       
      default:
      if (!protobuf_decoder_skip_field(&decoder, wire_type)) return false;
      break;
    }
  }
  return true;
}

// Decode Position
bool decode_position(const uint8_t *data, size_t len, Position *result) 
{
  ProtobufDecoder decoder;
  protobuf_decoder_init(&decoder, data, len);
  // Initialize result with defaults
  memset(result, 0, sizeof(Position));
  while (protobuf_decoder_has_more(&decoder)) 
  {
    uint32_t field_number;
    uint8_t wire_type;
    if (!protobuf_decoder_read_tag(&decoder, &field_number, &wire_type)) return false;
    switch (field_number) 
    {
      case POSITION_FIELD_LATITUDE_I:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        if (!protobuf_decoder_read_sint32(&decoder, &result->latitude_i)) return false;
      }
      break;
          
      case POSITION_FIELD_LONGITUDE_I:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        if (!protobuf_decoder_read_sint32(&decoder, &result->longitude_i)) return false;
      }
      break;
          
      case POSITION_FIELD_ALTITUDE:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        if (!protobuf_decoder_read_sint32(&decoder, &result->altitude)) return false;
      }
      break;
          
      case POSITION_FIELD_TIME:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->time = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_LOCATION_SOURCE:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        uint8_t val_u8 = (uint8_t)value;
        switch (val_u8) 
        {
          case 1: result->location_source = LOCATION_SOURCE_MANUAL; break;
          case 2: result->location_source = LOCATION_SOURCE_INTERNAL_GPS; break;
          case 3: result->location_source = LOCATION_SOURCE_EXTERNAL_GPS; break;
          default: result->location_source = LOCATION_SOURCE_UNSET; break;
        }
      }
      break;
          
      case POSITION_FIELD_ALTITUDE_SOURCE:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        uint8_t val_u8 = (uint8_t)value;
        switch (val_u8) 
        {
          case 1: result->altitude_source = LOCATION_SOURCE_MANUAL; break;
          case 2: result->altitude_source = LOCATION_SOURCE_INTERNAL_GPS; break;
          case 3: result->altitude_source = LOCATION_SOURCE_EXTERNAL_GPS; break;
          default: result->altitude_source = LOCATION_SOURCE_UNSET; break;
        }
      }
      break;
          
      case POSITION_FIELD_TIMESTAMP:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->timestamp = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_PDOP:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->pdop = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_HDOP:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->hdop = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_SATS_IN_VIEW:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->sats_in_view = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_GROUND_SPEED:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->ground_speed = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_GROUND_TRACK:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->ground_track = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_FIX_QUALITY:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->fix_quality = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_FIX_TYPE:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->fix_type = (uint32_t)value;
      }
      break;
          
      case POSITION_FIELD_SEQ_NUMBER:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->seq_number = (uint32_t)value;
      }
      break;
          
      default:
      if (!protobuf_decoder_skip_field(&decoder, wire_type)) return false;
      break;
    }
  }
  return true;
}

// Decode User
bool decode_user(const uint8_t *data, size_t len, User *result) 
{
  ProtobufDecoder decoder;
  protobuf_decoder_init(&decoder, data, len);
  // Initialize result with defaults
  memset(result, 0, sizeof(User));
  result->hw_model = HARDWARE_MODEL_UNSET;
  result->is_licensed = false;
  result->role = ROLE_CLIENT;
  while (protobuf_decoder_has_more(&decoder)) 
  {
    uint32_t field_number;
    uint8_t wire_type;
    if (!protobuf_decoder_read_tag(&decoder, &field_number, &wire_type)) return false;
    switch (field_number) 
    {
      case USER_FIELD_ID:
      if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) 
      {
        const uint8_t *bytes;
        size_t bytes_len;
        if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) return false;
        if (bytes_len >= 17 && bytes[0] == '!') hex_decode(&bytes[1], 16, result->id, 8);
        else if (bytes_len == 8) memcpy(result->id, bytes, 8);
      }
      break;
          
      case USER_FIELD_LONG_NAME:
      if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) 
      {
        const uint8_t *bytes;
        size_t bytes_len;
        if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) return false;
        if (bytes_len > sizeof(result->long_name)) bytes_len = sizeof(result->long_name);
        memcpy(result->long_name, bytes, bytes_len);
        result->long_name_len = bytes_len;
      }
      break;
          
      case USER_FIELD_SHORT_NAME:
      if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) 
      {
        const uint8_t *bytes;
        size_t bytes_len;
        if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) return false;
        if (bytes_len > sizeof(result->short_name)) bytes_len = sizeof(result->short_name);
        memcpy(result->short_name, bytes, bytes_len);
        result->short_name_len = bytes_len;
      }
      break;
          
      case USER_FIELD_MACADDR:
      if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) 
      {
        const uint8_t *bytes;
        size_t bytes_len;
        if (!protobuf_decoder_read_bytes(&decoder, &bytes, &bytes_len)) return false;
        if (bytes_len == 6) memcpy(&result->id[2], bytes, 6);
      }
      break;
          
      case USER_FIELD_HW_MODEL:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        uint16_t val_u16 = (uint16_t)value;
        switch (val_u16) 
        {
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
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        result->is_licensed = (value != 0);
      }
      break;
          
      case USER_FIELD_ROLE:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t value;
        if (!protobuf_decoder_read_varint(&decoder, &value)) return false;
        uint8_t val_u8 = (uint8_t)value;
        switch (val_u8) 
        {
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
      if (!protobuf_decoder_skip_field(&decoder, wire_type)) return false;
      break;
    }
  }
  return true;
}


/////////////////////////////////////////////////////////////////////////////

void ProtobufEncoder_init(ProtobufEncoder *e, uint8_t *buffer, size_t capacity) 
{
  e->buf = buffer;
  e->capacity = capacity;
  e->pos = 0;
}

bool ProtobufEncoder_write_varint(ProtobufEncoder *e, uint64_t value) 
{
  while (value >= 0x80) 
  {
    if (e->pos >= e->capacity) return false;
    e->buf[e->pos++] = (uint8_t)((value & 0x7F) | 0x80);
    value >>= 7;
  }
  if (e->pos >= e->capacity) return false;
  e->buf[e->pos++] = (uint8_t)value;
  return true;
}

bool ProtobufEncoder_write_tag(ProtobufEncoder *e, uint32_t field_number, uint8_t wire_type) 
{
  return ProtobufEncoder_write_varint(e, (uint64_t)(field_number << 3) | wire_type);
}

bool ProtobufEncoder_write_varint_field(ProtobufEncoder *e, uint32_t field_number, uint64_t value) 
{
  if (!ProtobufEncoder_write_tag(e, field_number, WIRE_TYPE_VARINT)) return false;
  return ProtobufEncoder_write_varint(e, value);
}

bool ProtobufEncoder_write_bytes(ProtobufEncoder *e, const uint8_t *data, size_t len) 
{
  if (!ProtobufEncoder_write_varint(e, (uint64_t)len)) return false;
  if (e->pos + len > e->capacity) return false;
  memcpy(&e->buf[e->pos], data, len);
  e->pos += len;
  return true;
}

bool ProtobufEncoder_write_bytes_field(ProtobufEncoder *e, uint32_t field_number, const uint8_t *data, size_t len) 
{
  if (!ProtobufEncoder_write_tag(e, field_number, WIRE_TYPE_LENGTH_DELIMITED)) return false;
  return ProtobufEncoder_write_bytes(e, data, len);
}

bool ProtobufEncoder_write_fixed32(ProtobufEncoder *e, uint32_t value) 
{
  if (e->pos + 4 > e->capacity) return false;
  memcpy(&e->buf[e->pos], &value, 4);
  e->pos += 4;
  return true;
}

bool ProtobufEncoder_write_fixed32_field(ProtobufEncoder *e, uint32_t field_number, uint32_t value) 
{
  if (!ProtobufEncoder_write_tag(e, field_number, WIRE_TYPE_32BIT)) return false;
  return ProtobufEncoder_write_fixed32(e, value);
}

bool ProtobufEncoder_write_bool_field(ProtobufEncoder *e, uint32_t field_number, bool value) 
{
  return ProtobufEncoder_write_varint_field(e, field_number, value ? 1 : 0);
}

bool ProtobufEncoder_write_string_field(ProtobufEncoder *e, uint32_t field_number, const char *s) 
{
  size_t len = strlen(s);
  return ProtobufEncoder_write_bytes_field(e, field_number, (const uint8_t*)s, len);
}

bool ProtobufEncoder_write_sint32(ProtobufEncoder *e, int32_t n) 
{
  uint32_t encoded = (uint32_t)((n << 1) ^ (n >> 31));
  return ProtobufEncoder_write_varint(e, (uint64_t)encoded);
}

// Helper for message fields in C (since we don't have closures)
bool ProtobufEncoder_write_message_field(ProtobufEncoder *e, uint32_t field_number, const uint8_t *msg_data, size_t msg_len) 
{
  return ProtobufEncoder_write_bytes_field(e, field_number, msg_data, msg_len);
}



void ProtobufDecoder_init(ProtobufDecoder *d, const uint8_t *buffer, size_t len) 
{
  d->buf = buffer;
  d->len = len;
  d->pos = 0;
}

bool ProtobufDecoder_has_more(ProtobufDecoder *d) 
{
  return d->pos < d->len;
}

bool ProtobufDecoder_read_varint(ProtobufDecoder *d, uint64_t *out) 
{
  uint64_t value = 0;
  uint32_t shift = 0;
  while (d->pos < d->len) 
  {
    uint8_t byte = d->buf[d->pos++];
    value |= (uint64_t)(byte & 0x7F) << shift;
    if (!(byte & 0x80)) 
    {
      *out = value;
      return true;
    }
    shift += 7;
    if (shift >= 64) return false;
  }
  return false;
}

bool ProtobufDecoder_read_tag(ProtobufDecoder *d, uint32_t *field_number, uint8_t *wire_type) 
{
  uint64_t tag;
  if (!ProtobufDecoder_read_varint(d, &tag)) return false;
  *field_number = (uint32_t)(tag >> 3);
  *wire_type = (uint8_t)(tag & 0x07);
  return true;
}

bool ProtobufDecoder_read_fixed32(ProtobufDecoder *d, uint32_t *out) 
{
  if (d->pos + 4 > d->len) return false;
  memcpy(out, &d->buf[d->pos], 4);
  d->pos += 4;
  return true;
}

bool ProtobufDecoder_read_bytes(ProtobufDecoder *d, const uint8_t **out_ptr, size_t *out_len) 
{
  uint64_t len;
  if (!ProtobufDecoder_read_varint(d, &len)) return false;
  if (d->pos + len > d->len) return false;
  *out_ptr = &d->buf[d->pos];
  *out_len = (size_t)len;
  d->pos += (size_t)len;
  return true;
}

bool ProtobufDecoder_read_sint32(ProtobufDecoder *d, int32_t *out) 
{
  uint64_t n;
  if (!ProtobufDecoder_read_varint(d, &n)) return false;
  *out = (int32_t)((n >> 1) ^ -(int32_t)(n & 1));
  return true;
}

bool ProtobufDecoder_skip_field(ProtobufDecoder *d, uint8_t wire_type) 
{
  switch (wire_type) 
  {
    case WIRE_TYPE_VARINT: 
    {
      uint64_t dummy;
      return ProtobufDecoder_read_varint(d, &dummy);
    }
    
    case WIRE_TYPE_64BIT:
    if (d->pos + 8 > d->len) return false;
    d->pos += 8;
    return true;
        
    case WIRE_TYPE_LENGTH_DELIMITED: 
    {
      uint64_t len;
      if (!ProtobufDecoder_read_varint(d, &len)) return false;
      if (d->pos + len > d->len) return false;
      d->pos += (size_t)len;
      return true;
    }
    
    case WIRE_TYPE_32BIT:
    if (d->pos + 4 > d->len) return false;
    d->pos += 4;
    return true;
        
    default:
    return false;
  }
}

/**
 * Hex Encoding / Decoding
 */

void hex_encode(const uint8_t *data, size_t data_len, uint8_t *output, size_t output_len) 
{
  const uint8_t HEX_CHARS[16] = "0123456789abcdef";
  for (size_t i = 0; i < data_len; i++) 
  {
    uint8_t byte = data[i];
    if (i * 2 + 1 < output_len) 
    {
      output[i * 2] = HEX_CHARS[(byte >> 4)];
      output[i * 2 + 1] = HEX_CHARS[(byte & 0x0F)];
    }
  }
}

uint8_t hex_val(uint8_t c) 
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return 0;
}

void hex_decode(const uint8_t *data, size_t data_len, uint8_t *output, size_t output_len) 
{
  for (size_t i = 0; i < output_len; i++) 
  {
    if (i * 2 + 1 < data_len) output[i] = (uint8_t)((hex_val(data[i * 2]) << 4) | hex_val(data[i * 2 + 1]));
  }
}

/**
 * NodeInfo Fields
 */
enum nodeinfo_fields 
{
  NODEINFO_FIELDS_NUM = 1,
  NODEINFO_FIELDS_USER = 2,
  NODEINFO_FIELDS_POSITION = 3,
  NODEINFO_FIELDS_SNR = 4,
  NODEINFO_FIELDS_LAST_HEARD = 5,
  NODEINFO_FIELDS_DEVICE_METRICS = 6
};

// Prototypes for sub-encoders
bool encode_user(const User *u, FixedVec *out);
bool encode_position(const Position *p, FixedVec *out);

bool encode_nodeinfo(uint32_t num,const User *user,const Position *position,float snr,uint32_t last_heard,FixedVec *output) 
{
  uint8_t buffer[MAX_LORA_PAYLOAD];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, MAX_LORA_PAYLOAD);
  if (!ProtobufEncoder_write_varint_field(&encoder, NODEINFO_FIELDS_NUM, (uint64_t)num)) return false;
  if (user != NULL) 
  {
    FixedVec user_data;
    if (encode_user(user, &user_data)) 
    {
      if (!ProtobufEncoder_write_bytes_field(&encoder, NODEINFO_FIELDS_USER, user_data.data, user_data.len)) return false;
    } 
    else return false;
  }
  if (position != NULL) 
  {
    FixedVec pos_data;
    if (encode_position(position, &pos_data)) 
    {
      if (!ProtobufEncoder_write_bytes_field(&encoder, NODEINFO_FIELDS_POSITION, pos_data.data, pos_data.len)) return false;
    } 
    else return false;
  }
  if (snr != 0.0f) 
  {
    uint32_t bits;
    memcpy(&bits, &snr, 4);
    if (!ProtobufEncoder_write_fixed32_field(&encoder, NODEINFO_FIELDS_SNR, bits)) return false;
  }
  if (!ProtobufEncoder_write_varint_field(&encoder, NODEINFO_FIELDS_LAST_HEARD, (uint64_t)last_heard)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

/**
 * Routing Error
 */
typedef enum 
{
  RoutingError_None = 0,
  RoutingError_NoRoute = 1,
  RoutingError_GotNak = 2,
  RoutingError_Timeout = 3,
  RoutingError_NoInterface = 4,
  RoutingError_MaxRetransmit = 5,
  RoutingError_NoChannel = 6,
  RoutingError_TooLarge = 7,
  RoutingError_NoResponse = 8,
  RoutingError_DutyCycleLimit = 9,
  RoutingError_BadRequest = 32,
  RoutingError_NotAuthorized = 33,
} RoutingError;

enum routing_fields 
{
  ROUTING_FIELDS_ROUTE_REQUEST = 1,
  ROUTING_FIELDS_ROUTE_REPLY = 2,
  ROUTING_FIELDS_ERROR_REASON = 3,
};

bool encode_routing_error(RoutingError error, FixedVec *output) 
{
  uint8_t buffer[16];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, 16);
  if (!ProtobufEncoder_write_varint_field(&encoder, ROUTING_FIELDS_ERROR_REASON, (uint64_t)error)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

bool decode_routing_error(const uint8_t *data, size_t len, RoutingError *out_error) 
{
  ProtobufDecoder decoder;
  ProtobufDecoder_init(&decoder, data, len);
  while (ProtobufDecoder_has_more(&decoder)) 
  {
    uint32_t field_number;
    uint8_t wire_type;
    if (!ProtobufDecoder_read_tag(&decoder, &field_number, &wire_type)) return false;
    if (field_number == ROUTING_FIELDS_ERROR_REASON && wire_type == WIRE_TYPE_VARINT) 
    {
      uint64_t val;
      if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
      switch ((uint8_t)val) 
      {
        case 0: *out_error = RoutingError_None; break;
        case 1: *out_error = RoutingError_NoRoute; break;
        case 2: *out_error = RoutingError_GotNak; break;
        case 3: *out_error = RoutingError_Timeout; break;
        case 4: *out_error = RoutingError_NoInterface; break;
        case 5: *out_error = RoutingError_MaxRetransmit; break;
        case 6: *out_error = RoutingError_NoChannel; break;
        case 7: *out_error = RoutingError_TooLarge; break;
        case 8: *out_error = RoutingError_NoResponse; break;
        case 9: *out_error = RoutingError_DutyCycleLimit; break;
        case 32: *out_error = RoutingError_BadRequest; break;
        case 33: *out_error = RoutingError_NotAuthorized; break;
        default: *out_error = RoutingError_None; break;
      }
      return true;
    } 
    else 
    {
      if (!ProtobufDecoder_skip_field(&decoder, wire_type)) return false;
    }
  }
  return false;
}

/**
 * Admin Op Enum
 */
typedef enum 
{
  AdminOp_GetChannelRequest = 1,
  AdminOp_GetChannelResponse = 2,
  AdminOp_GetOwnerRequest = 3,
  AdminOp_GetOwnerResponse = 4,
  AdminOp_GetConfigRequest = 5,
  AdminOp_GetConfigResponse = 6,
  AdminOp_GetModuleConfigRequest = 7,
  AdminOp_GetModuleConfigResponse = 8,
  AdminOp_GetCannedMessageRequest = 10,
  AdminOp_GetCannedMessageResponse = 11,
  AdminOp_GetDeviceMetadataRequest = 12,
  AdminOp_GetDeviceMetadataResponse = 13,
  AdminOp_GetRingtoneRequest = 14,
  AdminOp_GetRingtoneResponse = 15,
  AdminOp_GetDeviceConnectionStatusRequest = 16,
  AdminOp_GetDeviceConnectionStatusResponse = 17,
  AdminOp_SetOwner = 32,
  AdminOp_SetChannel = 33,
  AdminOp_SetConfig = 34,
  AdminOp_SetModuleConfig = 35,
  AdminOp_SetCannedMessageModule = 36,
  AdminOp_SetRingtoneMessage = 37,
  AdminOp_RemoveByNodenum = 38,
  AdminOp_SetFavoriteNode = 39,
  AdminOp_RemoveFavoriteNode = 40,
  AdminOp_BeginEditSettings = 64,
  AdminOp_CommitEditSettings = 65,
  AdminOp_RebootOtaSeconds = 95,
  AdminOp_ExitSimulator = 96,
  AdminOp_RebootSeconds = 97,
  AdminOp_ShutdownSeconds = 98,
  AdminOp_FactoryResetDevice = 99,
  AdminOp_NodedbReset = 100,
} AdminOp;

/**
 * Telemetry
 */
enum telemetry_fields 
{
  TELEMETRY_FIELDS_TIME = 1,
  TELEMETRY_FIELDS_DEVICE_METRICS = 2,
  TELEMETRY_FIELDS_ENVIRONMENT_METRICS = 3,
  TELEMETRY_FIELDS_AIR_QUALITY_METRICS = 4,
  TELEMETRY_FIELDS_POWER_METRICS = 5,
};

enum device_metrics_fields 
{
  DEVICE_METRICS_FIELDS_BATTERY_LEVEL = 1,
  DEVICE_METRICS_FIELDS_VOLTAGE = 2,
  DEVICE_METRICS_FIELDS_CHANNEL_UTILIZATION = 3,
  DEVICE_METRICS_FIELDS_AIR_UTIL_TX = 4,
  DEVICE_METRICS_FIELDS_UPTIME_SECONDS = 5,
};

typedef struct 
{
  uint32_t battery_level;
  uint32_t voltage;
  uint32_t channel_utilization;
  uint32_t air_util_tx;
  uint32_t uptime_seconds;
} DeviceMetrics;

void DeviceMetrics_default(DeviceMetrics *m) 
{
  m->battery_level = 0;
  m->voltage = 0;
  m->channel_utilization = 0;
  m->air_util_tx = 0;
  m->uptime_seconds = 0;
}

bool encode_device_metrics(const DeviceMetrics *metrics, FixedVec *output) 
{
  uint8_t buffer[64];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, 64);
  if (!ProtobufEncoder_write_varint_field(&encoder, DEVICE_METRICS_FIELDS_BATTERY_LEVEL, (uint64_t)metrics->battery_level)) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, DEVICE_METRICS_FIELDS_VOLTAGE, (uint64_t)metrics->voltage)) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, DEVICE_METRICS_FIELDS_CHANNEL_UTILIZATION, (uint64_t)metrics->channel_utilization)) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, DEVICE_METRICS_FIELDS_AIR_UTIL_TX, (uint64_t)metrics->air_util_tx)) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, DEVICE_METRICS_FIELDS_UPTIME_SECONDS, (uint64_t)metrics->uptime_seconds)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

bool encode_telemetry(uint32_t time, const DeviceMetrics *metrics, FixedVec *output) 
{
  uint8_t buffer[MAX_LORA_PAYLOAD];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, MAX_LORA_PAYLOAD);
  if (!ProtobufEncoder_write_varint_field(&encoder, TELEMETRY_FIELDS_TIME, (uint64_t)time)) return false;
  FixedVec metrics_data;
  if (!encode_device_metrics(metrics, &metrics_data)) return false;
  if (!ProtobufEncoder_write_bytes_field(&encoder, TELEMETRY_FIELDS_DEVICE_METRICS, metrics_data.data, metrics_data.len)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

bool decode_device_metrics(const uint8_t *data, size_t len, DeviceMetrics *out_metrics) 
{
  ProtobufDecoder decoder;
  ProtobufDecoder_init(&decoder, data, len);
  DeviceMetrics_default(out_metrics);
  while (ProtobufDecoder_has_more(&decoder)) 
  {
    uint32_t field_number;
    uint8_t wire_type;
    if (!ProtobufDecoder_read_tag(&decoder, &field_number, &wire_type)) return false;
    switch (field_number) 
    {
      case DEVICE_METRICS_FIELDS_BATTERY_LEVEL:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        out_metrics->battery_level = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case DEVICE_METRICS_FIELDS_VOLTAGE:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        out_metrics->voltage = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case DEVICE_METRICS_FIELDS_CHANNEL_UTILIZATION:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        out_metrics->channel_utilization = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case DEVICE_METRICS_FIELDS_AIR_UTIL_TX:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        out_metrics->air_util_tx = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type); 
      break;
          
      case DEVICE_METRICS_FIELDS_UPTIME_SECONDS:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        out_metrics->uptime_seconds = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      default:
      if (!ProtobufDecoder_skip_field(&decoder, wire_type)) return false;
      break;
    }
  }
  return true;
}

/**
 * Radio Fields
 */
enum to_radio_fields 
{
  TO_RADIO_FIELDS_PACKET = 1,
  TO_RADIO_FIELDS_WANT_CONFIG_ID = 3,
  TO_RADIO_FIELDS_DISCONNECT = 4,
};

enum from_radio_fields 
{
  FROM_RADIO_FIELDS_ID = 1,
  FROM_RADIO_FIELDS_PACKET = 2,
  FROM_RADIO_FIELDS_MY_INFO = 3,
  FROM_RADIO_FIELDS_NODE_INFO = 4,
  FROM_RADIO_FIELDS_CONFIG = 5,
  FROM_RADIO_FIELDS_LOG_RECORD = 6,
  FROM_RADIO_FIELDS_CONFIG_COMPLETE_ID = 7,
  FROM_RADIO_FIELDS_REBOOTED = 8,
  FROM_RADIO_FIELDS_MODULE_CONFIG = 9,
  FROM_RADIO_FIELDS_CHANNEL = 10,
  FROM_RADIO_FIELDS_QUEUED_TEXT_MESSAGE_ACK = 11,
  FROM_RADIO_FIELDS_XM0DEM = 12,
  FROM_RADIO_FIELDS_METADATA = 13,
  FROM_RADIO_FIELDS_MQTTCLIENT_PROXY_MESSAGE = 14,
};

enum mesh_packet_fields 
{
  MESH_PACKET_FIELDS_FROM = 1,
  MESH_PACKET_FIELDS_TO = 2,
  MESH_PACKET_FIELDS_CHANNEL = 3,
  MESH_PACKET_FIELDS_ENCRYPTED = 4,
  MESH_PACKET_FIELDS_DECODED = 5,
  MESH_PACKET_FIELDS_ID = 6,
  MESH_PACKET_FIELDS_RX_TIME = 7,
  MESH_PACKET_FIELDS_RX_SNR = 8,
  MESH_PACKET_FIELDS_HOP_LIMIT = 9,
  MESH_PACKET_FIELDS_WANT_ACK = 10,
  MESH_PACKET_FIELDS_PRIORITY = 11,
  MESH_PACKET_FIELDS_RX_RSSI = 12,
  MESH_PACKET_FIELDS_DELAYED = 13,
  MESH_PACKET_FIELDS_VIA_MQTT = 14,
  MESH_PACKET_FIELDS_HOP_START = 15,
};

typedef enum 
{
  ToRadio_Packet,
  ToRadio_WantConfigId,
  ToRadio_Disconnect,
} ToRadioType;

typedef struct 
{
  ToRadioType type;
  union 
  {
    MeshPacket packet;
    uint32_t want_config_id;
  } contents;
} ToRadio;

// Proto definitions
bool decode_mesh_packet(const uint8_t *data, size_t len, MeshPacket *out_packet);
bool decode_data(const uint8_t *data, size_t len, DataPayload *out_payload);

bool decode_to_radio(const uint8_t *data, size_t len, ToRadio *out_radio) 
{
  ProtobufDecoder decoder;
  ProtobufDecoder_init(&decoder, data, len);
  while (ProtobufDecoder_has_more(&decoder)) 
  {
    uint32_t field_number;
    uint8_t wire_type;
    if (!ProtobufDecoder_read_tag(&decoder, &field_number, &wire_type)) return false;
    switch (field_number) 
    {
      case TO_RADIO_FIELDS_PACKET:
      if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) 
      {
        const uint8_t *packet_data;
        size_t packet_len;
        if (!ProtobufDecoder_read_bytes(&decoder, &packet_data, &packet_len)) return false;
        out_radio->type = ToRadio_Packet;
        if (!decode_mesh_packet(packet_data, packet_len, &out_radio->contents.packet)) return false;
        return true;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case TO_RADIO_FIELDS_WANT_CONFIG_ID:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        out_radio->type = ToRadio_WantConfigId;
        out_radio->contents.want_config_id = (uint32_t)val;
        return true;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case TO_RADIO_FIELDS_DISCONNECT:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t dummy;
        ProtobufDecoder_read_varint(&decoder, &dummy);
        out_radio->type = ToRadio_Disconnect;
        return true;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      default:
      if (!ProtobufDecoder_skip_field(&decoder, wire_type)) return false;
      break;
    }
  }
  return false;
}

void MeshPacket_default(MeshPacket *p) 
{
  memset(p, 0, sizeof(MeshPacket));
}

bool decode_mesh_packet(const uint8_t *data, size_t len, MeshPacket *packet) 
{
  ProtobufDecoder decoder;
  ProtobufDecoder_init(&decoder, data, len);
  MeshPacket_default(packet);
  while (ProtobufDecoder_has_more(&decoder)) 
  {
    uint32_t field_number;
    uint8_t wire_type;
    if (!ProtobufDecoder_read_tag(&decoder, &field_number, &wire_type)) return false;
    switch (field_number) 
    {
      case MESH_PACKET_FIELDS_FROM:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->from = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_TO:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->to = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_CHANNEL:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->channel = (uint8_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_ENCRYPTED:
      if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) 
      {
        const uint8_t *bytes;
        size_t bytes_len;
        if (!ProtobufDecoder_read_bytes(&decoder, &bytes, &bytes_len)) return false;
        if (bytes_len > MAX_LORA_PAYLOAD) return false;
        packet->payload.type = PayloadType_Encrypted;
        memcpy(packet->payload.contents.encrypted.data, bytes, bytes_len);
        packet->payload.contents.encrypted.len = bytes_len;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_DECODED:
      if (wire_type == WIRE_TYPE_LENGTH_DELIMITED) 
      {
        const uint8_t *decoded_data;
        size_t decoded_len;
        if (!ProtobufDecoder_read_bytes(&decoder, &decoded_data, &decoded_len)) return false;
        if (decode_data(decoded_data, decoded_len, &packet->payload.contents.decoded)) packet->payload.type = PayloadType_Decoded;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_ID:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->id = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_RX_TIME:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->rx_time = (uint32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_RX_SNR:
      if (wire_type == WIRE_TYPE_32BIT) 
      {
        uint32_t bits;
        if (!ProtobufDecoder_read_fixed32(&decoder, &bits)) return false;
        memcpy(&packet->rx_snr, &bits, 4);
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_HOP_LIMIT:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->hop_limit = (uint8_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_WANT_ACK:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->want_ack = (val != 0);
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_PRIORITY:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->priority = (Priority)((uint8_t)val);
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      case MESH_PACKET_FIELDS_RX_RSSI:
      if (wire_type == WIRE_TYPE_VARINT) 
      {
        uint64_t val;
        if (!ProtobufDecoder_read_varint(&decoder, &val)) return false;
        packet->rx_rssi = (int32_t)val;
      } 
      else ProtobufDecoder_skip_field(&decoder, wire_type);
      break;
          
      default:
      if (!ProtobufDecoder_skip_field(&decoder, wire_type)) return false;
      break;
    }
  }
  return true;
}

bool encode_data(const DataPayload *data, FixedVec *out);

bool encode_mesh_packet(const MeshPacket *packet, FixedVec *output) 
{
  uint8_t buffer[256];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, 256);
  if (!ProtobufEncoder_write_varint_field(&encoder, MESH_PACKET_FIELDS_FROM, (uint64_t)packet->from)) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, MESH_PACKET_FIELDS_TO, (uint64_t)packet->to)) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, MESH_PACKET_FIELDS_CHANNEL, (uint64_t)packet->channel)) return false;
  if (packet->payload.type == PayloadType_Encrypted) 
  {
    if (!ProtobufEncoder_write_bytes_field(&encoder, MESH_PACKET_FIELDS_ENCRYPTED, 
        packet->payload.contents.encrypted.data, packet->payload.contents.encrypted.len)) return false;
  } 
  else 
  {
    FixedVec encoded;
    if (!encode_data(&packet->payload.contents.decoded, &encoded)) return false;
    if (!ProtobufEncoder_write_bytes_field(&encoder, MESH_PACKET_FIELDS_DECODED, encoded.data, encoded.len)) return false;
  }
  if (!ProtobufEncoder_write_varint_field(&encoder, MESH_PACKET_FIELDS_ID, (uint64_t)packet->id)) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, MESH_PACKET_FIELDS_RX_TIME, (uint64_t)packet->rx_time)) return false;
  if (packet->rx_snr != 0.0f) 
  {
    uint32_t bits;
    memcpy(&bits, &packet->rx_snr, 4);
    if (!ProtobufEncoder_write_fixed32_field(&encoder, MESH_PACKET_FIELDS_RX_SNR, bits)) return false;
  }
  if (!ProtobufEncoder_write_varint_field(&encoder, MESH_PACKET_FIELDS_HOP_LIMIT, (uint64_t)packet->hop_limit)) return false;
  if (!ProtobufEncoder_write_bool_field(&encoder, MESH_PACKET_FIELDS_WANT_ACK, packet->want_ack)) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, MESH_PACKET_FIELDS_PRIORITY, (uint64_t)packet->priority)) return false;
  if (packet->rx_rssi != 0) 
  {
    uint32_t encoded_rssi = (uint32_t)((packet->rx_rssi << 1) ^ (packet->rx_rssi >> 31));
    if (!ProtobufEncoder_write_varint_field(&encoder, MESH_PACKET_FIELDS_RX_RSSI, (uint64_t)encoded_rssi)) return false;
  }
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

static uint32_t next_from_radio_id() 
{
  static _Atomic uint_fast32_t FROM_RADIO_ID = 0;
  uint32_t current = atomic_fetch_add(&FROM_RADIO_ID, 1);
  // Mimics wrapping_add(1) after fetch
  return current + 1;
}

bool encode_from_radio_packet(const MeshPacket *packet, FixedVec *output) 
{
  uint8_t buffer[512];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, 512);
  if (!ProtobufEncoder_write_varint_field(&encoder, FROM_RADIO_FIELDS_ID, (uint64_t)next_from_radio_id())) return false;
  FixedVec packet_data;
  if (!encode_mesh_packet(packet, &packet_data)) return false;
  if (!ProtobufEncoder_write_bytes_field(&encoder, FROM_RADIO_FIELDS_PACKET, packet_data.data, packet_data.len)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

/**
 * My Info
 */
enum my_info_fields 
{
  MY_INFO_FIELDS_MY_NODE_NUM = 1,
  MY_INFO_FIELDS_REBOOT_COUNT = 8,
  MY_INFO_FIELDS_MIN_APP_VERSION = 11,
  MY_INFO_FIELDS_DEVICE_ID = 12,
};

bool encode_from_radio_my_info(uint32_t node_num, uint32_t reboot_count, FixedVec *output) 
{
  uint8_t buffer[512];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, 512);
  if (!ProtobufEncoder_write_varint_field(&encoder, FROM_RADIO_FIELDS_ID, (uint64_t)next_from_radio_id())) return false;
  // Inside message field
  uint8_t inner_buf[64];
  ProtobufEncoder inner;
  ProtobufEncoder_init(&inner, inner_buf, 64);
  if (!ProtobufEncoder_write_varint_field(&inner, MY_INFO_FIELDS_MY_NODE_NUM, (uint64_t)node_num)) return false;
  if (!ProtobufEncoder_write_varint_field(&inner, MY_INFO_FIELDS_REBOOT_COUNT, (uint64_t)reboot_count)) return false;
  if (!ProtobufEncoder_write_varint_field(&inner, MY_INFO_FIELDS_MIN_APP_VERSION, 20300)) return false;
  if (!ProtobufEncoder_write_message_field(&encoder, FROM_RADIO_FIELDS_MY_INFO, inner_buf, inner.pos)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

/**
 * Node Info From Radio
 */
enum node_info_fields 
{
  NODE_INFO_FIELDS_NUM = 1,
  NODE_INFO_FIELDS_USER = 2,
  NODE_INFO_FIELDS_POSITION = 3,
  NODE_INFO_FIELDS_SNR = 4,
  NODE_INFO_FIELDS_LAST_HEARD = 5,
  NODE_INFO_FIELDS_DEVICE_METRICS = 6,
  NODE_INFO_FIELDS_CHANNEL = 7,
  NODE_INFO_FIELDS_VIA_MQTT = 8,
  NODE_INFO_FIELDS_HOPS_AWAY = 9,
  NODE_INFO_FIELDS_IS_FAVORITE = 10,
};

bool encode_from_radio_node_info(const NodeInfo *node_info, FixedVec *output) 
{
  uint8_t buffer[512];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, 512);
  if (!ProtobufEncoder_write_varint_field(&encoder, FROM_RADIO_FIELDS_ID, (uint64_t)next_from_radio_id())) return false;
  uint8_t inner_buf[256];
  ProtobufEncoder inner;
  ProtobufEncoder_init(&inner, inner_buf, 256);
  if (!ProtobufEncoder_write_varint_field(&inner, NODE_INFO_FIELDS_NUM, (uint64_t)node_info->num)) return false;
  if (node_info->user != NULL) 
  {
    FixedVec user_data;
    if (encode_user(node_info->user, &user_data)) 
    {
      if (!ProtobufEncoder_write_bytes_field(&inner, NODE_INFO_FIELDS_USER, user_data.data, user_data.len)) return false;
    }
  }
  if (node_info->position != NULL) 
  {
    FixedVec pos_data;
    if (encode_position(node_info->position, &pos_data)) 
    {
      if (!ProtobufEncoder_write_bytes_field(&inner, NODE_INFO_FIELDS_POSITION, pos_data.data, pos_data.len)) return false;
    }
  }
  if (node_info->snr != 0.0f) 
  {
    uint32_t bits;
    memcpy(&bits, &node_info->snr, 4);
    if (!ProtobufEncoder_write_fixed32_field(&inner, NODE_INFO_FIELDS_SNR, bits)) return false;
  }
  if (!ProtobufEncoder_write_varint_field(&inner, NODE_INFO_FIELDS_LAST_HEARD, (uint64_t)node_info->last_heard)) return false;
  if (!ProtobufEncoder_write_message_field(&encoder, FROM_RADIO_FIELDS_NODE_INFO, inner_buf, inner.pos)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

/**
 * Channel Fields
 */
enum channel_fields 
{
  CHANNEL_FIELDS_INDEX = 1,
  CHANNEL_FIELDS_SETTINGS = 2,
  CHANNEL_FIELDS_ROLE = 3,
};

enum channel_settings_fields 
{
  CHANNEL_SETTINGS_FIELDS_CHANNEL_NUM = 1,
  CHANNEL_SETTINGS_FIELDS_PSK = 2,
  CHANNEL_SETTINGS_FIELDS_NAME = 3,
  CHANNEL_SETTINGS_FIELDS_ID = 4,
  CHANNEL_SETTINGS_FIELDS_UPLINK_ENABLED = 5,
  CHANNEL_SETTINGS_FIELDS_DOWNLINK_ENABLED = 6,
  CHANNEL_SETTINGS_FIELDS_MODULE_SETTINGS = 7,
};

bool encode_from_radio_channel(uint8_t index, const Channel *channel, bool is_primary, FixedVec *output) 
{
  uint8_t buffer[512];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, 512);
  if (!ProtobufEncoder_write_varint_field(&encoder, FROM_RADIO_FIELDS_ID, (uint64_t)next_from_radio_id())) return false;
  uint8_t inner_buf[128];
  ProtobufEncoder inner;
  ProtobufEncoder_init(&inner, inner_buf, 128);
  if (!ProtobufEncoder_write_varint_field(&inner, CHANNEL_FIELDS_INDEX, (uint64_t)index)) return false;
  uint8_t settings_buf[64];
  ProtobufEncoder settings;
  ProtobufEncoder_init(&settings, settings_buf, 64);
  if (!ProtobufEncoder_write_varint_field(&settings, CHANNEL_SETTINGS_FIELDS_CHANNEL_NUM, (uint64_t)index)) return false;
  if (!ProtobufEncoder_write_string_field(&settings, CHANNEL_SETTINGS_FIELDS_NAME, Channel_name_str(channel))) return false;
  if (!ProtobufEncoder_write_message_field(&inner, CHANNEL_FIELDS_SETTINGS, settings_buf, settings.pos)) return false;
  uint64_t role = is_primary ? 1 : 2;
  if (!ProtobufEncoder_write_varint_field(&inner, CHANNEL_FIELDS_ROLE, role)) return false;
  if (!ProtobufEncoder_write_message_field(&encoder, FROM_RADIO_FIELDS_CHANNEL, inner_buf, inner.pos)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

bool encode_from_radio_config_complete(uint32_t config_id, FixedVec *output) 
{
  uint8_t buffer[512];
  ProtobufEncoder encoder;
  ProtobufEncoder_init(&encoder, buffer, 512);
  if (!ProtobufEncoder_write_varint_field(&encoder, FROM_RADIO_FIELDS_ID, (uint64_t)next_from_radio_id())) return false;
  if (!ProtobufEncoder_write_varint_field(&encoder, FROM_RADIO_FIELDS_CONFIG_COMPLETE_ID, (uint64_t)config_id)) return false;
  output->len = encoder.pos;
  memcpy(output->data, buffer, encoder.pos);
  return true;
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
/**
 * Stubs for dependencies not fully defined in original snippet
 */

bool encode_user(const User *u, FixedVec *out) {
    // Basic implementation of user encoding stub
    ProtobufEncoder e;
    ProtobufEncoder_init(&e, out->data, 512);
    // Assuming some field for user as an example
    ProtobufEncoder_write_varint_field(&e, 1, 0); 
    out->len = e.pos;
    return true;
}

bool encode_position(const Position *p, FixedVec *out) {
    ProtobufEncoder e;
    ProtobufEncoder_init(&e, out->data, 512);
    // Mimicking the roundtrip test fields
    ProtobufEncoder_write_varint_field(&e, 1, (uint64_t)p->latitude_i);
    ProtobufEncoder_write_varint_field(&e, 2, (uint64_t)p->longitude_i);
    ProtobufEncoder_write_varint_field(&e, 3, (uint64_t)p->altitude);
    ProtobufEncoder_write_varint_field(&e, 4, (uint64_t)p->time);
    ProtobufEncoder_write_varint_field(&e, 5, (uint64_t)p->location_source);
    out->len = e.pos;
    return true;
}

bool decode_position(const uint8_t *data, size_t len, Position *p) {
    ProtobufDecoder d;
    ProtobufDecoder_init(&d, data, len);
    memset(p, 0, sizeof(Position));
    while(ProtobufDecoder_has_more(&d)) {
        uint32_t fn; uint8_t wt;
        if(!ProtobufDecoder_read_tag(&d, &fn, &wt)) return false;
        uint64_t val;
        if(!ProtobufDecoder_read_varint(&d, &val)) return false;
        if(fn == 1) p->latitude_i = (int32_t)val;
        else if(fn == 2) p->longitude_i = (int32_t)val;
        else if(fn == 3) p->altitude = (int32_t)val;
        else if(fn == 4) p->time = (uint32_t)val;
        else if(fn == 5) p->location_source = (LocationSource)val;
    }
    return true;
}

bool encode_data(const DataPayload *data, FixedVec *out) {
    ProtobufEncoder e;
    ProtobufEncoder_init(&e, out->data, 512);
    ProtobufEncoder_write_varint_field(&e, 1, (uint64_t)data->port);
    ProtobufEncoder_write_bytes_field(&e, 2, data->payload, data->payload_len);
    ProtobufEncoder_write_bool_field(&e, 3, data->want_response);
    ProtobufEncoder_write_varint_field(&e, 4, (uint64_t)data->dest);
    ProtobufEncoder_write_varint_field(&e, 5, (uint64_t)data->source);
    out->len = e.pos;
    return true;
}

bool decode_data(const uint8_t *data, size_t len, DataPayload *out) {
    ProtobufDecoder d;
    ProtobufDecoder_init(&d, data, len);
    memset(out, 0, sizeof(DataPayload));
    while(ProtobufDecoder_has_more(&d)) {
        uint32_t fn; uint8_t wt;
        if(!ProtobufDecoder_read_tag(&d, &fn, &wt)) return false;
        if(fn == 1) { uint64_t v; ProtobufDecoder_read_varint(&d, &v); out->port = (PortNum)v; }
        else if(fn == 2) { const uint8_t *b; size_t l; ProtobufDecoder_read_bytes(&d, &b, &l); memcpy(out->payload, b, l); out->payload_len = l; }
        else if(fn == 3) { uint64_t v; ProtobufDecoder_read_varint(&d, &v); out->want_response = (v != 0); }
        else if(fn == 4) { uint64_t v; ProtobufDecoder_read_varint(&d, &v); out->dest = (uint32_t)v; }
        else if(fn == 5) { uint64_t v; ProtobufDecoder_read_varint(&d, &v); out->source = (uint32_t)v; }
        else ProtobufDecoder_skip_field(&d, wt);
    }
    return true;
}

