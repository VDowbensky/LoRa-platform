#include "mod.h"

// meshtastic.c
#include "meshtastic.h"
#include <stdlib.h>

// PortNum conversion implementation
PortNum port_num_from_u32(uint32_t v) 
{
  switch (v) 
  {
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
Priority priority_from_u8(uint8_t v) 
{
  switch (v) 
  {
    case 1: return PRIORITY_MIN;
    case 10: return PRIORITY_BACKGROUND;
    case 70: return PRIORITY_RELIABLE;
    case 120: return PRIORITY_ACK;
    case 127: return PRIORITY_MAX;
    default: return PRIORITY_DEFAULT;
  }
}

// Position initialization
void position_init(Position* pos) 
{
  memset(pos, 0, sizeof(Position));
  pos->location_source = LOCATION_SOURCE_UNSET;
  pos->altitude_source = LOCATION_SOURCE_UNSET;
}

// DataPayload initialization
void data_payload_init(DataPayload* payload) 
{
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
void mesh_packet_init(MeshPacket* packet) 
{
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
MeshPacket* mesh_packet_clone(const MeshPacket* packet) 
{
  if (!packet) return NULL;
  MeshPacket* clone = (MeshPacket*)malloc(sizeof(MeshPacket));
  if (!clone) return NULL;
  memcpy(clone, packet, sizeof(MeshPacket));
  return clone;
}

// MeshPacket free
void mesh_packet_free(MeshPacket* packet) 
{
  if (packet) free(packet);
}

// PacketPayload initialization - encrypted
void packet_payload_init_encrypted(PacketPayload* payload) 
{
  payload->type = PACKET_PAYLOAD_ENCRYPTED;
  payload->data.encrypted.len = 0;
}

// PacketPayload initialization - decoded
void packet_payload_init_decoded(PacketPayload* payload, const DataPayload* data) 
{
  payload->type = PACKET_PAYLOAD_DECODED;
  memcpy(&payload->data.decoded, data, sizeof(DataPayload));
}

// PacketPayload clone
PacketPayload* packet_payload_clone(const PacketPayload* payload) 
{
  if (!payload) return NULL;
  PacketPayload* clone = (PacketPayload*)malloc(sizeof(PacketPayload));
  if (!clone) return NULL;
  memcpy(clone, payload, sizeof(PacketPayload));
  return clone;
}

// PacketPayload free
void packet_payload_free(PacketPayload* payload) 
{
  if (payload) free(payload);
}

// Vec helper functions
bool vec_u8_max_lora_payload_push(Vec_u8_MAX_LORA_PAYLOAD* vec, uint8_t byte) 
{
  if (!vec || vec->len >= MAX_LORA_PAYLOAD) return false;
  vec->data[vec->len++] = byte;
  return true;
}

bool vec_u8_max_lora_payload_extend(Vec_u8_MAX_LORA_PAYLOAD* vec, const uint8_t* data, size_t len) 
{
  if (!vec || !data || vec->len + len > MAX_LORA_PAYLOAD) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

bool vec_u8_max_message_size_push(Vec_u8_MAX_MESSAGE_SIZE* vec, uint8_t byte) 
{
  if (!vec || vec->len >= MAX_MESSAGE_SIZE) return false;
  vec->data[vec->len++] = byte;
  return true;
}

bool vec_u8_max_message_size_extend(Vec_u8_MAX_MESSAGE_SIZE* vec, const uint8_t* data, size_t len) 
{
  if (!vec || !data || vec->len + len > MAX_MESSAGE_SIZE) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

bool vec_u8_256_push(Vec_u8_256* vec, uint8_t byte) 
{
  if (!vec || vec->len >= 256) return false;
  vec->data[vec->len++] = byte;
  return true;
}

bool vec_u8_256_extend(Vec_u8_256* vec, const uint8_t* data, size_t len) 
{
  if (!vec || !data || vec->len + len > 256) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

bool vec_u8_40_extend(Vec_u8_40* vec, const uint8_t* data, size_t len) 
{
  if (!vec || !data || vec->len + len > 40) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

bool vec_u8_5_extend(Vec_u8_5* vec, const uint8_t* data, size_t len) 
{
  if (!vec || !data || vec->len + len > 5) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

bool vec_u8_64_extend(Vec_u8_64* vec, const uint8_t* data, size_t len) 
{
  if (!vec || !data || vec->len + len > 64) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

bool vec_u8_48_extend(Vec_u8_48* vec, const uint8_t* data, size_t len) 
{
  if (!vec || !data || vec->len + len > 48) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

bool vec_u8_128_extend(Vec_u8_128* vec, const uint8_t* data, size_t len) 
{
  if (!vec || !data || vec->len + len > 128) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

void vec_clear(void* vec, size_t element_size) 
{
  // Generic clear function - sets length to 0
  if (vec) *(size_t*)((uint8_t*)vec + element_size) = 0;
}

// Deque functions
void deque_init(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque) 
{
  if (!deque) return;
  deque->head = 0;
  deque->tail = 0;
  deque->count = 0;
}

bool deque_push_back(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque, const Vec_u8_MAX_MESSAGE_SIZE* item) 
{
  if (!deque || !item || deque->count >= 16) return false;
  memcpy(&deque->items[deque->tail], item, sizeof(Vec_u8_MAX_MESSAGE_SIZE));
  deque->tail = (deque->tail + 1) % 16;
  deque->count++;
  return true;
}

bool deque_pop_front(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque, Vec_u8_MAX_MESSAGE_SIZE* out) 
{
  if (!deque || !out || deque->count == 0) return false;
  memcpy(out, &deque->items[deque->head], sizeof(Vec_u8_MAX_MESSAGE_SIZE));
  deque->head = (deque->head + 1) % 16;
  deque->count--;
  return true;
}

void deque_clear(Deque_Vec_u8_MAX_MESSAGE_SIZE* deque) 
{
  if (!deque) return;
  deque->head = 0;
  deque->tail = 0;
  deque->count = 0;
}

size_t deque_len(const Deque_Vec_u8_MAX_MESSAGE_SIZE* deque) 
{
  return deque ? deque->count : 0;
}

bool deque_is_empty(const Deque_Vec_u8_MAX_MESSAGE_SIZE* deque) 
{
  return deque ? (deque->count == 0) : true;
}

// Helper functions for protobuf encoding/decoding
VarintResult decode_varint(const uint8_t* data, size_t len) 
{
  VarintResult result = {0, 0};
  if (!data || len == 0) return result;
  uint64_t value = 0;
  size_t shift = 0;
  size_t consumed = 0;
  for (size_t i = 0; i < len && i < 10; i++) 
  {
    uint8_t byte = data[i];
    value |= ((uint64_t)(byte & 0x7F)) << shift;
    consumed++;
    if ((byte & 0x80) == 0) break;
    shift += 7;
  }
  result.value = value;
  result.consumed = consumed;
  return result;
}

bool encode_varint(uint64_t value, Vec_u8_MAX_MESSAGE_SIZE* buf) 
{
  if (!buf) return false;
  while (value >= 0x80) 
  {
    if (!vec_u8_max_message_size_push(buf, (uint8_t)((value & 0x7F) | 0x80))) return false;
    value >>= 7;
  }
  return vec_u8_max_message_size_push(buf, (uint8_t)value);
}

bool write_tag(uint32_t field_num, uint32_t wire_type, Vec_u8_MAX_MESSAGE_SIZE* buf) 
{
  if (!buf) return false;
  uint64_t tag = ((uint64_t)field_num << 3) | wire_type;
  return encode_varint(tag, buf);
}

bool encode_varint_to_vec_64(uint64_t value, Vec_u8_64* buf) 
{
  if (!buf) return false;
  while (value >= 0x80) 
  {
    if (buf->len >= 64) return false;
    buf->data[buf->len++] = (uint8_t)((value & 0x7F) | 0x80);
    value >>= 7;
  }
  if (buf->len >= 64) return false;
  buf->data[buf->len++] = (uint8_t)value;
  return true;
}

bool write_tag_to_vec_64(uint32_t field_num, uint32_t wire_type, Vec_u8_64* buf) 
{
  if (!buf) return false;
  uint64_t tag = ((uint64_t)field_num << 3) | wire_type;
  return encode_varint_to_vec_64(tag, buf);
}

bool encode_varint_to_vec_48(uint64_t value, Vec_u8_48* buf) 
{
  if (!buf) return false;
  while (value >= 0x80) 
  {
    if (buf->len >= 48) return false;
    buf->data[buf->len++] = (uint8_t)((value & 0x7F) | 0x80);
    value >>= 7;
  }
  if (buf->len >= 48) return false;
  buf->data[buf->len++] = (uint8_t)value;
  return true;
}

bool write_tag_to_vec_48(uint32_t field_num, uint32_t wire_type, Vec_u8_48* buf) 
{
  if (!buf) return false;
  uint64_t tag = ((uint64_t)field_num << 3) | wire_type;
  return encode_varint_to_vec_48(tag, buf);
}

bool encode_varint_to_vec_128(uint64_t value, Vec_u8_128* buf) 
{
  if (!buf) return false;
  while (value >= 0x80) 
  {
    if (buf->len >= 128) return false;
    buf->data[buf->len++] = (uint8_t)((value & 0x7F) | 0x80);
    value >>= 7;
  }
  if (buf->len >= 128) return false;
  buf->data[buf->len++] = (uint8_t)value;
  return true;
}

bool write_tag_to_vec_128(uint32_t field_num, uint32_t wire_type, Vec_u8_128* buf) 
{
  if (!buf) return false;
  uint64_t tag = ((uint64_t)field_num << 3) | wire_type;
  return encode_varint_to_vec_128(tag, buf);
}

// Node ID formatting
void format_node_id(uint32_t node_id, char* out, size_t out_len) 
{
  if (!out || out_len < 9) return;
  snprintf(out, out_len, "!%08x", node_id);
}

// Parser functions
void meshtastic_parser_init(MeshtasticParser* parser) 
{
  if (!parser) return;
  parser->state = 0;
  parser->buffer.len = 0;
  parser->expected_length = 0;
  parser->current_index = 0;
}

void meshtastic_parser_reset(MeshtasticParser* parser) 
{
  meshtastic_parser_init(parser);
}

MeshtasticFrame* meshtastic_parser_feed(MeshtasticParser* parser, uint8_t byte) 
{
  if (!parser) return NULL;
  // State machine for parsing serial frames
  switch (parser->state) 
  {
    case 0: // Waiting for first sync byte
    if (byte == SERIAL_SYNC[0]) parser->state = 1;
    break;
        
    case 1: // Waiting for second sync byte
    if (byte == SERIAL_SYNC[1]) 
    {
      parser->state = 2;
      parser->buffer.len = 0;
    } 
    else parser->state = 0;
    break;
        
    case 2: // Reading length high byte
    parser->expected_length = (size_t)byte << 8;
    parser->state = 3;
    break;
        
    case 3: // Reading length low byte
    parser->expected_length |= byte;
    if (parser->expected_length > MAX_MESSAGE_SIZE) 
    {
      parser->state = 0;
      return NULL;
    }
    parser->state = 4;
    parser->current_index = 0;
    break;
        
    case 4: // Reading payload
    if (parser->buffer.len < MAX_MESSAGE_SIZE) 
    {
      parser->buffer.data[parser->buffer.len++] = byte;
      parser->current_index++;
      if (parser->current_index >= parser->expected_length) 
      {
        // Frame complete
        MeshtasticFrame* frame = (MeshtasticFrame*)malloc(sizeof(MeshtasticFrame));
        if (frame) memcpy(&frame->payload, &parser->buffer, sizeof(Vec_u8_MAX_MESSAGE_SIZE));
          parser->state = 0;
        parser->buffer.len = 0;
        return frame;
      }
    } 
    else parser->state = 0;
    break;
        
    default:
    parser->state = 0;
    break;
  }
  return NULL;
}

// MeshtasticHandler implementation
MeshtasticHandler* meshtastic_handler_new(uint32_t node_id) 
{
  MeshtasticHandler* handler = (MeshtasticHandler*)malloc(sizeof(MeshtasticHandler));
  if (!handler) return NULL;
  handler->node_id = node_id;
  channel_init(&handler->primary_channel);
  for (int i = 0; i < 7; i++) handler->secondary_channels[i] = NULL;
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

void meshtastic_handler_free(MeshtasticHandler* handler) 
{
  if (!handler) return;
  // Free secondary channels
  for (int i = 0; i < 7; i++) 
  {
    if (handler->secondary_channels[i]) free(handler->secondary_channels[i]);
  }
  // Free position if allocated
  if (handler->position) free(handler->position);
  // Free user if allocated
  if (handler->user) free(handler->user);
  free(handler);
}

void meshtastic_handler_set_channel_key(MeshtasticHandler* handler, const uint8_t* psk, size_t psk_len) 
{
  if (!handler || !psk) return;
  channel_set_key(&handler->primary_channel, psk, psk_len);
}

uint32_t meshtastic_handler_next_packet_id(MeshtasticHandler* handler) 
{
  if (!handler) return 0;
  handler->last_packet_id = handler->last_packet_id + 1;
  if (handler->last_packet_id == 0) handler->last_packet_id = 1;
  return handler->last_packet_id;
}

MeshPacket* meshtastic_handler_process_lora_packet(MeshtasticHandler* handler, const uint8_t* data, size_t len, int32_t rssi, float snr) 
{
  if (!handler || !data || len < LORA_HEADER_SIZE + MIC_SIZE) return NULL;
  MeshPacket* packet = parse_lora_packet(data, len);
  if (!packet) return NULL;
  packet->rx_rssi = rssi;
  packet->rx_snr = snr;
  MeshPacket* decrypted = meshtastic_handler_decrypt_packet(handler, packet);
  if (decrypted) handler->rx_count++;
  mesh_packet_free(packet);
  return decrypted;
}

MeshPacket* meshtastic_handler_decrypt_packet(const MeshtasticHandler* handler, const MeshPacket* packet) 
{
  if (!handler || !packet) return NULL;
  MeshPacket* result = mesh_packet_clone(packet);
  if (!result) return NULL;
  if (packet->payload.type == PACKET_PAYLOAD_ENCRYPTED) 
  {
    const Vec_u8_MAX_LORA_PAYLOAD* encrypted = &packet->payload.data.encrypted;
    const Channel* channel;
    if (packet->channel == 0) channel = &handler->primary_channel;
    else 
    {
      size_t idx = packet->channel - 1;
      if (idx >= 7 || !handler->secondary_channels[idx]) 
      {
        mesh_packet_free(result);
        return NULL;
      }
      channel = handler->secondary_channels[idx];
    }
    Vec_u8_MAX_LORA_PAYLOAD decrypted;
    decrypted.len = 0;
    if (!channel_decrypt(channel, packet->id, packet->from, encrypted, &decrypted)) 
    {
      mesh_packet_free(result);
      return NULL;
    }
    DataPayload* data = protobuf_decode_data(&decrypted);
    if (data) 
    {
      packet_payload_init_decoded(&result->payload, data);
      free(data);
    }
  }
  return result;
}

bool meshtastic_handler_create_packet(MeshtasticHandler* handler, uint32_t to, PortNum port, const uint8_t* payload, size_t payload_len, bool want_ack, Vec_u8_256* out) 
{
  if (!handler || !payload || !out) return false;
  uint32_t packet_id = meshtastic_handler_next_packet_id(handler);
  DataPayload data;
  data_payload_init(&data);
  data.port = port;
  if (!vec_u8_max_lora_payload_extend(&data.payload, payload, payload_len)) return false;
  data.want_response = want_ack;
  data.source = handler->node_id;
  data.dest = to;
  Vec_u8_MAX_LORA_PAYLOAD encoded;
  encoded.len = 0;
  if (!protobuf_encode_data(&data, &encoded)) return false;
  Vec_u8_MAX_LORA_PAYLOAD encrypted;
  encrypted.len = 0;
  if (!channel_encrypt(&handler->primary_channel, packet_id, handler->node_id, &encoded, &encrypted)) return false;
  if (!build_lora_packet(handler->node_id, to, packet_id, 0, DEFAULT_HOP_LIMIT, want_ack, &encrypted, out)) return false; 
  handler->tx_count++;
  return true;
}

bool meshtastic_handler_create_text_message(MeshtasticHandler* handler, uint32_t to, const char* text, Vec_u8_256* out) 
{
  if (!handler || !text || !out) return false;
  return meshtastic_handler_create_packet(handler, to, PORT_NUM_TEXT_MESSAGE, (const uint8_t*)text, strlen(text), true, out);
}

bool meshtastic_handler_create_position_packet(MeshtasticHandler* handler, Vec_u8_256* out) 
{
  if (!handler || !out || !handler->position) return false;
  Vec_u8_MAX_LORA_PAYLOAD encoded;
  encoded.len = 0;
  if (!protobuf_encode_position(handler->position, &encoded)) return false;
  return meshtastic_handler_create_packet(handler, 0xFFFFFFFF, PORT_NUM_POSITION, encoded.data, encoded.len, false, out);
}

bool meshtastic_handler_create_node_info_packet(MeshtasticHandler* handler, Vec_u8_256* out) 
{
  if (!handler || !out || !handler->user) return false;
  Vec_u8_MAX_LORA_PAYLOAD encoded;
  encoded.len = 0;
  if (!protobuf_encode_user(handler->user, &encoded)) return false;
  return meshtastic_handler_create_packet(handler, 0xFFFFFFFF, PORT_NUM_NODE_INFO, encoded.data, encoded.len, false, out);
}

bool meshtastic_handler_parse_serial_frame(const MeshtasticHandler* handler, const uint8_t* data, size_t len, Vec_u8_MAX_MESSAGE_SIZE* out) 
{
  if (!handler || !data || !out || len < 4) return false;
  if (data[0] != SERIAL_SYNC[0] || data[1] != SERIAL_SYNC[1]) return false;
  size_t payload_len = ((size_t)data[2] << 8) | data[3];
  if (len < 4 + payload_len) return false;
  out->len = 0;
  return vec_u8_max_message_size_extend(out, data + 4, payload_len);
}

bool meshtastic_handler_build_serial_frame(const MeshtasticHandler* handler, const uint8_t* payload, size_t payload_len, Vec_u8_MAX_MESSAGE_SIZE* out) 
{
  if (!handler || !payload || !out) return false;
  out->len = 0;
  if (!vec_u8_max_message_size_push(out, SERIAL_SYNC[0])) return false;
  if (!vec_u8_max_message_size_push(out, SERIAL_SYNC[1])) return false;
  uint16_t len_u16 = (uint16_t)payload_len;
  if (!vec_u8_max_message_size_push(out, (uint8_t)(len_u16 >> 8))) return false;
  if (!vec_u8_max_message_size_push(out, (uint8_t)(len_u16 & 0xFF))) return false;
  return vec_u8_max_message_size_extend(out, payload, payload_len);
}

MeshtasticFrame* meshtastic_handler_feed_serial(MeshtasticHandler* handler, uint8_t byte) 
{
  if (!handler) return NULL;
  return meshtastic_parser_feed(&handler->parser, byte);
}

bool meshtastic_handler_build_lora_packet(MeshtasticHandler* handler, const MeshtasticFrame* frame, Vec_u8_256* out) 
{
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
  while (idx < mesh_packet_len) 
  {
    if (idx >= mesh_packet_len) break;
    uint8_t tag = mesh_packet_data[idx];
    idx++;
    if (idx > mesh_packet_len) break;
    uint32_t field_num = tag >> 3;
    uint32_t wire_type = tag & 0x07;
    if (field_num == 2 && wire_type == 0) 
    { // to field
      VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
      to = (uint32_t)result.value;
      idx += result.consumed;
    }
    else if (field_num == 4 && wire_type == 0) 
    { // channel field
      VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
      channel = (uint8_t)result.value;
      idx += result.consumed;
    }
    else if (field_num == 6 && wire_type == 0) 
    { // hop_limit field
      VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
      hop_limit = (uint8_t)result.value;
      idx += result.consumed;
    }
    else if (field_num == 7 && wire_type == 0) 
    { // want_ack field
      VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
      want_ack = result.value != 0;
      idx += result.consumed;
    }
    else if (field_num == 8 && wire_type == 2) 
    { // decoded field (Data message)
      if (idx >= mesh_packet_len) break;
      size_t data_len = mesh_packet_data[idx];
      idx++;
      if (idx + data_len > mesh_packet_len) break;
      const uint8_t* data_msg = mesh_packet_data + idx;
      // Parse Data message fields
      size_t data_idx = 0;
      while (data_idx < data_len) 
      {
        if (data_idx >= data_len) break;
        uint8_t dtag = data_msg[data_idx];
        data_idx++;
        if (data_idx > data_len) break;
        uint32_t dfield = dtag >> 3;
        uint32_t dwire = dtag & 0x07;
        if (dfield == 1 && dwire == 0) 
        { // portnum field
          VarintResult result = decode_varint(data_msg + data_idx, data_len - data_idx);
          port = port_num_from_u32((uint32_t)result.value);
          data_idx += result.consumed;
        }
        else if (dfield == 2 && dwire == 2) 
        { // payload field
          if (data_idx >= data_len) break;
          size_t plen = data_msg[data_idx];
          data_idx++;
          if (data_idx + plen > data_len) break;
          
          vec_u8_max_lora_payload_extend(&inner_payload, data_msg + data_idx, plen);
          has_payload = true;
          data_idx += plen;
        }
        else if (dwire == 0) 
        { // Skip varint
          VarintResult result = decode_varint(data_msg + data_idx, data_len - data_idx);
          data_idx += result.consumed;
        }
        else if (dwire == 2) 
        { // Skip length-delimited
          if (data_idx >= data_len) break;
          size_t skip_len = data_msg[data_idx];
          data_idx += 1 + skip_len;
        }
        else break;
      }
      idx += data_len;
    }
    else if (wire_type == 0) 
    { // Skip varint
      VarintResult result = decode_varint(mesh_packet_data + idx, mesh_packet_len - idx);
      idx += result.consumed;
    }
    else if (wire_type == 2) 
    { // Skip length-delimited
      if (idx >= mesh_packet_len) break;
      size_t skip_len = mesh_packet_data[idx];
      idx += 1 + skip_len;
    }
    else break;
  }
  if (!has_payload) return false;
  return meshtastic_handler_create_packet(handler, to, port, inner_payload.data, inner_payload.len, want_ack, out);
}

void meshtastic_handler_reset_parser(MeshtasticHandler* handler) 
{
  if (!handler) return;
  meshtastic_parser_reset(&handler->parser);
}

ToRadioResponse* meshtastic_handler_process_toradio(MeshtasticHandler* handler, const MeshtasticFrame* frame) 
{
  if (!handler || !frame || frame->payload.len == 0) return NULL;
  const uint8_t* payload = frame->payload.data;
  size_t payload_len = frame->payload.len;
  if (payload_len < 2) return NULL;
  uint8_t field_tag = payload[0];
  uint32_t field_num = field_tag >> 3;
  uint32_t wire_type = field_tag & 0x07;
  if (field_num == 1 && wire_type == 2) 
  { // packet field
    Vec_u8_256 lora_packet;
    lora_packet.len = 0;
    if (meshtastic_handler_build_lora_packet(handler, frame, &lora_packet)) 
    {
      ToRadioResponse* response = (ToRadioResponse*)malloc(sizeof(ToRadioResponse));
      if (response) 
      {
        response->type = TO_RADIO_RESPONSE_LORA_PACKET;
        memcpy(&response->data.lora_packet, &lora_packet, sizeof(Vec_u8_256));
      }
      return response;
    }
  }
  else if (field_num == 3 && wire_type == 0) 
  { // want_config_id field
    VarintResult result = decode_varint(payload + 1, payload_len - 1);
    uint32_t config_id = (uint32_t)result.value;
    return meshtastic_handler_build_config_response(handler, config_id);
  }
  else if (field_num == 4 && wire_type == 0) 
  { // disconnect field
    meshtastic_handler_reset_parser(handler);
    return NULL;
  }
  return NULL;
}

ToRadioResponse* meshtastic_handler_build_config_response(MeshtasticHandler* handler, uint32_t config_id) 
{
  if (!handler) return NULL;
  // Clear pending responses
  deque_clear(&handler->pending_responses);
  handler->config_request_id = config_id;
  handler->config_channel_index = 0;
  // Encode and queue my_info
  Vec_u8_MAX_MESSAGE_SIZE my_info;
  my_info.len = 0;
  if (meshtastic_handler_encode_privacy_myinfo(handler, &my_info)) deque_push_back(&handler->pending_responses, &my_info);
  // Encode and queue node_info
  Vec_u8_MAX_MESSAGE_SIZE node_info;
  node_info.len = 0;
  if (meshtastic_handler_encode_privacy_nodeinfo(handler, &node_info)) deque_push_back(&handler->pending_responses, &node_info);
  // Encode and queue primary channel config
  Vec_u8_MAX_MESSAGE_SIZE channel_config;
  channel_config.len = 0;
  if (meshtastic_handler_encode_channel_config(handler, 0, &channel_config)) deque_push_back(&handler->pending_responses, &channel_config);
  // Encode and queue secondary channel configs
  for (int i = 0; i < 7; i++) 
  {
    if (handler->secondary_channels[i]) 
    {
      Vec_u8_MAX_MESSAGE_SIZE sec_channel_config;
      sec_channel_config.len = 0;
      if (meshtastic_handler_encode_channel_config(handler, (uint8_t)(i + 1), &sec_channel_config)) deque_push_back(&handler->pending_responses, &sec_channel_config);
    }
  }
  // Encode and queue config complete
  Vec_u8_MAX_MESSAGE_SIZE complete;
  complete.len = 0;
  if (meshtastic_handler_encode_config_complete(handler, config_id, &complete)) deque_push_back(&handler->pending_responses, &complete);
  // Return first pending response
  return meshtastic_handler_poll_pending_response(handler);
}

ToRadioResponse* meshtastic_handler_poll_pending_response(MeshtasticHandler* handler) 
{
  if (!handler) return NULL;
  Vec_u8_MAX_MESSAGE_SIZE response;
  response.len = 0;
  if (deque_pop_front(&handler->pending_responses, &response)) 
  {
    ToRadioResponse* result = (ToRadioResponse*)malloc(sizeof(ToRadioResponse));
    if (result) 
    {
      result->type = TO_RADIO_RESPONSE_FROM_RADIO;
      memcpy(&result->data.from_radio, &response, sizeof(Vec_u8_MAX_MESSAGE_SIZE));
    }
    return result;
  }
  return NULL;
}

bool meshtastic_handler_has_pending_responses(const MeshtasticHandler* handler) 
{
  if (!handler) return false;
  return !deque_is_empty(&handler->pending_responses);
}

size_t meshtastic_handler_pending_response_count(const MeshtasticHandler* handler) 
{
  if (!handler) return 0;
  return deque_len(&handler->pending_responses);
}

bool meshtastic_handler_encode_privacy_myinfo(const MeshtasticHandler* handler, Vec_u8_MAX_MESSAGE_SIZE* out) 
{
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

bool meshtastic_handler_encode_privacy_nodeinfo(const MeshtasticHandler* handler, Vec_u8_MAX_MESSAGE_SIZE* out) 
{
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
  if (handler->user) 
  {
    long_name = handler->user->long_name.data;
    long_name_len = handler->user->long_name.len;
  } 
  else 
  {
    long_name = (const uint8_t*)"LunarNode";
    long_name_len = 9;
  }
  if (!write_tag_to_vec_64(2, WIRE_LEN, &user)) return false;
  if (!encode_varint_to_vec_64(long_name_len, &user)) return false;
  if (!vec_u8_64_extend(&user, long_name, long_name_len)) return false;
   // short_name field (field 3, length-delimited)
  const uint8_t* short_name;
  size_t short_name_len;
  if (handler->user) 
  {
    short_name = handler->user->short_name.data;
    short_name_len = handler->user->short_name.len;
  } 
  else 
  {
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

bool meshtastic_handler_encode_channel_config(const MeshtasticHandler* handler, uint8_t index, Vec_u8_MAX_MESSAGE_SIZE* out) 
{
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
  if (name_len > 0) 
  {
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

bool meshtastic_handler_encode_config_complete(const MeshtasticHandler* handler, uint32_t config_id, Vec_u8_MAX_MESSAGE_SIZE* out) 
{
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

bool meshtastic_handler_handle_admin_message(MeshtasticHandler* handler, const uint8_t* payload, size_t payload_len, Vec_u8_MAX_MESSAGE_SIZE* out) 
{
  if (!handler || !payload || !out || payload_len == 0) return false;
  uint8_t tag = payload[0];
  uint32_t field_num = tag >> 3;
  switch (field_num) 
  {
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

/* // Channel functions (stub implementations - would be in channel.c)
void channel_init(Channel* channel) 
{
  if (!channel) return;
  memset(channel, 0, sizeof(Channel));
}

void channel_set_key(Channel* channel, const uint8_t* psk, size_t psk_len) 
{
  if (!channel || !psk) return;
  size_t copy_len = psk_len < 32 ? psk_len : 32;
  memcpy(channel->key, psk, copy_len);
} */

/* bool channel_decrypt(const Channel* channel, uint32_t packet_id, uint32_t from, const Vec_u8_MAX_LORA_PAYLOAD* encrypted, Vec_u8_MAX_LORA_PAYLOAD* out) {
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
} */

const char* channel_name_str(const Channel* channel) 
{
  if (!channel) return "";
  return (const char*)channel->name;
}

/* // Packet functions (stub implementations - would be in packet.c)
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
} */
