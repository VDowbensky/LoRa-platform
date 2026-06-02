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
typedef enum 
{
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
typedef enum 
{
  PRIORITY_UNSET = 0,
  PRIORITY_MIN = 1,
  PRIORITY_BACKGROUND = 10,
  PRIORITY_DEFAULT = 64,
  PRIORITY_RELIABLE = 70,
  PRIORITY_ACK = 120,
  PRIORITY_MAX = 127
} Priority;

// LocationSource enum
typedef enum 
{
  LOCATION_SOURCE_UNSET = 0,
  LOCATION_SOURCE_MANUAL = 1,
  LOCATION_SOURCE_INTERNAL_GPS = 2,
  LOCATION_SOURCE_EXTERNAL_GPS = 3
} LocationSource;

// HardwareModel enum
typedef enum 
{
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
typedef enum 
{
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
typedef struct 
{
  uint8_t data[MAX_LORA_PAYLOAD];
  size_t len;
} Vec_u8_MAX_LORA_PAYLOAD;

typedef struct 
{
  uint8_t data[MAX_MESSAGE_SIZE];
  size_t len;
} Vec_u8_MAX_MESSAGE_SIZE;

typedef struct 
{
  uint8_t data[256];
  size_t len;
} Vec_u8_256;

typedef struct 
{
  uint8_t data[40];
  size_t len;
} Vec_u8_40;

typedef struct 
{
  uint8_t data[5];
  size_t len;
} Vec_u8_5;

typedef struct 
{
  uint8_t data[64];
  size_t len;
} Vec_u8_64;

typedef struct 
{
  uint8_t data[48];
  size_t len;
} Vec_u8_48;

typedef struct 
{
  uint8_t data[128];
  size_t len;
} Vec_u8_128;

// Position structure
struct Position 
{
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
struct User 
{
  uint8_t id[8];
  Vec_u8_40 long_name;
  Vec_u8_5 short_name;
  HardwareModel hw_model;
  bool is_licensed;
  Role role;
};

// DataPayload structure
struct DataPayload 
{
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
typedef enum 
{
  PACKET_PAYLOAD_ENCRYPTED,
  PACKET_PAYLOAD_DECODED
} PacketPayloadType;

struct PacketPayload 
{
  PacketPayloadType type;
  union 
  {
    Vec_u8_MAX_LORA_PAYLOAD encrypted;
    DataPayload decoded;
  } data;
};

// MeshPacket structure
struct MeshPacket 
{
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
struct NodeInfo 
{
  uint32_t num;
  User* user;  // NULL if not present
  Position* position;  // NULL if not present
  uint32_t last_heard;
  float snr;
};

// Deque structure for pending responses
typedef struct 
{
  Vec_u8_MAX_MESSAGE_SIZE items[16];
  size_t head;
  size_t tail;
  size_t count;
} Deque_Vec_u8_MAX_MESSAGE_SIZE;

// Channel structure placeholder (from channel module)
struct Channel 
{
  uint8_t key[32];
  uint8_t name[12];
  size_t name_len;
  // Additional channel fields would be defined in channel.h
};

// MeshtasticParser structure placeholder (implementation details)
struct MeshtasticParser 
{
  uint8_t state;
  Vec_u8_MAX_MESSAGE_SIZE buffer;
  size_t expected_length;
  size_t current_index;
};

// MeshtasticFrame structure
struct MeshtasticFrame 
{
  Vec_u8_MAX_MESSAGE_SIZE payload;
};

// ToRadioResponse structure (tagged union)
typedef enum 
{
  TO_RADIO_RESPONSE_LORA_PACKET,
  TO_RADIO_RESPONSE_FROM_RADIO
} ToRadioResponseType;

typedef struct 
{
  ToRadioResponseType type;
  union 
  {
    Vec_u8_256 lora_packet;
    Vec_u8_MAX_MESSAGE_SIZE from_radio;
  } data;
} ToRadioResponse;

// MeshtasticHandler structure
struct MeshtasticHandler 
{
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


typedef struct 
{
  uint64_t value;
  size_t consumed;
} VarintResult;

//Function declarations

PortNum port_num_from_u32(uint32_t v);
// Priority conversion implementation
Priority priority_from_u8(uint8_t v);
// Position initialization
void position_init(Position* pos);
// DataPayload initialization
void data_payload_init(DataPayload* payload);
// MeshPacket initialization
void mesh_packet_init(MeshPacket* packet);
// MeshPacket clone
MeshPacket* mesh_packet_clone(const MeshPacket* packet);
// MeshPacket free
void mesh_packet_free(MeshPacket* packet);
// PacketPayload initialization - encrypted
void packet_payload_init_encrypted(PacketPayload* payload);
// PacketPayload initialization - decoded
void packet_payload_init_decoded(PacketPayload* payload, const DataPayload* data);
// PacketPayload clone
PacketPayload* packet_payload_clone(const PacketPayload* payload);
// PacketPayload free
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
// Helper functions for protobuf encoding/decoding
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
// MeshtasticHandler implementation
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
const char* channel_name_str(const Channel* channel);

#endif // MESHTASTIC_H
