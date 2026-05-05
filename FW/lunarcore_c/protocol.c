#include "protocol.h"

// Constants
static const uint8_t SYNC[2] = {0xAA, 0x55};
static const uint8_t END = 0x0D;
#define MAX_DATA_SIZE 255
#define MAX_FRAME_SIZE (2 + 2 + 1 + 1 + MAX_DATA_SIZE + 2 + 1) // 264

// Forward declaration for external type
struct RadioConfig;

// Command enum
typedef enum 
{
  COMMAND_PING = 0x01,
  COMMAND_PONG = 0x02,
  COMMAND_CONFIGURE = 0x10,
  COMMAND_CONFIG_ACK = 0x11,
  COMMAND_TRANSMIT = 0x20,
  COMMAND_TX_DONE = 0x21,
  COMMAND_TX_ERROR = 0x22,
  COMMAND_RECEIVE = 0x30,
  COMMAND_GET_STATS = 0x40,
  COMMAND_STATS_RESPONSE = 0x41,
  COMMAND_CAD = 0x50,
  COMMAND_CAD_RESULT = 0x51,
  COMMAND_RESET = 0xF0,
  COMMAND_VERSION = 0xF1,
  COMMAND_VERSION_RESPONSE = 0xF2,
  COMMAND_ERROR = 0xFF
} Command;

// Convert byte to Command enum (returns true if valid, false otherwise)
bool command_from_byte(uint8_t b, Command *out_command) 
{
  switch (b) 
  {
    case 0x01: *out_command = COMMAND_PING; return true;
    case 0x02: *out_command = COMMAND_PONG; return true;
    case 0x10: *out_command = COMMAND_CONFIGURE; return true;
    case 0x11: *out_command = COMMAND_CONFIG_ACK; return true;
    case 0x20: *out_command = COMMAND_TRANSMIT; return true;
    case 0x21: *out_command = COMMAND_TX_DONE; return true;
    case 0x22: *out_command = COMMAND_TX_ERROR; return true;
    case 0x30: *out_command = COMMAND_RECEIVE; return true;
    case 0x40: *out_command = COMMAND_GET_STATS; return true;
    case 0x41: *out_command = COMMAND_STATS_RESPONSE; return true;
    case 0x50: *out_command = COMMAND_CAD; return true;
    case 0x51: *out_command = COMMAND_CAD_RESULT; return true;
    case 0xF0: *out_command = COMMAND_RESET; return true;
    case 0xF1: *out_command = COMMAND_VERSION; return true;
    case 0xF2: *out_command = COMMAND_VERSION_RESPONSE; return true;
    case 0xFF: *out_command = COMMAND_ERROR; return true;
    default: return false;
  }
}

// Vec equivalent for data storage
typedef struct 
{
  uint8_t data[MAX_DATA_SIZE];
  size_t len;
} DataVec;

typedef struct 
{
  uint8_t data[MAX_FRAME_SIZE];
  size_t len;
} FrameVec;

// Frame struct
typedef struct 
{
  Command command;
  uint8_t sequence;
  DataVec data;
} Frame;

// Create new frame with empty data
Frame frame_new(Command command, uint8_t sequence) 
{
  Frame frame;
  frame.command = command;
  frame.sequence = sequence;
  frame.data.len = 0;
  return frame;
}

// Create frame with data (returns NULL if data too large)
Frame* frame_with_data(Command command, uint8_t sequence, const uint8_t *data, size_t data_len, Frame *out_frame) 
{
  if (data_len > MAX_DATA_SIZE) return NULL;
  out_frame->command = command;
  out_frame->sequence = sequence;
  out_frame->data.len = data_len;
  for (size_t i = 0; i < data_len; i++) out_frame->data.data[i] = data[i];
  return out_frame;
}

// CRC16 table
static const uint16_t CRC16_TABLE[256] = 
{
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

// Calculate CRC16
static inline uint16_t crc16(const uint8_t *data, size_t len) 
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) 
  {
    size_t index = ((crc >> 8) ^ data[i]) & 0xFF;
    crc = (crc << 8) ^ CRC16_TABLE[index];
  }
  return crc;
}

// Encode frame into buffer
void frame_encode(const Frame *frame, FrameVec *out_buf) 
{
  out_buf->len = 0;
  // Add sync bytes
  out_buf->data[out_buf->len++] = SYNC[0];
  out_buf->data[out_buf->len++] = SYNC[1];
  // Add length
  uint16_t len = (uint16_t)frame->data.len;
  out_buf->data[out_buf->len++] = (uint8_t)len;
  out_buf->data[out_buf->len++] = (uint8_t)(len >> 8);
  // Add command and sequence
  out_buf->data[out_buf->len++] = (uint8_t)frame->command;
  out_buf->data[out_buf->len++] = frame->sequence;
  // Add data
  for (size_t i = 0; i < frame->data.len; i++) out_buf->data[out_buf->len++] = frame->data.data[i];
  // Calculate and add CRC (from byte 4 onwards: command, sequence, and data)
  uint16_t crc = crc16(&out_buf->data[4], out_buf->len - 4);
  out_buf->data[out_buf->len++] = (uint8_t)crc;
  out_buf->data[out_buf->len++] = (uint8_t)(crc >> 8);
      // Add end marker
  out_buf->data[out_buf->len++] = END;
}

// Parser state enum
typedef enum 
{
  PARSER_STATE_WAIT_SYNC1,
  PARSER_STATE_WAIT_SYNC2,
  PARSER_STATE_WAIT_LEN_LOW,
  PARSER_STATE_WAIT_LEN_HIGH,
  PARSER_STATE_WAIT_COMMAND,
  PARSER_STATE_WAIT_SEQUENCE,
  PARSER_STATE_WAIT_DATA,
  PARSER_STATE_WAIT_CRC_LOW,
  PARSER_STATE_WAIT_CRC_HIGH,
  PARSER_STATE_WAIT_END
} ParserState;

// Frame parser struct
typedef struct 
{
  ParserState state;
  uint16_t data_len;
  uint16_t data_idx;
  uint8_t command;
  uint8_t sequence;
  DataVec data;
  uint8_t crc_low;
} FrameParser;

// Create new parser
FrameParser frame_parser_new(void) 
{
  FrameParser parser;
  parser.state = PARSER_STATE_WAIT_SYNC1;
  parser.data_len = 0;
  parser.data_idx = 0;
  parser.command = 0;
  parser.sequence = 0;
  parser.data.len = 0;
  parser.crc_low = 0;
  return parser;
}

// Reset parser state
void frame_parser_reset(FrameParser *parser) 
{
  parser->state = PARSER_STATE_WAIT_SYNC1;
  parser->data.len = 0;
  parser->data_len = 0;
  parser->data_idx = 0;
}

// Feed byte to parser (returns pointer to frame if complete, NULL otherwise)
Frame* frame_parser_feed(FrameParser *parser, uint8_t byte, Frame *out_frame) 
{
  switch (parser->state) 
  {
    case PARSER_STATE_WAIT_SYNC1:
    if (byte == SYNC[0]) parser->state = PARSER_STATE_WAIT_SYNC2;
    break;
            
    case PARSER_STATE_WAIT_SYNC2:
    if (byte == SYNC[1]) parser->state = PARSER_STATE_WAIT_LEN_LOW;
    else if (byte == SYNC[0]) 
    {
      // Stay in WAIT_SYNC2 state (do nothing)
    } 
    else frame_parser_reset(parser);
    break;
            
    case PARSER_STATE_WAIT_LEN_LOW:
    parser->data_len = (uint16_t)byte;
    parser->state = PARSER_STATE_WAIT_LEN_HIGH;
    break;
            
    case PARSER_STATE_WAIT_LEN_HIGH:
    parser->data_len |= ((uint16_t)byte) << 8;
    if (parser->data_len > MAX_DATA_SIZE) 
    {
      frame_parser_reset(parser);
      return NULL;
    }
    parser->state = PARSER_STATE_WAIT_COMMAND;
    break;
            
    case PARSER_STATE_WAIT_COMMAND:
    parser->command = byte;
    parser->state = PARSER_STATE_WAIT_SEQUENCE;
    break;
            
    case PARSER_STATE_WAIT_SEQUENCE:
    parser->sequence = byte;
    parser->data.len = 0;
    parser->data_idx = 0;
    if (parser->data_len == 0) parser->state = PARSER_STATE_WAIT_CRC_LOW;
    else parser->state = PARSER_STATE_WAIT_DATA;
    break;
            
    case PARSER_STATE_WAIT_DATA:
    parser->data.data[parser->data.len++] = byte;
    parser->data_idx++;
    if (parser->data_idx >= parser->data_len) parser->state = PARSER_STATE_WAIT_CRC_LOW;
    break;
            
    case PARSER_STATE_WAIT_CRC_LOW:
    parser->crc_low = byte;
    parser->state = PARSER_STATE_WAIT_CRC_HIGH;
    break;
            
    case PARSER_STATE_WAIT_CRC_HIGH: 
    {
      uint16_t received_crc = ((uint16_t)parser->crc_low) | (((uint16_t)byte) << 8);
      // Build CRC data buffer (command + sequence + data)
      uint8_t crc_data[258];  // 1 + 1 + 256
      size_t crc_data_len = 0;
      crc_data[crc_data_len++] = parser->command;
      crc_data[crc_data_len++] = parser->sequence;
      for (size_t i = 0; i < parser->data.len; i++) crc_data[crc_data_len++] = parser->data.data[i];
      uint16_t calculated_crc = crc16(crc_data, crc_data_len);
      if (received_crc == calculated_crc) parser->state = PARSER_STATE_WAIT_END;
      else frame_parser_reset(parser);
      break;
    }
        
    case PARSER_STATE_WAIT_END:
    if (byte == END) 
    {
      // Try to convert command byte to Command enum
      Command cmd;
      if (command_from_byte(parser->command, &cmd)) 
      {
        // Build frame
        out_frame->command = cmd;
        out_frame->sequence = parser->sequence;
        out_frame->data.len = parser->data.len;
        memcpy(out_frame->data.data, parser->data.data, parser->data.len);
        frame_parser_reset(parser);
        return out_frame;
      }
    }
    frame_parser_reset(parser);
    break;
  }
  return NULL;
}

// Builder functions

// Build pong frame
Frame build_pong(uint8_t sequence) 
{
  return frame_new(COMMAND_PONG, sequence);
}

// Build config acknowledgement frame
Frame build_config_ack(uint8_t sequence) 
{
  return frame_new(COMMAND_CONFIG_ACK, sequence);
}

// Build tx done frame
Frame build_tx_done(uint8_t sequence) 
{
  return frame_new(COMMAND_TX_DONE, sequence);
}

// Build tx error frame (returns NULL if data too large)
Frame* build_tx_error(uint8_t sequence, uint8_t error_code, Frame *out_frame) 
{
  uint8_t data[1] = {error_code};
  return frame_with_data(COMMAND_TX_ERROR, sequence, data, 1, out_frame);
}

// Build receive frame (returns NULL if data too large)
Frame* build_receive(int16_t rssi, int8_t snr, const uint8_t *data, size_t data_len, Frame *out_frame) 
{
  if (data_len > MAX_DATA_SIZE - 4) return NULL;
  out_frame->command = COMMAND_RECEIVE;
  out_frame->sequence = 0;
  out_frame->data.len = 0;
  // Add RSSI (2 bytes, little-endian)
  out_frame->data.data[out_frame->data.len++] = (uint8_t)rssi;
  out_frame->data.data[out_frame->data.len++] = (uint8_t)(rssi >> 8);
  // Add SNR (1 byte)
  out_frame->data.data[out_frame->data.len++] = (uint8_t)snr;
  // Add reserved byte (0)
  out_frame->data.data[out_frame->data.len++] = 0;
  // Add data
  for (size_t i = 0; i < data_len; i++) out_frame->data.data[out_frame->data.len++] = data[i];
  return out_frame;
}

// Build CAD result frame (returns NULL if data too large)
Frame* build_cad_result(uint8_t sequence, bool detected, Frame *out_frame) 
{
  uint8_t data[1] = {detected ? 1 : 0};
  return frame_with_data(COMMAND_CAD_RESULT, sequence, data, 1, out_frame);
}

// Build version response frame (returns NULL if version string too large)
Frame* build_version_response(uint8_t sequence, const char *version, Frame *out_frame) 
{
  size_t version_len = strlen(version);
  return frame_with_data(COMMAND_VERSION_RESPONSE, sequence, (const uint8_t*)version, version_len, out_frame);
}

// Build error frame (returns NULL if message too large)
Frame* build_error(uint8_t sequence, const char *message, Frame *out_frame) 
{
  size_t message_len = strlen(message);
  return frame_with_data(COMMAND_ERROR, sequence, (const uint8_t*)message, message_len, out_frame);
}

// Build stats response frame (returns NULL if data too large)
Frame* build_stats_response(uint8_t sequence,uint32_t tx_packets,uint32_t rx_packets,uint32_t tx_errors,uint32_t rx_errors,Frame *out_frame) 
{
  uint8_t data[16];
  // Copy tx_packets (little-endian)
  data[0] = (uint8_t)(tx_packets & 0xFF);
  data[1] = (uint8_t)((tx_packets >> 8) & 0xFF);
  data[2] = (uint8_t)((tx_packets >> 16) & 0xFF);
  data[3] = (uint8_t)((tx_packets >> 24) & 0xFF);
  // Copy rx_packets (little-endian)
  data[4] = (uint8_t)(rx_packets & 0xFF);
  data[5] = (uint8_t)((rx_packets >> 8) & 0xFF);
  data[6] = (uint8_t)((rx_packets >> 16) & 0xFF);
  data[7] = (uint8_t)((rx_packets >> 24) & 0xFF);
  // Copy tx_errors (little-endian)
  data[8] = (uint8_t)(tx_errors & 0xFF);
  data[9] = (uint8_t)((tx_errors >> 8) & 0xFF);
  data[10] = (uint8_t)((tx_errors >> 16) & 0xFF);
  data[11] = (uint8_t)((tx_errors >> 24) & 0xFF);
  // Copy rx_errors (little-endian)
  data[12] = (uint8_t)(rx_errors & 0xFF);
  data[13] = (uint8_t)((rx_errors >> 8) & 0xFF);
  data[14] = (uint8_t)((rx_errors >> 16) & 0xFF);
  data[15] = (uint8_t)((rx_errors >> 24) & 0xFF);
  return frame_with_data(COMMAND_STATS_RESPONSE, sequence, data, 16, out_frame);
}

// RadioConfig struct definition (referenced by parse_config)
typedef struct 
{
  uint32_t frequency;
  uint8_t spreading_factor;
  uint8_t bandwidth;
  uint8_t coding_rate;
  int8_t tx_power;
  uint8_t sync_word;
  uint16_t preamble_length;
  bool crc_enabled;
  bool implicit_header;
  bool ldro;
} RadioConfig;

// Parse configuration data (returns NULL if data invalid)
RadioConfig* parse_config(const uint8_t *data, size_t data_len, RadioConfig *out_config) 
{
  if (data_len < 14) return NULL;
  // Extract frequency (little-endian)
  out_config->frequency = ((uint32_t)data[0]) | 
                           (((uint32_t)data[1]) << 8) | 
                           (((uint32_t)data[2]) << 16) | 
                           (((uint32_t)data[3]) << 24);
  // Extract spreading factor
  out_config->spreading_factor = data[4];
  // Extract and convert bandwidth
  uint16_t bandwidth_raw = ((uint16_t)data[5]) | (((uint16_t)data[6]) << 8);
  uint8_t bandwidth = (uint8_t)(bandwidth_raw / 125);
  out_config->bandwidth = bandwidth < 2 ? bandwidth : 2;  // min(bandwidth, 2)
  // Extract coding rate
  out_config->coding_rate = data[7];
  // Extract tx power
  out_config->tx_power = (int8_t)data[8];
  // Extract sync word
  out_config->sync_word = data[9];
  // Extract preamble length (little-endian)
  out_config->preamble_length = ((uint16_t)data[10]) | (((uint16_t)data[11]) << 8);
  // Extract flags
  uint8_t flags = data[12];
  out_config->crc_enabled = (flags & 0x01) != 0;
  out_config->implicit_header = (flags & 0x02) != 0;
  out_config->ldro = (flags & 0x04) != 0;
  return out_config;
}
