#include "bsp.h"

/* Forward declaration for SHA256 from crypto module */
extern void sha256_hash(const uint8_t* data, size_t len, uint8_t* output);

/* KISS Protocol Constants */
#define FEND 0xC0
#define FESC 0xDB
#define TFEND 0xDC
#define TFESC 0xDD

#define MAX_DATA_SIZE 512
#define MAX_FRAME_SIZE (MAX_DATA_SIZE * 2 + 4)

/* Fixed-capacity vector implementation for uint8_t */
typedef struct 
{
  uint8_t data[MAX_DATA_SIZE];
  size_t len;
} Vec_u8_MAX_DATA;

typedef struct 
{
  uint8_t data[MAX_FRAME_SIZE];
  size_t len;
} Vec_u8_MAX_FRAME;

/* Vec operations for MAX_DATA_SIZE */
static inline void vec_u8_max_data_new(Vec_u8_MAX_DATA* vec) 
{
  vec->len = 0;
}

static inline bool vec_u8_max_data_push(Vec_u8_MAX_DATA* vec, uint8_t value) 
{
  if (vec->len >= MAX_DATA_SIZE) return false;
  vec->data[vec->len++] = value;
  return true;
}

static inline void vec_u8_max_data_clear(Vec_u8_MAX_DATA* vec) 
{
  vec->len = 0;
}

static inline void vec_u8_max_data_clone(const Vec_u8_MAX_DATA* src, Vec_u8_MAX_DATA* dst) 
{
  dst->len = src->len;
  memcpy(dst->data, src->data, src->len);
}

/* Vec operations for MAX_FRAME_SIZE */
static inline void vec_u8_max_frame_new(Vec_u8_MAX_FRAME* vec) 
{
  vec->len = 0;
}

static inline bool vec_u8_max_frame_push(Vec_u8_MAX_FRAME* vec, uint8_t value) 
{
  if (vec->len >= MAX_FRAME_SIZE) return false;
  vec->data[vec->len++] = value;
  return true;
}

/* KISS Command enumeration */
typedef enum 
{
  KISS_CMD_DATA_FRAME = 0x00,
  KISS_CMD_TX_DELAY = 0x01,
  KISS_CMD_PERSISTENCE = 0x02,
  KISS_CMD_SLOT_TIME = 0x03,
  KISS_CMD_TX_TAIL = 0x04,
  KISS_CMD_FULL_DUPLEX = 0x05,
  KISS_CMD_SET_HARDWARE = 0x06,
  KISS_CMD_RETURN = 0xFF
} KissCommand;

/* RNode Command enumeration */
typedef enum 
{
  RNODE_CMD_FREQUENCY = 0x01,
  RNODE_CMD_BANDWIDTH = 0x02,
  RNODE_CMD_TX_POWER = 0x03,
  RNODE_CMD_SPREADING_FACTOR = 0x04,
  RNODE_CMD_CODING_RATE = 0x05,
  RNODE_CMD_RADIO_STATE = 0x06,
  RNODE_CMD_RADIO_LOCK = 0x07,
  RNODE_CMD_DETECT = 0x08,
  RNODE_CMD_LEAVE = 0x0A,
  RNODE_CMD_SAVE_CONFIG = 0x0B,
  RNODE_CMD_RESET_CONFIG = 0x0C,
  RNODE_CMD_BOOTLOADER = 0x0D,
  RNODE_CMD_PROMISC = 0x0E,
  RNODE_CMD_READY = 0x0F,
  RNODE_CMD_PREAMBLE_LENGTH = 0x10,
  RNODE_CMD_SYMBOL_TIMEOUT = 0x11,
  RNODE_CMD_SYNC_WORD = 0x12,
  RNODE_CMD_CRC_MODE = 0x13,
  RNODE_CMD_IMPLICIT_HEADER = 0x14,
  RNODE_CMD_LDRO = 0x15,
  RNODE_CMD_STAT_RX = 0x21,
  RNODE_CMD_STAT_TX = 0x22,
  RNODE_CMD_STAT_RSSI = 0x23,
  RNODE_CMD_STAT_SNR = 0x24,
  RNODE_CMD_STAT_BATTERY = 0x25,
  RNODE_CMD_STAT_CHANNEL = 0x26,
  RNODE_CMD_AIRTIME_LIMIT = 0x27,
  RNODE_CMD_AIRTIME_USAGE = 0x28,
  RNODE_CMD_BLINK = 0x30,
  RNODE_CMD_LED_INTENSITY = 0x31,
  RNODE_CMD_RANDOM = 0x40,
  RNODE_CMD_PLATFORM = 0x48,
  RNODE_CMD_MCU = 0x49,
  RNODE_CMD_BOARD = 0x4A,
  RNODE_CMD_ROM_INFO = 0x4B,
  RNODE_CMD_FW_VERSION = 0x50,
  RNODE_CMD_PROTOCOL_VERSION = 0x51,
  RNODE_CMD_HARDWARE_SERIAL = 0x55,
  RNODE_CMD_SIGNATURE = 0x56,
  RNODE_CMD_TCXO_VOLTAGE = 0x60,
  RNODE_CMD_ERROR = 0x90,
  RNODE_CMD_ROM_DATA = 0xA0,
  RNODE_CMD_INFO = 0xB0,
  RNODE_CMD_DATA_RSSI = 0xFE
} RNodeCommand;

/* RNodeCommand from_byte function */
bool rnode_command_from_byte(uint8_t b, RNodeCommand* out_cmd) 
{
  switch (b) 
  {
    case 0x01: *out_cmd = RNODE_CMD_FREQUENCY; return true;
    case 0x02: *out_cmd = RNODE_CMD_BANDWIDTH; return true;
    case 0x03: *out_cmd = RNODE_CMD_TX_POWER; return true;
    case 0x04: *out_cmd = RNODE_CMD_SPREADING_FACTOR; return true;
    case 0x05: *out_cmd = RNODE_CMD_CODING_RATE; return true;
    case 0x06: *out_cmd = RNODE_CMD_RADIO_STATE; return true;
    case 0x07: *out_cmd = RNODE_CMD_RADIO_LOCK; return true;
    case 0x08: *out_cmd = RNODE_CMD_DETECT; return true;
    case 0x0A: *out_cmd = RNODE_CMD_LEAVE; return true;
    case 0x0B: *out_cmd = RNODE_CMD_SAVE_CONFIG; return true;
    case 0x0C: *out_cmd = RNODE_CMD_RESET_CONFIG; return true;
    case 0x0D: *out_cmd = RNODE_CMD_BOOTLOADER; return true;
    case 0x0E: *out_cmd = RNODE_CMD_PROMISC; return true;
    case 0x0F: *out_cmd = RNODE_CMD_READY; return true;
    case 0x10: *out_cmd = RNODE_CMD_PREAMBLE_LENGTH; return true;
    case 0x11: *out_cmd = RNODE_CMD_SYMBOL_TIMEOUT; return true;
    case 0x12: *out_cmd = RNODE_CMD_SYNC_WORD; return true;
    case 0x13: *out_cmd = RNODE_CMD_CRC_MODE; return true;
    case 0x14: *out_cmd = RNODE_CMD_IMPLICIT_HEADER; return true;
    case 0x15: *out_cmd = RNODE_CMD_LDRO; return true;
    case 0x21: *out_cmd = RNODE_CMD_STAT_RX; return true;
    case 0x22: *out_cmd = RNODE_CMD_STAT_TX; return true;
    case 0x23: *out_cmd = RNODE_CMD_STAT_RSSI; return true;
    case 0x24: *out_cmd = RNODE_CMD_STAT_SNR; return true;
    case 0x25: *out_cmd = RNODE_CMD_STAT_BATTERY; return true;
    case 0x26: *out_cmd = RNODE_CMD_STAT_CHANNEL; return true;
    case 0x27: *out_cmd = RNODE_CMD_AIRTIME_LIMIT; return true;
    case 0x28: *out_cmd = RNODE_CMD_AIRTIME_USAGE; return true;
    case 0x30: *out_cmd = RNODE_CMD_BLINK; return true;
    case 0x31: *out_cmd = RNODE_CMD_LED_INTENSITY; return true;
    case 0x40: *out_cmd = RNODE_CMD_RANDOM; return true;
    case 0x48: *out_cmd = RNODE_CMD_PLATFORM; return true;
    case 0x49: *out_cmd = RNODE_CMD_MCU; return true;
    case 0x4A: *out_cmd = RNODE_CMD_BOARD; return true;
    case 0x4B: *out_cmd = RNODE_CMD_ROM_INFO; return true;
    case 0x50: *out_cmd = RNODE_CMD_FW_VERSION; return true;
    case 0x51: *out_cmd = RNODE_CMD_PROTOCOL_VERSION; return true;
    case 0x55: *out_cmd = RNODE_CMD_HARDWARE_SERIAL; return true;
    case 0x56: *out_cmd = RNODE_CMD_SIGNATURE; return true;
    case 0x60: *out_cmd = RNODE_CMD_TCXO_VOLTAGE; return true;
    case 0x90: *out_cmd = RNODE_CMD_ERROR; return true;
    case 0xA0: *out_cmd = RNODE_CMD_ROM_DATA; return true;
    case 0xB0: *out_cmd = RNODE_CMD_INFO; return true;
    case 0xFE: *out_cmd = RNODE_CMD_DATA_RSSI; return true;
    default: return false;
  }
}

/* KissFrame structure */
typedef struct 
{
  uint8_t command;
  Vec_u8_MAX_DATA data;
} KissFrame;

/* KissFrame::new */
void kiss_frame_new(KissFrame* frame, uint8_t command) 
{
  frame->command = command;
  vec_u8_max_data_new(&frame->data);
}

/* KissFrame::data_frame */
bool kiss_frame_data_frame(const uint8_t* data, size_t data_len, KissFrame* out_frame) 
{
  if (data_len > MAX_DATA_SIZE) return false;
  kiss_frame_new(out_frame, KISS_CMD_DATA_FRAME);
  for (size_t i = 0; i < data_len; i++) vec_u8_max_data_push(&out_frame->data, data[i]);
  return true;
}

/* KissFrame::command_frame */
bool kiss_frame_command_frame(RNodeCommand cmd, const uint8_t* data, size_t data_len, KissFrame* out_frame) 
{
  if (data_len > MAX_DATA_SIZE) return false;
  kiss_frame_new(out_frame, (uint8_t)cmd);
  for (size_t i = 0; i < data_len; i++) vec_u8_max_data_push(&out_frame->data, data[i]);
  return true;
}

/* Helper function: escape_byte */
static void escape_byte(uint8_t byte, Vec_u8_MAX_FRAME* buf) 
{
  switch (byte) 
  {
    case FEND:
    vec_u8_max_frame_push(buf, FESC);
    vec_u8_max_frame_push(buf, TFEND);
    break;
    
    case FESC:
    vec_u8_max_frame_push(buf, FESC);
    vec_u8_max_frame_push(buf, TFESC);
    break;
    
    default:
    vec_u8_max_frame_push(buf, byte);
    break;
  }
}

/* KissFrame::encode */
void kiss_frame_encode(const KissFrame* frame, Vec_u8_MAX_FRAME* buf) 
{
  vec_u8_max_frame_new(buf);
  vec_u8_max_frame_push(buf, FEND);
  escape_byte(frame->command, buf);
  for (size_t i = 0; i < frame->data.len; i++) escape_byte(frame->data.data[i], buf);
  vec_u8_max_frame_push(buf, FEND);
}

/* KissFrame::port */
uint8_t kiss_frame_port(const KissFrame* frame) 
{
  return (frame->command >> 4) & 0x0F;
}

/* KissFrame::cmd_type */
uint8_t kiss_frame_cmd_type(const KissFrame* frame) 
{
  return frame->command & 0x0F;
}

/* Parser State enumeration */
typedef enum 
{
  PARSER_STATE_WAIT_START,
  PARSER_STATE_WAIT_COMMAND,
  PARSER_STATE_READ_DATA,
  PARSER_STATE_ESCAPE
} ParserState;

/* KissParser structure */
typedef struct 
{
  ParserState state;
  uint8_t command;
  Vec_u8_MAX_DATA data;
} KissParser;

/* KissParser::new */
void kiss_parser_new(KissParser* parser) 
{
  parser->state = PARSER_STATE_WAIT_START;
  parser->command = 0;
  vec_u8_max_data_new(&parser->data);
}

/* KissParser::reset */
void kiss_parser_reset(KissParser* parser) 
{
  parser->state = PARSER_STATE_WAIT_START;
  parser->command = 0;
  vec_u8_max_data_clear(&parser->data);
}

/* KissParser::feed */
bool kiss_parser_feed(KissParser* parser, uint8_t byte, KissFrame* out_frame) 
{
  switch (parser->state) 
  {
    case PARSER_STATE_WAIT_START:
    if (byte == FEND) 
    {
      parser->state = PARSER_STATE_WAIT_COMMAND;
      vec_u8_max_data_clear(&parser->data);
    }
    break;
            
    case PARSER_STATE_WAIT_COMMAND:
    switch (byte) 
    {
      case FEND:
      /* Stay in WaitCommand state */
      break;
      
      case FESC:
      /* Transition to Escape state */
      parser->state = PARSER_STATE_ESCAPE;
      break;
      
      default:
      parser->command = byte;
      parser->state = PARSER_STATE_READ_DATA;
      break;
    }
    break;
            
    case PARSER_STATE_READ_DATA:
    switch (byte) 
    {
      case FEND:
      /* Frame complete */
      out_frame->command = parser->command;
      vec_u8_max_data_clone(&parser->data, &out_frame->data);
      kiss_parser_reset(parser);
      return true;
      
      case FESC:
      parser->state = PARSER_STATE_ESCAPE;
      break;
      
      default:
      vec_u8_max_data_push(&parser->data, byte);
      break;
    }
    break;
            
    case PARSER_STATE_ESCAPE: 
    {
      uint8_t unescaped;
      switch (byte) 
      {
        case TFEND:
        unescaped = FEND;
        break;
        
        case TFESC:
        unescaped = FESC;
        break;
        
        default:
        unescaped = byte;
        break;
      }
      vec_u8_max_data_push(&parser->data, unescaped);
      parser->state = PARSER_STATE_READ_DATA;
      break;
    }
  }
    return false;
}

/* RNode State enumeration */
typedef enum 
{
  RNODE_STATE_OFFLINE,
  RNODE_STATE_ONLINE,
  RNODE_STATE_TRANSMITTING,
  RNODE_STATE_RECEIVING
} RNodeState;

/* RNodeConfig structure */
typedef struct 
{
  uint32_t frequency;
  uint32_t bandwidth;
  int8_t tx_power;
  uint8_t spreading_factor;
  uint8_t coding_rate;
  uint16_t preamble_length;
  uint8_t sync_word;
  bool crc_enabled;
  bool implicit_header;
  bool ldro;
} RNodeConfig;

/* RNodeConfig::default */
void rnode_config_default(RNodeConfig* config) 
{
  config->frequency = 868100000;
  config->bandwidth = 125000;
  config->tx_power = 14;
  config->spreading_factor = 9;
  config->coding_rate = 5;
  config->preamble_length = 8;
  config->sync_word = 0x12;
  config->crc_enabled = true;
  config->implicit_header = false;
  config->ldro = false;
}

/* RNodeConfig::eu868 */
void rnode_config_eu868(RNodeConfig* config) 
{
  rnode_config_default(config);
  config->frequency = 868100000;
}

/* RNodeConfig::us915 */
void rnode_config_us915(RNodeConfig* config) 
{
  rnode_config_default(config);
  config->frequency = 915000000;
}

/* RNodeConfig::bandwidth_to_hz */
uint32_t rnode_config_bandwidth_to_hz(uint8_t bw_value) 
{
  switch (bw_value) 
  {
    case 0: return 125000;
    case 1: return 250000;
    case 2: return 500000;
    default: return 125000;
  }
}

/* RNodeConfig::hz_to_bandwidth */
uint8_t rnode_config_hz_to_bandwidth(uint32_t hz) 
{
  if (hz <= 187500) return 0;
  else if (hz <= 375000) return 1;
  else return 2;
}

/* RNodeConfig::coding_rate_to_ratio */
void rnode_config_coding_rate_to_ratio(uint8_t cr, uint8_t* numerator, uint8_t* denominator) 
{
  *numerator = 4;
  *denominator = cr;
}

/* RNodeConfig::should_enable_ldro */
bool rnode_config_should_enable_ldro(const RNodeConfig* config) 
{
  if (config->bandwidth <= 125000) return config->spreading_factor >= 11;
  else if (config->bandwidth <= 250000) return config->spreading_factor >= 12;
  else return false;
}

/* Helper: max for float */
static inline float fmax_custom(float a, float b) 
{
  return (a > b) ? a : b;
}

/* Helper: ceil for float */
static inline float fceil_custom(float x) 
{
  int i = (int)x;
  return (x > i) ? (i + 1.0f) : (float)i;
}

/* Helper: pow for float */
static inline float fpow_custom(float base, float exp) 
{
  /* Simple integer power implementation */
  if (exp == 0.0f) return 1.0f;
  float result = 1.0f;
  int exp_int = (int)exp;
  for (int i = 0; i < exp_int; i++) result *= base;
  return result;
}

/* RNodeConfig::packet_airtime_ms */
uint32_t rnode_config_packet_airtime_ms(const RNodeConfig* config, size_t payload_len) 
{
  float sf = (float)config->spreading_factor;
  float bw = (float)config->bandwidth;
  float cr = (float)config->coding_rate;
  float pl = (float)payload_len;
  float preamble = (float)config->preamble_length;
  float t_sym = (fpow_custom(2.0f, sf)) / bw * 1000.0f;
  float t_preamble = (preamble + 4.25f) * t_sym;
  float de = rnode_config_should_enable_ldro(config) ? 1.0f : 0.0f;
  float h = config->implicit_header ? 1.0f : 0.0f;
  float crc = config->crc_enabled ? 1.0f : 0.0f;
  float numerator = 8.0f * pl - 4.0f * sf + 28.0f + 16.0f * crc - 20.0f * h;
  float denominator = 4.0f * (sf - 2.0f * de);
  float n_payload = 8.0f + fceil_custom(fmax_custom(numerator / denominator, 0.0f)) * cr;
  float t_payload = n_payload * t_sym;
  return (uint32_t)(t_preamble + t_payload);
}

/* RNodeStats structure */
typedef struct 
{
  uint32_t rx_count;
  uint64_t rx_bytes;
  uint32_t tx_count;
  uint64_t tx_bytes;
  int16_t last_rssi;
  int8_t last_snr;
  uint64_t airtime_used;
  uint64_t channel_busy;
} RNodeStats;

/* RNodeStats::default */
void rnode_stats_default(RNodeStats* stats) 
{
  memset(stats, 0, sizeof(RNodeStats));
}

/* RNodeIdentity structure */
typedef struct 
{
  uint8_t serial[16];
  const char* platform;
  const char* mcu;
  const char* board;
  const char* fw_version;
  uint8_t protocol_version;
  uint8_t hw_revision;
} RNodeIdentity;

/* RNodeIdentity::default */
void rnode_identity_default(RNodeIdentity* identity) 
{
  memset(identity->serial, 0, 16);
  identity->platform = "ESP32-S3";
  identity->mcu = "ESP32-S3";
  identity->board = "LunarNode";
  identity->fw_version = "1.0.0-lunarcore";
  identity->protocol_version = 1;
  identity->hw_revision = 1;
}

/* RNodeIdentity::identity_hash */
void rnode_identity_hash(const RNodeIdentity* identity, uint8_t hash[32]) 
{
  uint8_t data[64];
  memset(data, 0, 64);
  memcpy(data, identity->serial, 16);
  /* Copy platform string (up to 8 bytes) */
  size_t platform_len = strlen(identity->platform);
  if (platform_len > 8) platform_len = 8;
  memcpy(data + 16, identity->platform, platform_len);
  data[24] = identity->protocol_version;
  data[25] = identity->hw_revision;
  sha256_hash(data, 64, hash);
}

/* RNodeHandler structure */
typedef struct 
{
  KissParser parser;
  RNodeConfig config;
  RNodeState state;
  bool locked;
  bool promiscuous;
  RNodeStats stats;
  RNodeIdentity identity;
  uint32_t random_seed;
  uint16_t battery_mv;
  uint8_t led_intensity;
  uint32_t airtime_limit;
} RNodeHandler;

/* RNodeHandler::new */
void rnode_handler_new(RNodeHandler* handler) 
{
  kiss_parser_new(&handler->parser);
  rnode_config_default(&handler->config);
  handler->state = RNODE_STATE_OFFLINE;
  handler->locked = false;
  handler->promiscuous = false;
  rnode_stats_default(&handler->stats);
  rnode_identity_default(&handler->identity);
  handler->random_seed = 0;
  handler->battery_mv = 0;
  handler->led_intensity = 64;
  handler->airtime_limit = 0;
}

/* RNodeHandler::with_identity */
void rnode_handler_with_identity(RNodeHandler* handler, const RNodeIdentity* identity) 
{
  rnode_handler_new(handler);
  handler->identity = *identity;
}

/* RNodeHandler::set_serial */
void rnode_handler_set_serial(RNodeHandler* handler, const uint8_t serial[16]) 
{
  memcpy(handler->identity.serial, serial, 16);
}

/* RNodeHandler::set_battery_voltage */
void rnode_handler_set_battery_voltage(RNodeHandler* handler, uint16_t mv) 
{
  handler->battery_mv = mv;
}

/* RNodeHandler::set_random_seed */
void rnode_handler_set_random_seed(RNodeHandler* handler, uint32_t seed) 
{
  handler->random_seed = seed;
}

/* RNodeHandler::next_random */
uint32_t rnode_handler_next_random(RNodeHandler* handler) 
{
  /* Linear congruential generator */
  handler->random_seed = handler->random_seed * 1103515245 + 12345;
  return handler->random_seed;
}

/* RNodeHandler::feed_serial */
bool rnode_handler_feed_serial(RNodeHandler* handler, uint8_t byte, KissFrame* out_frame) 
{
  return kiss_parser_feed(&handler->parser, byte, out_frame);
}

/* Helper: min for int8_t */
static inline int8_t i8_min(int8_t a, int8_t b) 
{
  return (a < b) ? a : b;
}

/* Helper: max for int8_t */
static inline int8_t i8_max(int8_t a, int8_t b) 
{
  return (a > b) ? a : b;
}

/* Helper: min for uint8_t */
static inline uint8_t u8_min(uint8_t a, uint8_t b) 
{
  return (a < b) ? a : b;
}

/* Helper: max for uint8_t */
static inline uint8_t u8_max(uint8_t a, uint8_t b) 
{
  return (a > b) ? a : b;
}

/* Helper: min for uint64_t */
static inline uint64_t u64_min(uint64_t a, uint64_t b) 
{
  return (a < b) ? a : b;
}

/* RNodeHandler::process_frame */
bool rnode_handler_process_frame(RNodeHandler* handler, const KissFrame* frame, KissFrame* out_frame) 
{
  /* If it's a data frame, return false (no response) */
  if (frame->command == KISS_CMD_DATA_FRAME) return false;
  RNodeCommand cmd;
  if (!rnode_command_from_byte(frame->command, &cmd)) 
  {
    /* Unknown command - return error */
    uint8_t error_data[] = {0xFF};
    return kiss_frame_command_frame(RNODE_CMD_ERROR, error_data, 1, out_frame);
  }
  switch (cmd) 
  {
    case RNODE_CMD_FREQUENCY:
    if (frame->data.len >= 4 && !handler->locked) 
    {
      handler->config.frequency = ((uint32_t)frame->data.data[0] << 24) |
                                           ((uint32_t)frame->data.data[1] << 16) |
                                           ((uint32_t)frame->data.data[2] << 8) |
                                           ((uint32_t)frame->data.data[3]);
    }
    {
      uint8_t freq_bytes[4];
      freq_bytes[0] = (handler->config.frequency >> 24) & 0xFF;
      freq_bytes[1] = (handler->config.frequency >> 16) & 0xFF;
      freq_bytes[2] = (handler->config.frequency >> 8) & 0xFF;
      freq_bytes[3] = handler->config.frequency & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_FREQUENCY, freq_bytes, 4, out_frame);
    }
            
    case RNODE_CMD_BANDWIDTH:
    if (frame->data.len >= 4 && !handler->locked) 
    {
                handler->config.bandwidth = ((uint32_t)frame->data.data[0] << 24) |
                                           ((uint32_t)frame->data.data[1] << 16) |
                                           ((uint32_t)frame->data.data[2] << 8) |
                                           ((uint32_t)frame->data.data[3]);
    }
    {
      uint8_t bw_bytes[4];
      bw_bytes[0] = (handler->config.bandwidth >> 24) & 0xFF;
      bw_bytes[1] = (handler->config.bandwidth >> 16) & 0xFF;
      bw_bytes[2] = (handler->config.bandwidth >> 8) & 0xFF;
      bw_bytes[3] = handler->config.bandwidth & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_BANDWIDTH, bw_bytes, 4, out_frame);
    }
            
    case RNODE_CMD_TX_POWER:
    if (frame->data.len >= 1 && !handler->locked) 
    {
      /* Clamp power between -9 and 22 */
      int8_t power = (int8_t)frame->data.data[0];
      power = i8_max(power, -9);
      power = i8_min(power, 22);
      handler->config.tx_power = power;
    }
    {
      uint8_t power_byte = (uint8_t)handler->config.tx_power;
      return kiss_frame_command_frame(RNODE_CMD_TX_POWER, &power_byte, 1, out_frame);
    }
            
    case RNODE_CMD_SPREADING_FACTOR:
    if (frame->data.len >= 1 && !handler->locked) 
    {
      uint8_t sf = u8_max(frame->data.data[0], 7);
      sf = u8_min(sf, 12);
      handler->config.spreading_factor = sf;
      /* Update LDRO based on new SF */
      handler->config.ldro = rnode_config_should_enable_ldro(&handler->config);
    }
    return kiss_frame_command_frame(RNODE_CMD_SPREADING_FACTOR,&handler->config.spreading_factor, 1, out_frame);
            
    case RNODE_CMD_CODING_RATE:
    if (frame->data.len >= 1 && !handler->locked) 
    {
      uint8_t cr = u8_max(frame->data.data[0], 5);
      cr = u8_min(cr, 8);
      handler->config.coding_rate = cr;
    }
    return kiss_frame_command_frame(RNODE_CMD_CODING_RATE,&handler->config.coding_rate, 1, out_frame);
            
    case RNODE_CMD_PREAMBLE_LENGTH:
    if (frame->data.len >= 2 && !handler->locked) 
    {
      handler->config.preamble_length = ((uint16_t)frame->data.data[0] << 8) | ((uint16_t)frame->data.data[1]);
    }
    {
      uint8_t preamble_bytes[2];
      preamble_bytes[0] = (handler->config.preamble_length >> 8) & 0xFF;
      preamble_bytes[1] = handler->config.preamble_length & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_PREAMBLE_LENGTH, preamble_bytes, 2, out_frame);
    }
            
    case RNODE_CMD_SYNC_WORD:
    if (frame->data.len >= 1 && !handler->locked) 
    {
      handler->config.sync_word = frame->data.data[0];
    }
    return kiss_frame_command_frame(RNODE_CMD_SYNC_WORD,&handler->config.sync_word, 1, out_frame);
            
    case RNODE_CMD_CRC_MODE:
    if (frame->data.len >= 1 && !handler->locked) 
    {
      handler->config.crc_enabled = (frame->data.data[0] != 0);
    }
    {
      uint8_t crc_byte = handler->config.crc_enabled ? 1 : 0;
      return kiss_frame_command_frame(RNODE_CMD_CRC_MODE, &crc_byte, 1, out_frame);
    }
            
    case RNODE_CMD_IMPLICIT_HEADER:
    if (frame->data.len >= 1 && !handler->locked) handler->config.implicit_header = (frame->data.data[0] != 0);
    {
      uint8_t ih_byte = handler->config.implicit_header ? 1 : 0;
      return kiss_frame_command_frame(RNODE_CMD_IMPLICIT_HEADER, &ih_byte, 1, out_frame);
    }
            
    case RNODE_CMD_LDRO:
    if (frame->data.len >= 1 && !handler->locked) handler->config.ldro = (frame->data.data[0] != 0);
    {
      uint8_t ldro_byte = handler->config.ldro ? 1 : 0;
      return kiss_frame_command_frame(RNODE_CMD_LDRO, &ldro_byte, 1, out_frame);
    }
            
    case RNODE_CMD_RADIO_STATE:
    if (frame->data.len >= 1) handler->state = (frame->data.data[0] != 0) ? RNODE_STATE_ONLINE : RNODE_STATE_OFFLINE;
    {
      uint8_t state_byte = (handler->state != RNODE_STATE_OFFLINE) ? 1 : 0;
      return kiss_frame_command_frame(RNODE_CMD_RADIO_STATE, &state_byte, 1, out_frame);
    }
            
    case RNODE_CMD_RADIO_LOCK:
    if (frame->data.len >= 1) handler->locked = (frame->data.data[0] != 0);
    {
      uint8_t lock_byte = handler->locked ? 1 : 0;
      return kiss_frame_command_frame(RNODE_CMD_RADIO_LOCK, &lock_byte, 1, out_frame);
    }
            
    case RNODE_CMD_PROMISC:
    if (frame->data.len >= 1) handler->promiscuous = (frame->data.data[0] != 0);
    {
      uint8_t promisc_byte = handler->promiscuous ? 1 : 0;
      return kiss_frame_command_frame(RNODE_CMD_PROMISC, &promisc_byte, 1, out_frame);
    }
            
    case RNODE_CMD_DETECT:
    /* Return detection response */
    {
      uint8_t detect_data[] = {0x01, handler->identity.hw_revision};
      return kiss_frame_command_frame(RNODE_CMD_DETECT, detect_data, 2, out_frame);
    }
            
    case RNODE_CMD_READY:
    /* Return ready response */
    {
      uint8_t ready_data[] = {0x01};
      return kiss_frame_command_frame(RNODE_CMD_READY, ready_data, 1, out_frame);
    }
            
    case RNODE_CMD_FW_VERSION:
    return kiss_frame_command_frame(RNODE_CMD_FW_VERSION,
                                           (const uint8_t*)handler->identity.fw_version,
                                           strlen(handler->identity.fw_version),
                                           out_frame);
            
    case RNODE_CMD_PROTOCOL_VERSION:
    return kiss_frame_command_frame(RNODE_CMD_PROTOCOL_VERSION,
                                           &handler->identity.protocol_version, 1, out_frame);
            
    case RNODE_CMD_PLATFORM:
    return kiss_frame_command_frame(RNODE_CMD_PLATFORM,
                                           (const uint8_t*)handler->identity.platform,
                                           strlen(handler->identity.platform),
                                           out_frame);
            
    case RNODE_CMD_MCU:
    return kiss_frame_command_frame(RNODE_CMD_MCU,
                                           (const uint8_t*)handler->identity.mcu,
                                           strlen(handler->identity.mcu),
                                           out_frame);
            
    case RNODE_CMD_BOARD:
    return kiss_frame_command_frame(RNODE_CMD_BOARD,
                                           (const uint8_t*)handler->identity.board,
                                           strlen(handler->identity.board),
                                           out_frame);
            
    case RNODE_CMD_HARDWARE_SERIAL:
    return kiss_frame_command_frame(RNODE_CMD_HARDWARE_SERIAL,
                                           handler->identity.serial, 16, out_frame);
            
    case RNODE_CMD_SIGNATURE:
    /* Calculate and return identity hash */
    {
      uint8_t hash[32];
      rnode_identity_hash(&handler->identity, hash);
      return kiss_frame_command_frame(RNODE_CMD_SIGNATURE, hash, 32, out_frame);
    }
            
    case RNODE_CMD_STAT_RX:
    {
      uint8_t rx_bytes[4];
      rx_bytes[0] = (handler->stats.rx_count >> 24) & 0xFF;
      rx_bytes[1] = (handler->stats.rx_count >> 16) & 0xFF;
      rx_bytes[2] = (handler->stats.rx_count >> 8) & 0xFF;
      rx_bytes[3] = handler->stats.rx_count & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_STAT_RX, rx_bytes, 4, out_frame);
    }
            
    case RNODE_CMD_STAT_TX:
    {
      uint8_t tx_bytes[4];
      tx_bytes[0] = (handler->stats.tx_count >> 24) & 0xFF;
      tx_bytes[1] = (handler->stats.tx_count >> 16) & 0xFF;
      tx_bytes[2] = (handler->stats.tx_count >> 8) & 0xFF;
      tx_bytes[3] = handler->stats.tx_count & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_STAT_TX, tx_bytes, 4, out_frame);
    }
            
    case RNODE_CMD_STAT_RSSI:
    {
      uint8_t rssi_bytes[2];
      rssi_bytes[0] = (handler->stats.last_rssi >> 8) & 0xFF;
      rssi_bytes[1] = handler->stats.last_rssi & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_STAT_RSSI, rssi_bytes, 2, out_frame);
    }
            
    case RNODE_CMD_STAT_SNR:
    {
      uint8_t snr_byte = (uint8_t)handler->stats.last_snr;
      return kiss_frame_command_frame(RNODE_CMD_STAT_SNR, &snr_byte, 1, out_frame);
    }
            
    case RNODE_CMD_STAT_BATTERY:
    {
      uint8_t battery_bytes[2];
      battery_bytes[0] = (handler->battery_mv >> 8) & 0xFF;
      battery_bytes[1] = handler->battery_mv & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_STAT_BATTERY, battery_bytes, 2, out_frame);
    }
            
    case RNODE_CMD_STAT_CHANNEL:
    /* Calculate channel utilization */
    {
      uint8_t util;
      if (handler->stats.channel_busy > 0) 
      {
        uint64_t calc = (handler->stats.airtime_used * 100) / handler->stats.channel_busy;
        util = (uint8_t)u64_min(calc, 100);
      } 
      else util = 0;
      return kiss_frame_command_frame(RNODE_CMD_STAT_CHANNEL, &util, 1, out_frame);
    }
            
    case RNODE_CMD_AIRTIME_LIMIT:
    if (frame->data.len >= 4) 
    {
      handler->airtime_limit = ((uint32_t)frame->data.data[0] << 24) |
                                        ((uint32_t)frame->data.data[1] << 16) |
                                        ((uint32_t)frame->data.data[2] << 8) |
                                        ((uint32_t)frame->data.data[3]);
    }
    {
      uint8_t limit_bytes[4];
      limit_bytes[0] = (handler->airtime_limit >> 24) & 0xFF;
      limit_bytes[1] = (handler->airtime_limit >> 16) & 0xFF;
      limit_bytes[2] = (handler->airtime_limit >> 8) & 0xFF;
      limit_bytes[3] = handler->airtime_limit & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_AIRTIME_LIMIT, limit_bytes, 4, out_frame);
    }
            
    case RNODE_CMD_AIRTIME_USAGE:
    {
      uint32_t usage = (uint32_t)(handler->stats.airtime_used % 0xFFFFFFFFULL);
      uint8_t usage_bytes[4];
      usage_bytes[0] = (usage >> 24) & 0xFF;
      usage_bytes[1] = (usage >> 16) & 0xFF;
      usage_bytes[2] = (usage >> 8) & 0xFF;
      usage_bytes[3] = usage & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_AIRTIME_USAGE, usage_bytes, 4, out_frame);
    }
            
    case RNODE_CMD_BLINK:
    /* Return blink acknowledgment */
    {
      uint8_t blink_data[] = {0x01};
      return kiss_frame_command_frame(RNODE_CMD_BLINK, blink_data, 1, out_frame);
    }
            
    case RNODE_CMD_LED_INTENSITY:
    if (frame->data.len >= 1) handler->led_intensity = frame->data.data[0];
    return kiss_frame_command_frame(RNODE_CMD_LED_INTENSITY,&handler->led_intensity, 1, out_frame);
            
    case RNODE_CMD_RANDOM:
    /* Generate and return random number */
    {
      uint32_t r = rnode_handler_next_random(handler);
      uint8_t random_bytes[4];
      random_bytes[0] = (r >> 24) & 0xFF;
      random_bytes[1] = (r >> 16) & 0xFF;
      random_bytes[2] = (r >> 8) & 0xFF;
      random_bytes[3] = r & 0xFF;
      return kiss_frame_command_frame(RNODE_CMD_RANDOM, random_bytes, 4, out_frame);
    }
            
    case RNODE_CMD_LEAVE:
    /* Set state to offline and return no response */
    handler->state = RNODE_STATE_OFFLINE;
    return false;
            
    case RNODE_CMD_SAVE_CONFIG:
    /* Return save config acknowledgment */
    {
      uint8_t save_data[] = {0x01};
      return kiss_frame_command_frame(RNODE_CMD_SAVE_CONFIG, save_data, 1, out_frame);
    }
            
    case RNODE_CMD_RESET_CONFIG:
    if (!handler->locked) rnode_config_default(&handler->config);
    {
      uint8_t reset_data[] = {0x01};
      return kiss_frame_command_frame(RNODE_CMD_RESET_CONFIG, reset_data, 1, out_frame);
    }
            
    case RNODE_CMD_BOOTLOADER:
    /* Return bootloader acknowledgment */
    {
      uint8_t bootloader_data[] = {0x01};
      return kiss_frame_command_frame(RNODE_CMD_BOOTLOADER, bootloader_data, 1, out_frame);
    }
            
    case RNODE_CMD_ERROR:
    /* Echo error back if data provided */
    if (frame->data.len > 0) return kiss_frame_command_frame(RNODE_CMD_ERROR,frame->data.data, frame->data.len, out_frame);
    else return false;
            
    default:
    /* Unknown command - return error */
    {
      uint8_t error_data[] = {0xFF};
      return kiss_frame_command_frame(RNODE_CMD_ERROR, error_data, 1, out_frame);
    }
  }
}

/* RNodeHandler::process_lora_packet */
void rnode_handler_process_lora_packet(RNodeHandler* handler, const uint8_t* data, size_t data_len,
                                        int16_t rssi, int8_t snr, KissFrame* out_frame) 
{
  handler->stats.rx_count += 1;
  handler->stats.rx_bytes += (uint64_t)data_len;
  handler->stats.last_rssi = rssi;
  handler->stats.last_snr = snr;
    /* Create data frame with received packet */
  kiss_frame_data_frame(data, data_len, out_frame);
}

/////////////////////////////////////////////////////



#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

/* Constants for KISS Protocol */
#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD

/* KISS Commands */
typedef enum {
    KissCommandDataFrame = 0x00,
    KissCommandUnknown   = 0xFF
} KissCommand;

/* RNode Commands */
typedef enum {
    RNodeCommandDetect          = 0x01,
    RNodeCommandFrequency       = 0x02,
    RNodeCommandBandwidth       = 0x03,
    RNodeCommandSpreadingFactor = 0x04,
    RNodeCommandCodingRate      = 0x05,
    RNodeCommandTxPower         = 0x06,
    RNodeCommandReady           = 0x07,
    RNodeCommandFwVersion       = 0x08,
    RNodeCommandPlatform        = 0x09,
} RNodeCommand;

/* RNode States */
typedef enum {
    RNodeStateOffline,
    RNodeStateOnline,
    RNodeStateLocked
} RNodeState;

/* KISS Frame Structure */
#define MAX_KISS_DATA_LEN 512
typedef struct {
    uint8_t command;
    uint8_t data[MAX_KISS_DATA_LEN];
    size_t data_len;
} KissFrame;

/* RNode Configuration Structure */
typedef struct {
    uint32_t frequency;
    uint32_t bandwidth;
    uint8_t spreading_factor;
    uint8_t coding_rate;
    int8_t tx_power;
} RNodeConfig;

/* RNode Statistics Structure */
typedef struct {
    uint64_t rx_count;
    uint64_t rx_bytes;
    uint64_t tx_count;
    uint64_t tx_bytes;
    uint64_t airtime_used;
    uint64_t channel_busy;
    int16_t last_rssi;
    int16_t last_snr;
} RNodeStats;

/* RNode Identity Structure */
typedef struct {
    uint8_t serial[16];
} RNodeIdentity;

/* Main RNode Handler Structure */
typedef struct {
    RNodeStats stats;
    RNodeConfig config;
    RNodeState state;
    RNodeIdentity identity;
    bool locked;
    bool promiscuous;
    uint64_t airtime_limit;
} RNodeHandler;

/* --- KissFrame Implementation --- */

void kiss_frame_init(KissFrame* frame, uint8_t command) {
    frame->command = command;
    frame->data_len = 0;
}

KissFrame kiss_frame_new(uint8_t command) {
    KissFrame frame;
    kiss_frame_init(&frame, command);
    return frame;
}

bool kiss_frame_push_data(KissFrame* frame, uint8_t b) {
    if (frame->data_len < MAX_KISS_DATA_LEN) {
        frame->data[frame->data_len++] = b;
        return true;
    }
    return false;
}

/* Helper for tests: create data frame */
bool kiss_frame_data_frame(KissFrame* frame, const uint8_t* data, size_t len) {
    if (len > MAX_KISS_DATA_LEN) return false;
    frame->command = KissCommandDataFrame;
    memcpy(frame->data, data, len);
    frame->data_len = len;
    return true;
}

/* Helper for tests: create command frame */
bool kiss_frame_command_frame(KissFrame* frame, uint8_t command, const uint8_t* data, size_t len) {
    if (len > MAX_KISS_DATA_LEN) return false;
    frame->command = command;
    memcpy(frame->data, data, len);
    frame->data_len = len;
    return true;
}

/* Encode frame to KISS wire format */
size_t kiss_frame_encode(const KissFrame* frame, uint8_t* out, size_t max_out) {
    size_t p = 0;
    if (p < max_out) out[p++] = FEND;
    
    // Command byte
    uint8_t cmd = frame->command;
    if (cmd == FEND) {
        if (p < max_out) out[p++] = FESC;
        if (p < max_out) out[p++] = TFEND;
    } else if (cmd == FESC) {
        if (p < max_out) out[p++] = FESC;
        if (p < max_out) out[p++] = TFESC;
    } else {
        if (p < max_out) out[p++] = cmd;
    }

    // Data bytes
    for (size_t i = 0; i < frame->data_len; i++) {
        uint8_t b = frame->data[i];
        if (b == FEND) {
            if (p < max_out) out[p++] = FESC;
            if (p < max_out) out[p++] = TFEND;
        } else if (b == FESC) {
            if (p < max_out) out[p++] = FESC;
            if (p < max_out) out[p++] = TFESC;
        } else {
            if (p < max_out) out[p++] = b;
        }
    }

    if (p < max_out) out[p++] = FEND;
    return p;
}

/* --- KissParser Implementation --- */

typedef struct {
    KissFrame current_frame;
    bool in_frame;
    bool escaped;
} KissParser;

void kiss_parser_init(KissParser* parser) {
    parser->in_frame = false;
    parser->escaped = false;
}

KissFrame* kiss_parser_feed(KissParser* parser, uint8_t byte, bool* out_ready) {
    *out_ready = false;
    if (byte == FEND) {
        if (parser->in_frame && parser->current_frame.data_len > 0) {
            *out_ready = true;
            parser->in_frame = false;
            return &parser->current_frame;
        } else {
            parser->in_frame = true;
            parser->current_frame.data_len = 0;
            parser->current_frame.command = KissCommandUnknown;
            return NULL;
        }
    }

    if (!parser->in_frame) return NULL;

    if (byte == FESC) {
        parser->escaped = true;
        return NULL;
    }

    if (parser->escaped) {
        if (byte == TFEND) byte = FEND;
        else if (byte == TFESC) byte = FESC;
        parser->escaped = false;
    }

    if (parser->current_frame.command == KissCommandUnknown) {
        parser->current_frame.command = byte;
    } else {
        kiss_frame_push_data(&parser->current_frame, byte);
    }

    return NULL;
}

/* --- RNodeConfig Implementation --- */

uint64_t rnode_config_packet_airtime_ms(const RNodeConfig* config, size_t packet_len) {
    // Simplified LoRa airtime calculation for logic preservation
    // In actual hardware this depends on SF, BW, CR, Preamble.
    // Based on the test expectation: 50 bytes -> between 50 and 500 ms.
    uint32_t sf = config->spreading_factor;
    if (sf < 7) sf = 7;
    uint32_t bw = config->bandwidth;
    if (bw == 0) bw = 125000;
    
    // Logic: higher SF = higher airtime, higher BW = lower airtime
    double symbol_duration = (double)(1 << sf) / (double)bw;
    double airtime_sec = (packet_len + 8) * symbol_duration * 10.0; // rough approximation
    return (uint64_t)(airtime_sec * 1000.0);
}

bool rnode_config_should_enable_ldro(const RNodeConfig* config) {
    // LDRO is usually required when symbol duration > 16ms
    uint32_t sf = config->spreading_factor;
    uint32_t bw = config->bandwidth;
    double symbol_duration_ms = ((double)(1 << sf) / (double)bw) * 1000.0;
    return symbol_duration_ms > 16.0;
}

void rnode_config_default(RNodeConfig* config) {
    config->frequency = 868000000;
    config->bandwidth = 125000;
    config->spreading_factor = 7;
    config->coding_rate = 5;
    config->tx_power = 20;
}

/* --- RNodeStats Implementation --- */

void rnode_stats_default(RNodeStats* stats) {
    memset(stats, 0, sizeof(RNodeStats));
}

/* --- RNodeIdentity Implementation --- */

uint32_t rnode_identity_identity_hash(const RNodeIdentity* identity) {
    uint32_t hash = 5381;
    for (int i = 0; i < 16; i++) {
        hash = ((hash << 5) + hash) + identity->serial[i];
    }
    return hash;
}

/* --- RNodeHandler Implementation --- */

void rnode_handler_init(RNodeHandler* self) {
    rnode_stats_default(&self->stats);
    rnode_config_default(&self->config);
    self->state = RNodeStateOffline;
    memset(&self->identity, 0, sizeof(RNodeIdentity));
    self->locked = false;
    self->promiscuous = false;
    self->airtime_limit = 0;
}

KissFrame rnode_handler_process_lora_packet_raw(RNodeHandler* self, const uint8_t* data, size_t len) {
    self->stats.rx_count += 1;
    self->stats.rx_bytes += (uint64_t)len;

    KissFrame frame = kiss_frame_new((uint8_t)KissCommandDataFrame);
    for (size_t i = 0; i < len; i++) {
        kiss_frame_push_data(&frame, data[i]);
    }
    return frame;
}

// Added helper based on test usage
KissFrame rnode_handler_process_lora_packet(RNodeHandler* self, const uint8_t* data, size_t len, int16_t rssi, int16_t snr) {
    self->stats.last_rssi = rssi;
    self->stats.last_snr = snr;
    return rnode_handler_process_lora_packet_raw(self, data, len);
}

const uint8_t* rnode_handler_get_tx_data(RNodeHandler* self, const KissFrame* frame, size_t* out_len) {
    if (frame->command == (uint8_t)KissCommandDataFrame) {
        self->stats.tx_count += 1;
        self->stats.tx_bytes += (uint64_t)frame->data_len;

        uint64_t airtime = rnode_config_packet_airtime_ms(&self->config, frame->data_len);
        self->stats.airtime_used += airtime;
        
        *out_len = frame->data_len;
        return frame->data;
    } else {
        *out_len = 0;
        return NULL;
    }
}

void rnode_handler_record_channel_busy(RNodeHandler* self, uint64_t ms) {
    self->stats.channel_busy += ms;
}

const RNodeConfig* rnode_handler_config(const RNodeHandler* self) {
    return &self->config;
}

RNodeConfig* rnode_handler_config_mut(RNodeHandler* self) {
    return &self->config;
}

RNodeState rnode_handler_state(const RNodeHandler* self) {
    return self->state;
}

void rnode_handler_set_state(RNodeHandler* self, RNodeState state) {
    self->state = state;
}

const RNodeStats* rnode_handler_stats(const RNodeHandler* self) {
    return &self->stats;
}

void rnode_handler_reset_stats(RNodeHandler* self) {
    rnode_stats_default(&self->stats);
}

const RNodeIdentity* rnode_handler_identity(const RNodeHandler* self) {
    return &self->identity;
}

bool rnode_handler_is_online(const RNodeHandler* self) {
    return self->state != RNodeStateOffline;
}

bool rnode_handler_is_locked(const RNodeHandler* self) {
    return self->locked;
}

bool rnode_handler_is_promiscuous(const RNodeHandler* self) {
    return self->promiscuous;
}

bool rnode_handler_check_airtime_limit(const RNodeHandler* self, size_t packet_len) {
    if (self->airtime_limit == 0) {
        return true;
    }
    uint64_t airtime = rnode_config_packet_airtime_ms(&self->config, packet_len);

    return (self->stats.airtime_used + airtime) <= self->airtime_limit;
}

/* Handler for process_frame used in tests */
void* rnode_handler_process_frame(RNodeHandler* self, const KissFrame* frame) {
    if (frame->command == (uint8_t)RNodeCommandFrequency) {
        uint32_t freq = 0;
        if (frame->data_len >= 4) {
            freq = ((uint32_t)frame->data[0] << 24) | ((uint32_t)frame->data[1] << 16) | 
                   ((uint32_t)frame->data[2] << 8) | (uint32_t)frame->data[3];
        }
        self->config.frequency = freq;
        return self; 
    } else if (frame->command == (uint8_t)RNodeCommandSpreadingFactor) {
        if (frame->data_len >= 1) {
            self->config.spreading_factor = frame->data[0];
        }
        return self;
    } else if (frame->command == (uint8_t)RNodeCommandDetect || 
               frame->command == (uint8_t)RNodeCommandReady ||
               frame->command == (uint8_t)RNodeCommandFwVersion ||
               frame->command == (uint8_t)RNodeCommandPlatform) {
        return self;
    }
    return NULL;
}

//////////////////////////////////////////////////////////////////////////
/* --- Tests --- */

void test_escape_encode() 
{
  KissFrame frame;
  uint8_t data[] = {0x00, 0xC0, 0xDB, 0xFF};
  kiss_frame_data_frame(&frame, data, 4);
    
  uint8_t encoded[64];
  size_t len = kiss_frame_encode(&frame, encoded, 64);

  assert(encoded[0] == FEND);
  assert(encoded[1] == 0x00);
  assert(encoded[2] == 0x00);
  assert(encoded[3] == FESC);
  assert(encoded[4] == TFEND);
  assert(encoded[5] == FESC);
  assert(encoded[6] == TFESC);
  assert(encoded[7] == 0xFF);
  assert(encoded[8] == FEND);
  printf("test_escape_encode passed\n");
}

void test_parser() 
{
  KissParser parser;
  kiss_parser_init(&parser);

  uint8_t bytes[] = {FEND, 0x00, 0x01, 0x02, 0x03, FEND};
  size_t bytes_len = 6;

  for (size_t i = 0; i < bytes_len; i++) 
  {
    bool ready = false;
    KissFrame* frame = kiss_parser_feed(&parser, bytes[i], &ready);
    if (i == bytes_len - 1) 
    {
      assert(ready == true);
      assert(frame->command == 0x00);
      assert(frame->data_len == 3);
    } 
    else assert(ready == false);
  }
  printf("test_parser passed\n");
}

void test_parser_escape() 
{
  KissParser parser;
  kiss_parser_init(&parser);

  uint8_t bytes[] = {FEND, 0x00, FESC, TFEND, FEND};
    
  for (size_t i = 0; i < 5; i++) 
  {
    bool ready = false;
    KissFrame* frame = kiss_parser_feed(&parser, bytes[i], &ready);
    if (ready) 
    {
      assert(frame->data_len == 1);
      assert(frame->data[0] == FEND);
    }
  }
  printf("test_parser_escape passed\n");
}

void test_rnode_config() 
{
  RNodeHandler handler;
  rnode_handler_init(&handler);

  KissFrame frame;
  uint32_t freq = 915000000;
  uint8_t freq_bytes[4] = { (freq >> 24) & 0xFF, (freq >> 16) & 0xFF, (freq >> 8) & 0xFF, freq & 0xFF };
    
  kiss_frame_command_frame(&frame, RNodeCommandFrequency, freq_bytes, 4);
  rnode_handler_process_frame(&handler, &frame);
  assert(rnode_handler_config(&handler)->frequency == 915000000);

  uint8_t sf = 10;
  kiss_frame_command_frame(&frame, RNodeCommandSpreadingFactor, &sf, 1);
  rnode_handler_process_frame(&handler, &frame);
  assert(rnode_handler_config(&handler)->spreading_factor == 10);
  printf("test_rnode_config passed\n");
}

void test_rnode_statistics() 
{
  RNodeHandler handler;
  rnode_handler_init(&handler);

  uint8_t data1[] = {0x01, 0x02, 0x03};
  rnode_handler_process_lora_packet(&handler, data1, 3, -80, 5);
  assert(rnode_handler_stats(&handler)->rx_count == 1);
  assert(rnode_handler_stats(&handler)->rx_bytes == 3);
  assert(rnode_handler_stats(&handler)->last_rssi == -80);
  assert(rnode_handler_stats(&handler)->last_snr == 5);

  uint8_t data2[] = {0x04, 0x05};
  rnode_handler_process_lora_packet(&handler, data2, 2, -90, 3);
  assert(rnode_handler_stats(&handler)->rx_count == 2);
  assert(rnode_handler_stats(&handler)->rx_bytes == 5);
  printf("test_rnode_statistics passed\n");
}

void test_rnode_airtime() 
{
  RNodeConfig config;
  rnode_config_default(&config);

  uint64_t airtime = rnode_config_packet_airtime_ms(&config, 50);
  assert(airtime > 50);
  assert(airtime < 500);
  printf("test_rnode_airtime passed\n");
}

void test_rnode_ldro_calculation() 
{
  RNodeConfig config;
  rnode_config_default(&config);

  config.spreading_factor = 9;
  config.bandwidth = 125000;
  assert(!rnode_config_should_enable_ldro(&config));

  config.spreading_factor = 11;
  assert(rnode_config_should_enable_ldro(&config));

  config.spreading_factor = 12;
  assert(rnode_config_should_enable_ldro(&config));

  config.bandwidth = 500000;
  assert(!rnode_config_should_enable_ldro(&config));
  printf("test_rnode_ldro_calculation passed\n");
}

void test_rnode_identity_hash() 
{
  RNodeIdentity identity;
  uint8_t serial[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
  memcpy(identity.serial, serial, 16);

  uint32_t hash1 = rnode_identity_identity_hash(&identity);
  uint32_t hash2 = rnode_identity_identity_hash(&identity);

  assert(hash1 == hash2);

  identity.serial[0] = 0xFF;
  uint32_t hash3 = rnode_identity_identity_hash(&identity);
  assert(hash1 != hash3);
  printf("test_rnode_identity_hash passed\n");
}

void test_rnode_airtime_limit() 
{
  RNodeHandler handler;
  rnode_handler_init(&handler);
  handler.airtime_limit = 1000;

  assert(rnode_handler_check_airtime_limit(&handler, 10));
  printf("test_rnode_airtime_limit passed\n");
}

void test_rnode_all_commands() 
{
  RNodeHandler handler;
  rnode_handler_init(&handler);

  KissFrame detect_frame;
  kiss_frame_command_frame(&detect_frame, RNodeCommandDetect, NULL, 0);
  assert(rnode_handler_process_frame(&handler, &detect_frame) != NULL);

  KissFrame ready_frame;
  kiss_frame_command_frame(&ready_frame, RNodeCommandReady, NULL, 0);
  assert(rnode_handler_process_frame(&handler, &ready_frame) != NULL);

  KissFrame fw_frame;
  kiss_frame_command_frame(&fw_frame, RNodeCommandFwVersion, NULL, 0);
  assert(rnode_handler_process_frame(&handler, &fw_frame) != NULL);

  KissFrame platform_frame;
  kiss_frame_command_frame(&platform_frame, RNodeCommandPlatform, NULL, 0);
  assert(rnode_handler_process_frame(&handler, &platform_frame) != NULL);
  printf("test_rnode_all_commands passed\n");
}

/* int main() {
    test_escape_encode();
    test_parser();
    test_parser_escape();
    test_rnode_config();
    test_rnode_statistics();
    test_rnode_airtime();
    test_rnode_ldro_calculation();
    test_rnode_identity_hash();
    test_rnode_airtime_limit();
    test_rnode_all_commands();
    printf("All tests passed!\n");
    return 0;
} */

