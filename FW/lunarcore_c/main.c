

// Global atomic state variables
static bool dio1_triggered = ATOMIC_VAR_INIT(false);
static bool packet_pending = ATOMIC_VAR_INIT(false);
static bool tx_complete = ATOMIC_VAR_INIT(false);
static uint_least32_t system_ticks = ATOMIC_VAR_INIT(0);
static uint_least32_t last_activity = ATOMIC_VAR_INIT(0);
static uint_least32_t error_count = ATOMIC_VAR_INIT(0);



// Function declarations for NodeIdentity
void node_identity_read_mac_address(uint8_t mac[6]);
void node_identity_read_hardware_serial(uint8_t serial[8]);
void node_identity_load_or_create_identity(const uint8_t hardware_serial[8], 
                                                   uint32_t *node_id, uint8_t private_key[32]);
uint32_t node_identity_generate_random_node_id(void);
void node_identity_generate_random_private_key(const uint8_t hardware_serial[8], 
                                                       uint8_t private_key[32]);
NodeIdentity node_identity_from_hardware(void);
NodeIdentity* node_identity_factory_reset(void);
void node_identity_x25519_pubkey(const NodeIdentity *self, uint8_t pubkey[32]);


// Function declarations for BatteryState
void battery_state_new(BatteryState *battery);
void battery_state_update(BatteryState *battery, uint32_t adc_value);
uint8_t battery_state_voltage_to_percentage(uint32_t mv);



// Function declarations for Stats
void stats_new(Stats *stats);
void stats_record_rx(Stats *stats, size_t len, int16_t rssi, int8_t snr);
void stats_record_tx(Stats *stats, size_t len, uint32_t airtime_ms);


// Function declarations for LedController
void led_controller_new(LedController *led);
void led_controller_set_idle(LedController *led);
void led_controller_set_active(LedController *led);
void led_controller_set_error(LedController *led);
void led_controller_flash(LedController *led, uint8_t count);
bool led_controller_update(LedController *led, uint32_t current_time);



// Function declarations for LunarCore
void lunar_core_new(LunarCore *core, Sx1262 *radio, const NodeIdentity *identity);
const NodeIdentity* lunar_core_identity(const LunarCore *core);
void lunar_core_update_battery(LunarCore *core, uint32_t adc_value);
void lunar_core_handle_dio1_interrupt(LunarCore *core);
void lunar_core_process_radio_events(LunarCore *core);
void lunar_core_process_at_command(LunarCore *core, uart_port_t uart);
void lunar_core_process_serial_byte(LunarCore *core, uint8_t byte, uart_port_t uart);
void lunar_core_configure_radio_for_protocol(LunarCore *core, Protocol protocol);
void lunar_core_handle_meshcore_frame(LunarCore *core, const Frame *frame, uart_port_t uart);
void lunar_core_handle_meshtastic_frame(LunarCore *core, const MeshtasticFrame *frame, uart_port_t uart);
void lunar_core_handle_rnode_frame(LunarCore *core, const KissFrame *frame, uart_port_t uart);

// Helper function declarations
void write_hex8(uart_port_t uart, uint8_t value);
void write_hex32(uart_port_t uart, uint32_t value);
void write_u32(uart_port_t uart, uint32_t value);
void write_i16(uart_port_t uart, int16_t value);
const char* format_battery(const BatteryState *battery, char *buf);
bool parse_u32_from_cmd(const uint8_t *cmd, size_t len, uint32_t *result);
bool parse_i8_from_cmd(const uint8_t *cmd, size_t len, int8_t *result);

// Implementation: NodeIdentity functions

void node_identity_read_mac_address(uint8_t mac[6]) 
{
  // Read MAC address from ESP32 eFuse
  esp_efuse_mac_get_default(mac); //to be changed!!! I never use ESP
}

void node_identity_read_hardware_serial(uint8_t serial[8]) 
{
  // Read hardware serial from eFuse registers
  volatile uint32_t *efuse_base = (volatile uint32_t *)0x6001A044; //to be changed!!! I never use ESP
  uint32_t word0 = *efuse_base;
  uint32_t word1 = *(efuse_base + 1);
    
  memcpy(&serial[0], &word0, 4);
  memcpy(&serial[4], &word1, 4);
}

uint32_t node_identity_generate_random_node_id(void) //to be changed!!! I never use ESP 
{
  uint8_t random_bytes[4];
  esp_fill_random(random_bytes, 4);
    
  uint32_t id;
  memcpy(&id, random_bytes, 4);
  // Set high bit to indicate random ID
  id |= 0x80000000;
  return id;
}

void node_identity_generate_random_private_key(const uint8_t hardware_serial[8],uint8_t private_key[32]) 
{
  uint8_t random_bytes[32];
  esp_fill_random(random_bytes, 32);
    
  // Combine random data with hardware serial
  uint8_t seed_input[40];
  memcpy(&seed_input[0], random_bytes, 32);
  memcpy(&seed_input[32], hardware_serial, 8);
    
  // Hash to generate private key
  sha256_hash(seed_input, 40, private_key);
  
  // Clamp for Curve25519
  private_key[0] &= 248;
  private_key[31] &= 127;
  private_key[31] |= 64;
}

void node_identity_load_or_create_identity(const uint8_t hardware_serial[8],uint32_t *node_id, uint8_t private_key[32]) 
{
  nvs_handle_t handle;
  esp_err_t err;
    
  // Try to open NVS
  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) 
  {
    // Initialize NVS flash and retry
    nvs_flash_init();
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) 
    {
      // NVS not available, use ephemeral identity
      ESP_LOGW("NodeIdentity", "NVS not available, using ephemeral identity");
      *node_id = node_identity_generate_random_node_id();
      node_identity_generate_random_private_key(hardware_serial, private_key);
      return;
    }
  }
    
  // Try to load existing identity
  uint32_t stored_node_id;
  size_t key_len = 32;
  bool has_node_id = (nvs_get_u32(handle, NVS_KEY_NODE_ID, &stored_node_id) == ESP_OK);
  bool has_private_key = (nvs_get_blob(handle, NVS_KEY_PRIVATE_KEY, private_key, &key_len) == ESP_OK) && (key_len == 32);
    
  if (has_node_id && has_private_key) 
  {
    // Found existing identity
    ESP_LOGI("NodeIdentity", "Loaded existing node identity from NVS");
    *node_id = stored_node_id;
    nvs_close(handle);
    return;
  }
    
  // Create new identity
  ESP_LOGI("NodeIdentity", "Creating new random node identity (privacy-first)");
  *node_id = node_identity_generate_random_node_id();
  node_identity_generate_random_private_key(hardware_serial, private_key);
    
  // Store in NVS
  nvs_set_u32(handle, NVS_KEY_NODE_ID, *node_id);
  nvs_set_blob(handle, NVS_KEY_PRIVATE_KEY, private_key, 32);
  nvs_commit(handle);
  nvs_close(handle);
    
  ESP_LOGI("NodeIdentity", "Stored new identity in NVS");
}

NodeIdentity node_identity_from_hardware(void) 
{
  NodeIdentity identity;
    
  // Read hardware info
  node_identity_read_mac_address(identity.mac_address);
  node_identity_read_hardware_serial(identity.hardware_serial);
    
  // Load or create identity
  node_identity_load_or_create_identity(identity.hardware_serial, 
                                          &identity.node_id, 
                                          identity.private_key);
    
  // Generate public key from private key
  ed25519_public_key(identity.private_key, identity.public_key);
  
  return identity;
}

NodeIdentity* node_identity_factory_reset(void) 
{
  // Erase NVS partition
  nvs_flash_erase_partition(NVS_NAMESPACE);
    
  ESP_LOGI("NodeIdentity", "Factory reset: erased old identity, generating new one");
    
  // Create new identity
  NodeIdentity *identity = malloc(sizeof(NodeIdentity));
  if (identity) *identity = node_identity_from_hardware();
  return identity;
}

void node_identity_x25519_pubkey(const NodeIdentity *self, uint8_t pubkey[32]) 
{
  x25519_base(self->private_key, pubkey);
}



// Implementation: Stats functions

void stats_new(Stats *stats) 
{
  stats->tx_packets = 0;
  stats->rx_packets = 0;
  stats->tx_errors = 0;
  stats->rx_errors = 0;
  stats->protocol_switches = 0;
  stats->tx_bytes = 0;
  stats->rx_bytes = 0;
  stats->uptime_seconds = 0;
  stats->last_rssi = 0;
  stats->last_snr = 0;
  stats->airtime_ms = 0;
}

void stats_record_rx(Stats *stats, size_t len, int16_t rssi, int8_t snr) 
{
  stats->rx_packets++;
  stats->rx_bytes += len;
  stats->last_rssi = rssi;
  stats->last_snr = snr;
}

void stats_record_tx(Stats *stats, size_t len, uint32_t airtime_ms) 
{
  stats->tx_packets++;
  stats->tx_bytes += len;
  stats->airtime_ms += airtime_ms;
}


// Implementation: LunarCore functions

void lunar_core_new(LunarCore *core, Sx1262 *radio, const NodeIdentity *identity) 
{
  // Initialize radio
  memcpy(&core->radio, radio, sizeof(Sx1262));
    
  // Initialize protocol handlers
  protocol_router_new(&core->router);
  frame_parser_new(&core->meshcore_parser);
  meshtastic_handler_new(&core->meshtastic, identity->node_id);
  rnode_handler_new(&core->rnode);
  //ble_manager_new(&core->ble);
    
  // Initialize stats and state
  stats_new(&core->stats);
  memcpy(&core->identity, identity, sizeof(NodeIdentity));
  battery_state_new(&core->battery);
  led_controller_new(&core->led);
    
  core->rx_active = false;
  core->serial_protocol = PROTOCOL_UNKNOWN;
  core->vext_enabled = true;
  core->last_battery_check = 0;
  core->at_buffer_len = 0;
    
  // Prepare X25519 private key
  uint8_t x25519_private[32];
  memcpy(x25519_private, identity->private_key, 32);
  x25519_private[0] &= 248;
  x25519_private[31] &= 127;
  x25519_private[31] |= 64;
    
  // Initialize session and onion routing
  session_manager_new(&core->session_manager);
  onion_router_new(&core->onion_router, x25519_private);
  route_builder_new(&core->route_builder);
    
  // Generate our universal address
  address_translator_from_public_key(identity->public_key, &core->our_address);
}

const NodeIdentity* lunar_core_identity(const LunarCore *core) 
{
  return &core->identity;
}



void lunar_core_handle_dio1_interrupt(LunarCore *core) 
{
  // Clear interrupt flag
  dio1_triggered = false;
  // Read interrupt status from radio
  uint16_t irq_status;
  if (sx1262_get_irq_status(&core->radio, &irq_status) == 0) 
  {
    // TX complete
    if (irq_status & 0x01) 
    {
      tx_complete = true;
      last_activity = system_ticks;
    }
    // RX done
    if (irq_status & 0x02) 
    {
      packet_pending = true;
      last_activity = system_ticks;
    }
    // Preamble detected (0x04) - no action needed
    // CRC error
    if (irq_status & 0x40) 
    {
      core->stats.rx_errors++;
      error_count++;
    }
    // Clear interrupt
    sx1262_clear_irq(&core->radio, irq_status);
  }
}

void lunar_core_process_radio_events(LunarCore *core) 
{
  // Handle TX complete
  if (atomic_exchange(&tx_complete, false)) 
  {
    // Restart RX mode
    if (sx1262_start_rx(&core->radio, 0) == 0) core->rx_active = true;
    led_controller_flash(&core->led, 1);
  }
  // Handle RX packet
  if (atomic_exchange(&packet_pending, false)) led_controller_flash(&core->led, 2);
}

////////////////////////////

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* Dependencies and External Definitions (Mocked or ESP-IDF based) */
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants */
#define FIRMWARE_VERSION "1.0.0"
#define WATCHDOG_TIMEOUT_SEC 30
#define BAUD_RATE 115200

/* Enums and Structs */
typedef enum 
{
  Protocol_MeshCore,
  Protocol_Meshtastic,
  Protocol_RNode,
  Protocol_Unknown
} Protocol;

typedef enum 
{
  Command_Ping,
  Command_Configure,
  Command_Transmit,
  Command_Version,
  Command_GetStats,
  Command_Reset,
  Command_Unknown
} Command;

typedef enum 
{
  RadioState_Standby,
  RadioState_Rx,
  RadioState_Tx
} RadioState;

typedef enum 
{
  RadioError_None,
  RadioError_TxTimeout,
  RadioError_BusyTimeout,
  RadioError_RxTimeout,
  RadioError_CrcError,
  RadioError_Other
} RadioError;

typedef enum 
{
  CryptoError_None,
  CryptoError_SessionError,
  CryptoError_EncryptionFailed,
  CryptoError_DecryptionFailed,
  CryptoError_BufferOverflow,
  CryptoError_InvalidFormat,
  CryptoError_NoSession,
  CryptoError_OnionError
} CryptoError;

typedef struct 
{
  uint32_t frequency;
  uint8_t spreading_factor;
  uint8_t bandwidth;
  uint8_t coding_rate;
  uint8_t tx_power;
  uint8_t sync_word;
  uint16_t preamble_length;
  bool crc_enabled;
  bool implicit_header;
  bool ldro;
} RadioConfig;

typedef struct 
{
  uint8_t command;
  uint32_t sequence;
  uint8_t data[256];
  size_t data_len;
} Frame;

typedef struct 
{
    // Meshtastic specific fields
} MeshtasticFrame;

typedef struct 
{
  uint8_t command;
  uint8_t data[256];
  size_t data_len;
} KissFrame;

typedef enum {
    KissCommand_DataFrame = 0x00
} KissCommand;

typedef struct 
{
  uint32_t tx_packets;
  uint32_t rx_packets;
  uint32_t tx_errors;
  uint32_t rx_errors;
  uint32_t uptime_seconds;
} Stats;

typedef struct 
{
  uint32_t voltage_mv;
  uint8_t percentage;
} BatteryState;

/* Forward Declarations for Methods */
uint32_t millis(void);
void send_frame(void* self, uart_port_t uart, const Frame* frame);

/* Global Atomic-like variables */
volatile bool DIO1_TRIGGERED = false;
volatile uint32_t SYSTEM_TICKS = 0;
volatile uint32_t LAST_ACTIVITY = 0;

/* LunarCore Structure (Simplified for context) */
typedef struct 
{
  void* radio; // Placeholder for Sx1262 instance
  void* rnode;
  void* router;
  void* meshtastic;
  void* ble;
  void* identity;
  void* session_manager;
  void* route_builder;
  void* onion_router;
  void* led;
  Stats stats;
  bool rx_active;
  Protocol serial_protocol;
  uint16_t our_address;
} LunarCore;

/* Implementation of Methods */

void configure_radio_for_protocol(LunarCore* self, Protocol protocol) 
{
  RadioConfig config;
  bool valid_protocol = true;

  switch (protocol) 
  {
    case Protocol_MeshCore:
    config.frequency = 915000000;
    config.spreading_factor = 9;
    config.bandwidth = 0;
    config.coding_rate = 1;
    config.tx_power = 14;
    config.sync_word = 0x12;
    config.preamble_length = 8;
    config.crc_enabled = true;
    config.implicit_header = false;
    config.ldro = false;
    break;
    
    case Protocol_Meshtastic:
    config.frequency = 906875000;
    config.spreading_factor = 11;
    config.bandwidth = 0;
    config.coding_rate = 1;
    config.tx_power = 17;
    config.sync_word = 0x2B;
    config.preamble_length = 16;
    config.crc_enabled = true;
    config.implicit_header = false;
    config.ldro = true;
    break;
    
    case Protocol_RNode: 
    {
      // Assuming rnode_config_t cfg = rnode_get_config(self->rnode);
      struct 
      { 
        uint32_t frequency; 
        uint8_t spreading_factor; 
        uint32_t bandwidth; 
        uint8_t coding_rate; 
        uint8_t tx_power; 
      } cfg; 
      // Mocking cfg retrieval
      config.frequency = cfg.frequency;
      config.spreading_factor = cfg.spreading_factor;
      switch (cfg.bandwidth) 
      {
        case 125000: config.bandwidth = 0; break;
        case 250000: config.bandwidth = 1; break;
        case 500000: config.bandwidth = 2; break;
        default: config.bandwidth = 0; break;
      }
      config.coding_rate = (cfg.coding_rate > 4) ? cfg.coding_rate - 4 : 0;
      config.tx_power = cfg.tx_power;
      config.sync_word = 0x12;
      config.preamble_length = 8;
      config.crc_enabled = true;
      config.implicit_header = false;
      config.ldro = (cfg.spreading_factor >= 11);
      break;
    }
    default:
    return;
  }
  if (radio_configure(self->radio, &config) != ESP_OK) ESP_LOGE("LunarCore", "Failed to configure radio");
  router_set_lora_protocol(self->router, protocol);
}

void handle_meshcore_frame(LunarCore* self, const Frame* frame, uart_port_t uart) 
{
  switch (frame->command) 
  {
    case Command_Ping: 
    {
      Frame response; // protocol_build_pong(frame->sequence);
      send_frame(self, uart, &response);
      break;
    }
    case Command_Configure: 
    {
      RadioConfig config;
      if (protocol_parse_config(frame->data, frame->data_len, &config)) 
      {
        if (radio_configure(self->radio, &config) == ESP_OK) 
        {
          Frame response = protocol_build_config_ack(frame->sequence);
          send_frame(self, uart, &response);
        } 
        else 
        {
          Frame response;
          if (protocol_build_error(frame->sequence, "Config failed", &response)) send_frame(self, uart, &response);
        }
      }
      break;
    }
    case Command_Transmit: 
    {
      self->rx_active = false;
      RadioError err = radio_transmit(self->radio, frame->data, frame->data_len);
      if (err == RadioError_None) 
      {
        self->stats.tx_packets++;
        Frame response = protocol_build_tx_done(frame->sequence);
        send_frame(self, uart, &response);
        radio_start_rx(self->radio, 0);
        self->rx_active = true;
      } 
      else 
      {
        self->stats.tx_errors++;
        uint8_t code = (err == RadioError_TxTimeout) ? 1 : (err == RadioError_BusyTimeout ? 2 : 255);
        Frame response;
        if (protocol_build_tx_error(frame->sequence, code, &response)) send_frame(self, uart, &response);
      }
      break;
    }
    case Command_Version: 
    {
      Frame response;
      if (protocol_build_version_response(frame->sequence, FIRMWARE_VERSION, &response)) send_frame(self, uart, &response);
      break;
    }
    case Command_GetStats: 
    {
      Frame response;
      if (protocol_build_stats_response(frame->sequence, self->stats.tx_packets, self->stats.rx_packets, self->stats.tx_errors, self->stats.rx_errors, &response)) 
      {
        send_frame(self, uart, &response);
      }
      break;
    }
    case Command_Reset: 
    {
      // radio_init(self->radio);
      self->rx_active = false;
      Frame response; // protocol_build_pong(frame->sequence);
      send_frame(self, uart, &response);
      break;
    }
    default: 
    {
      Frame response;
      if (protocol_build_error(frame->sequence, "Unknown command", &response)) send_frame(self, uart, &response);
      break;
    }
  }
  if (!self->rx_active && radio_get_state(self->radio) == RadioState_Standby) 
  {
    if (radio_start_rx(self->radio, 0) == ESP_OK) self->rx_active = true;
  }
}

void send_meshtastic_response(LunarCore* self, const uint8_t* response, size_t len, uart_port_t uart) 
{
  uint8_t serial_frame[512];
  size_t frame_len;
  if (meshtastic_build_serial_frame(self->meshtastic, response, len, serial_frame, &frame_len)) uart_write_bytes(uart, (const char*)serial_frame, frame_len);
  ble_queue_from_radio(self->ble, response, len);
  self->meshtastic_rx_count++;
  ble_notify_from_num(self->ble, self->meshtastic_rx_count);
}

void flush_meshtastic_responses(LunarCore* self, uart_port_t uart) 
{
  uint8_t response[256];
  size_t len;
  while (meshtastic_poll_pending_response(self->meshtastic, response, &len)) 
  {
    send_meshtastic_response(self, response, len, uart);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void handle_meshtastic_frame(LunarCore* self, const MeshtasticFrame* frame, uart_port_t uart) 
{
  ToRadioResponse res = meshtastic_process_toradio(self->meshtastic, frame);
  if (res.type == LoRaPacket) 
  {
    self->rx_active = false;
    if (radio_transmit(self->radio, res.data, res.len) == RadioError_None) 
    {
      self->stats.tx_packets++;
      radio_start_rx(self->radio, 0);
      self->rx_active = true;
    } 
    else self->stats.tx_errors++;
  } 
  else if (res.type == FromRadio) 
  {
    send_meshtastic_response(self, res.data, res.len, uart);
    flush_meshtastic_responses(self, uart);
  }
  if (!self->rx_active) 
  {
    if (radio_start_rx(self->radio, 0) == ESP_OK) self->rx_active = true;
  }
}

void handle_rnode_frame(LunarCore* self, const KissFrame* frame, uart_port_t uart) 
{
  uint8_t response[256]; size_t res_len;
  if (rnode_process_frame(self->rnode, frame, response, &res_len)) uart_write_bytes(uart, (const char*)response, res_len);
  if (frame->command == (uint8_t)KissCommand_DataFrame) 
  {
    uint8_t tx_data[256]; size_t tx_len;
    if (rnode_get_tx_data(self->rnode, frame, tx_data, &tx_len)) 
    {
      self->rx_active = false;
      if (radio_transmit(self->radio, tx_data, tx_len) == RadioError_None) 
      {
        self->stats.tx_packets++;
        radio_start_rx(self->radio, 0);
        self->rx_active = true;
      } 
      else self->stats.tx_errors++;
    }
  }
  if (rnode_is_online(self->rnode) && !self->rx_active) 
  {
    if (radio_start_rx(self->radio, 0) == ESP_OK) self->rx_active = true;
  }
}

void route_rx_packet(LunarCore* self, const uint8_t* data, size_t len, int16_t rssi, int8_t snr, uart_port_t uart) 
{
  switch (self->serial_protocol) 
  {
    case Protocol_MeshCore: 
    {
      Frame frame;
      if (protocol_build_receive(rssi, snr, data, len, &frame)) send_frame(self, uart, &frame);
      break;
    }
    case Protocol_Meshtastic: 
    {
      Packet pkt;
      if (meshtastic_process_lora_packet(self->meshtastic, data, len, (int32_t)rssi, (float)snr, &pkt)) 
      {
        uint8_t from_radio[512]; size_t fr_len;
        if (meshtastic_encode_fromradio_packet(&pkt, from_radio, &fr_len)) 
        {
          uint8_t serial_frame[512]; size_t sf_len;
          if (meshtastic_build_serial_frame(self->meshtastic, from_radio, fr_len, serial_frame, &sf_len)) uart_write_bytes(uart, (const char*)serial_frame, sf_len);
          //ble_notify_from_num(self->ble, self->meshtastic_rx_count);
        }
      }
      break;
    }
    case Protocol_RNode: 
    {
      KissFrame frame = rnode_process_lora_packet(self->rnode, data, len, rssi, snr);
      uint8_t encoded[512]; size_t enc_len;
      rnode_encode_kiss(&frame, encoded, &enc_len);
      uart_write_bytes(uart, (const char*)encoded, enc_len);
      break;
    }
    default:
    break;
  }
}

void check_rx(LunarCore* self, uart_port_t uart) 
{
  if (!self->rx_active) return;
  uint8_t data[256]; 
  size_t len; 
  int16_t rssi; 
  int8_t snr;
  RadioError err = radio_check_rx(self->radio, data, &len, &rssi, &snr);
  if (err == RadioError_None && len > 0) 
  {
    self->stats.rx_packets++;
    route_rx_packet(self, data, len, rssi, snr, uart);
  } 
  else if (err == RadioError_RxTimeout) radio_start_rx(self->radio, 0);
  else if (err == RadioError_CrcError) 
  {
    self->stats.rx_errors++;
    radio_start_rx(self->radio, 0);
  } 
  else if (err != RadioError_None) self->stats.rx_errors++;
}

void send_frame(void* self, uart_port_t uart, const Frame* frame) 
{
  uint8_t encoded[512];
  size_t len;
  protocol_encode_frame(frame, encoded, &len);
  uart_write_bytes(uart, (const char*)encoded, len);
}

/* Crypto and Routing */

CryptoError encrypt_for_tx(LunarCore* self, const uint8_t* plaintext, size_t plain_len, const uint8_t recipient_public[32], bool use_onion, uint8_t* output, size_t* out_len) 
{
  Session* session = session_manager_get_session(self->session_manager, recipient_public);
  if (!session) 
  {
    uint8_t shared[32];
    x25519(shared, identity_get_private(self->identity), recipient_public);
    session_manager_create_session(self->session_manager, shared, recipient_public);
    session = session_manager_get_session(self->session_manager, recipient_public);
    if (!session) return CryptoError_SessionError;
  }
  uint8_t header[48], ciphertext[256]; size_t cipher_len;
  if (session_encrypt(session, plaintext, plain_len, header, ciphertext, &cipher_len) != ESP_OK) return CryptoError_EncryptionFailed;
  uint8_t session_encrypted[304]; size_t se_len = 0;
  memcpy(session_encrypted, header, 48); se_len += 48;
  memcpy(session_encrypted + 48, ciphertext, cipher_len); se_len += cipher_len;
  uint8_t payload_for_wire[304]; size_t pfw_len = se_len;
  memcpy(payload_for_wire, session_encrypted, se_len);
  if (use_onion && route_builder_relay_count(self->route_builder) >= 3) 
  {
    // Onion logic...
  }
  uint16_t dest_addr = address_translator_from_public(recipient_public);
  uint32_t session_hint = session_derive_hint(session, millis() / 1000);
  WirePacket wire_packet = wire_packet_new_data(dest_addr, session_hint, payload_for_wire, (pfw_len > 214 ? 214 : pfw_len));
  wire_packet_encode(&wire_packet, output, out_len);
  return CryptoError_None;
}


void IRAM_ATTR timer_tick_isr(void* arg) 
{
  SYSTEM_TICKS++;
}

uint32_t millis(void) 
{
  return SYSTEM_TICKS;
}


/* Utilities */
size_t write_u32_to_buf(uint32_t val, uint8_t* buf) 
{
  if (val == 0) 
  {
    buf[0] = '0';
    return 1;
  }
  uint8_t digits[10];
  size_t count = 0;
  while (val > 0) 
  {
    digits[count++] = '0' + (val % 10);
    val /= 10;
  }
  for (size_t i = 0; i < count; i++) buf[i] = digits[count - 1 - i];
  return count;
}

const char* format_battery(const BatteryState* battery, uint8_t buf[32]) 
{
  size_t idx = 0;
  const char* prefix = "Battery: ";
  size_t pre_len = strlen(prefix);
  memcpy(buf + idx, prefix, pre_len);
  idx += pre_len;
  idx += write_u32_to_buf(battery->voltage_mv, buf + idx);
  const char* suffix = "mV (";
  size_t suf_len = strlen(suffix);
  memcpy(buf + idx, suffix, suf_len);
  idx += suf_len;
  idx += write_u32_to_buf((uint32_t)battery->percentage, buf + idx);
  const char* end = "%)";
  size_t end_len = strlen(end);
  memcpy(buf + idx, end, end_len);
  idx += end_len;
  buf[idx] = '\0';
  return (const char*)buf;
}

void write_u32(uart_port_t uart, uint32_t val) 
{
  uint8_t buf[10];
  size_t len = write_u32_to_buf(val, buf);
  uart_write_bytes(uart, (const char*)buf, len);
}

void write_i16(uart_port_t uart, int16_t val) 
{
  if (val < 0) 
  {
    uart_write_bytes(uart, "-", 1);
    write_u32(uart, (uint32_t)(-val));
  } 
  else write_u32(uart, (uint32_t)val);
}

void write_hex8(uart_port_t uart, uint8_t val) 
{
  const char* HEX = "0123456789ABCDEF";
  char buf[2] = {HEX[val >> 4], HEX[val & 0xF]};
  uart_write_bytes(uart, buf, 2);
}

void write_hex32(uart_port_t uart, uint32_t val) 
{
  const char* HEX = "0123456789ABCDEF";
  char buf[8];
  for (int i = 0; i < 8; i++) buf[7 - i] = HEX[(val >> (i * 4)) & 0xF];
  uart_write_bytes(uart, buf, 8);
}

uint32_t* parse_u32_from_cmd(const uint8_t* bytes, size_t len, uint32_t* out) 
{
  uint32_t result = 0;
  bool found_digit = false;
  for (size_t i = 0; i < len; i++) 
  {
    if (bytes[i] >= '0' && bytes[i] <= '9') 
    {
      result = result * 10 + (bytes[i] - '0');
      found_digit = true;
    } 
    else if (found_digit) break;
  }
  if (found_digit) 
  { 
    *out = result; 
    return out; 
  }
  return NULL;
}

int8_t* parse_i8_from_cmd(const uint8_t* bytes, size_t len, int8_t* out) 
{
  int32_t result = 0;
  bool negative = false, found_digit = false, started = false;
  for (size_t i = 0; i < len; i++) 
  {
    if (bytes[i] == '-' && !started) 
    { 
      negative = true; 
      started = true; 
    }
    else if (bytes[i] >= '0' && bytes[i] <= '9') 
    {
      result = result * 10 + (bytes[i] - '0');
      found_digit = true; started = true;
    } 
    else if (found_digit) break;
  }
  if (found_digit) 
  {
    int32_t val = negative ? -result : result;
    if (val >= -128 && val <= 127) 
    { 
      *out = (int8_t)val; 
      return out; 
    }
  }
  return NULL;
}


void lunar_core_process_serial_byte(LunarCore *core, uint8_t byte, uart_port_t uart) 
{
  // Protocol detection
  if (core->serial_protocol == PROTOCOL_UNKNOWN) 
  {
    TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
    Protocol detected = protocol_detector_feed(&transport->detector, byte);
    if (detected != PROTOCOL_UNKNOWN) 
    {
      core->serial_protocol = detected;
      core->stats.protocol_switches++;
      ESP_LOGI("LunarCore", "Protocol detected: %s", protocol_name(detected));
      lunar_core_configure_radio_for_protocol(core, detected);
    }
  }
  // Process byte according to detected protocol
  switch (core->serial_protocol) 
  {
    case PROTOCOL_MESHCORE: 
    {
      Frame *frame = frame_parser_feed(&core->meshcore_parser, byte);
      if (frame != NULL) 
      {
        lunar_core_handle_meshcore_frame(core, frame, uart);
        TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
        protocol_detector_confirm_frame(&transport->detector);
      }
      break;
    }
        
    case PROTOCOL_MESHTASTIC: 
    {
      MeshtasticFrame *frame = meshtastic_handler_feed_serial(&core->meshtastic, byte);
      if (frame != NULL) 
      {
        lunar_core_handle_meshtastic_frame(core, frame, uart);
        TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
        protocol_detector_confirm_frame(&transport->detector);
      }
      break;
    }
        
    case PROTOCOL_RNODE: 
    {
      KissFrame *frame = rnode_handler_feed_serial(&core->rnode, byte);
      if (frame != NULL) 
      {
        lunar_core_handle_rnode_frame(core, frame, uart);
        TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
        protocol_detector_confirm_frame(&transport->detector);
      }
      break;
    }
        
    default:
    break;
  }
}


//Main lunarcore function

void main(void) 
{
  run_lunarcore();
}



/* Atomic variables for thread-safe counters and timestamps */
static _Atomic uint32_t DIO1_COUNT = 0;
static _Atomic uint32_t LAST_ACTIVITY = 0;

/* Helper functions to simulate Rust functionality */
uint32_t millis() 
{
  return (uint32_t)(esp_timer_get_time() / 1000);
}

void IRAM_ATTR dio1_isr(void* arg) 
{
  atomic_fetch_add(&DIO1_COUNT, 1);
}



/* Main Entry Point: run_lunarcore */
void run_lunarcore(void) 
{
  // identity logic
  node_identity_t identity = node_identity_from_hardware();
  printf("[INIT] Node ID: %08X\r\n",(uint32_t)identity.node_id);
  // GPIO Setup (Peripherals::take equivalent)
  gpio_init();
  // OLED Reset and I2C Setup
  i2c_init();
  oled_init();
  //display starting animation
  status_display_boot_animation(&status_display, (void (*)(uint32_t))vTaskDelay);
  // SPI and LoRa Radio Setup
  spi_init();
  sx1262_t radio = 
  {
    .spi = spi_handle,
    .nss = GPIO_NUM_8,
    .reset = GPIO_NUM_12,
    .busy = GPIO_NUM_13,
    .dio1 = GPIO_NUM_14
  };
  radio_init();
  retarget_init();  
  lunarcore_logic_init();

  // LunarCore Logic Init
  lunar_core_t lunarcore;
  memset(&lunarcore, 0, sizeof(lunarcore));
  lunarcore.radio = radio;
  lunarcore.identity = identity;
  lunarcore.repeater_enabled = load_repeater_setting();
  printf("[INIT] LunarCore OK (repeater: %s)\r\n", lunarcore.repeater_enabled ? "ON" : "OFF");
  //ble_init(); //not used
  // UART Init
  uart_init();
  init_watchdog();
  // ADC Init
  adc_init();

  printf("========================================\r\n");
  printf("  Protocols: MeshCore, Meshtastic, KISS\r\n");
  printf("  Waiting for protocol detection...\r\n");
  printf("========================================\r\n");

  radio_config();

  radio_start_rx();

  uint32_t last_second = 0;
  uint32_t battery_check_interval = 0;
  uint32_t last_beacon_time = 0;

  uint32_t beacon_interval = 3000 + (lunarcore.identity.public_key[0] % 2000);

  uint8_t beacon_data[8];
  memcpy(&beacon_data[0], "TEST", 4);
  memcpy(&beacon_data[4], &lunarcore.identity.public_key[0], 4);

  uint16_t current_irq = 0;
  uint16_t last_nonzero_irq = 0;

  // Infinite Loop
  while (1) 
  {
    uint32_t now = millis();
    // Beacon Logic
    if (!lunar_core_app_connected(&lunarcore) && (now - last_beacon_time) >= beacon_interval) 
    {
      last_beacon_time = now;
      lunarcore.rx_active = false;
      if (sx1262_transmit(&lunarcore.radio, beacon_data, sizeof(beacon_data)) == ESP_OK) 
      {
        lunarcore.stats.tx_packets += 1;
        lunarcore.rx_active = true;
      }
    }
    feed_watchdog();
    lunar_core_process_radio_events(&lunarcore, UART_NUM_0);
    // Serial Processing
    uint8_t byte;
    while (uart_read_bytes(UART_NUM_0, &byte, 1, 0) > 0) 
    {
      lunar_core_process_serial_byte(&lunarcore, byte, UART_NUM_0);
      atomic_store_explicit(&LAST_ACTIVITY, now, memory_order_relaxed);
    }
    // Radio IRQ Processing
    uint16_t irq;
    if (sx1262_get_irq_status(&lunarcore.radio, &irq) == ESP_OK) 
    {
      current_irq = irq;
      if (irq != 0) 
      {
        last_nonzero_irq = irq;
        // RxError, CRC error
        if (irq & 0x40) lunarcore.stats.rx_errors++;
        // RxDone and no error, process RX packet
        if ((irq & 0x02) && !(irq & 0x40)) 
        {
          uint8_t rx_buffer[256];
          size_t rx_len = 0;
          int16_t rssi;
          int8_t snr;
          if (sx1262_read_packet(&lunarcore.radio, rx_buffer, &rx_len, &rssi, &snr) == ESP_OK) 
          {
            lunarcore.stats.rx_packets += 1;
            lunarcore.stats.last_rssi = rssi;
            lunar_core_route_rx_packet(&lunarcore, rx_buffer, rx_len, rssi, snr, UART_NUM_0);
            if (lunar_core_maybe_relay_packet(&lunarcore, rx_buffer, rx_len, now)) led_flash(&lunarcore.led, 3);
            else led_flash(&lunarcore.led, 2);
          } 
          else lunarcore.stats.rx_errors += 1;
        }
        sx1262_clear_irq(&lunarcore.radio, irq);
        // TxDone or RxDone -> restart RX
        if ((irq & 0x01) || (irq & 0x02)) 
        {
          sx1262_start_rx(&lunarcore.radio, 0);
          lunarcore.rx_active = true;
        }
      }
    }
    // LED Update
    if (led_update(&lunarcore.led, now)) 
    {
      if (lunarcore.led.is_on) gpio_set_level(GPIO_NUM_35, 1);
      else gpio_set_level(GPIO_NUM_35, 0);
    }
    // Battery Check (10s interval)
    if ((now - battery_check_interval) >= 10000) 
    {
      battery_check_interval = now;
      int adc_raw;
      if (adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &adc_raw) == ESP_OK) lunar_core_update_battery(&lunarcore, (uint32_t)adc_raw);
    }
    // BLE and Protocol Logic
/*     {
      uint32_t ble_now = ble_connection_count(&lunarcore.ble);
      if (lunarcore.serial_protocol != Protocol_Unknown && ble_now == 0 && lunarcore.last_serial_rx > 0 && (now - lunarcore.last_serial_rx) > 30000) 
      {
        ESP_LOGI(TAG, "Serial idle 30s, resetting protocol");
        lunar_core_reset_protocol(&lunarcore);
      }
      if (lunarcore.prev_ble_connections > 0 && ble_now == 0 && (lunarcore.last_serial_rx == 0 || (now - lunarcore.last_serial_rx) > 5000)) 
      {
        ESP_LOGI(TAG, "BLE disconnected, resetting protocol");
        lunar_core_reset_protocol(&lunarcore);
      }
      lunarcore.prev_ble_connections = ble_now;
    } */
    // Second-based periodic tasks
    uint32_t current_second = now / 1000;
    if (current_second > last_second) 
    {
      last_second = current_second;
      lunarcore.stats.uptime_seconds = current_second;
      if (current_second % 2 == 0) 
      {
        const char* protocol_name;
        switch (lunarcore.serial_protocol) 
        {
          case Protocol_MeshCore:   protocol_name = "MeshCore"; break;
          case Protocol_Meshtastic: protocol_name = "Meshtastic"; break;
          case Protocol_RNode:      protocol_name = "RNode/KISS"; break;
          case Protocol_AtCommand:  protocol_name = "AT Command"; break;
          default:                  protocol_name = "Detecting..."; break;
        }
        uint8_t chip_mode = 0, cmd_status = 0;
        sx1262_get_status(&lunarcore.radio, &chip_mode, &cmd_status);
        uint16_t device_errors = sx1262_get_errors(&lunarcore.radio);
        status_content_t status = 
        {
          .protocol = protocol_name,
          .node_id = lunarcore.identity.node_id,
          .battery_pct = lunarcore.battery.percentage,
          .rx_count = lunarcore.stats.rx_packets,
          .tx_count = lunarcore.stats.tx_packets,
          .rssi = lunarcore.stats.last_rssi,
          .connected = (lunarcore.serial_protocol != Protocol_Unknown),
          .irq_status = current_irq,
          .last_irq = last_nonzero_irq,
          .dio1_count = atomic_load_explicit(&DIO1_COUNT, memory_order_relaxed),
          .chip_mode = chip_mode,
          .device_errors = device_errors,
          .repeater_active = lunar_core_repeater_active(&lunarcore),
          .relay_count = lunarcore.relay_count
        };
        status_display_show_status(&status_display, &status);
      }
      if (current_second % 60 == 0) 
      {
/*         ESP_LOGI(TAG, "Uptime: %lus, RX: %lu, TX: %lu",
        (unsigned long)current_second,
        (unsigned long)lunarcore.stats.rx_packets,
        (unsigned long)lunarcore.stats.tx_packets); */
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
