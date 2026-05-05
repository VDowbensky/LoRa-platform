// Required  standard library headers
#include "bsp.h"

// Include module headers (these would need to be created separately)
#include "crypto.h"
#include "rng.h"
#include "sx1262.h"
#include "protocol.h"
#include "protocol_router.h"
#include "meshtastic.h"
#include "rnode.h"
#include "ble.h"
#include "display.h"
#include "transport.h"
#include "session.h"
#include "onion.h"

// Firmware version string
#define FIRMWARE_VERSION "LunarCore v1.0.0"

// Configuration constants
#define BAUD_RATE 115200
#define RX_BUFFER_SIZE 512
#define DEFAULT_FREQUENCY 915000000
#define BATTERY_DIVIDER_RATIO 4.9f
#define ADC_VREF_MV 3300
#define ADC_MAX_VALUE 4095
#define BATTERY_LOW_MV 3400
#define BATTERY_CRITICAL_MV 3200
#define WATCHDOG_TIMEOUT_SEC 30

// LED blink intervals
#define LED_BLINK_IDLE 2000
#define LED_BLINK_ACTIVE 500
#define LED_BLINK_ERROR 100


// NVS storage keys
#define NVS_NAMESPACE "lunarcore"
#define NVS_KEY_NODE_ID "node_id"
#define NVS_KEY_PRIVATE_KEY "priv_key"

// Maximum sizes
#define MAX_AT_BUFFER_SIZE 128
#define MAX_HEAP_VEC_SIZE 256
#define MAX_ONION_DATA_SIZE 237

// Global atomic state variables
static atomic_bool dio1_triggered = ATOMIC_VAR_INIT(false);
static atomic_bool packet_pending = ATOMIC_VAR_INIT(false);
static atomic_bool tx_complete = ATOMIC_VAR_INIT(false);
static atomic_uint_least32_t system_ticks = ATOMIC_VAR_INIT(0);
static atomic_uint_least32_t last_activity = ATOMIC_VAR_INIT(0);
static atomic_uint_least32_t error_count = ATOMIC_VAR_INIT(0);

// CryptoError enumeration
typedef enum 
{
  CRYPTO_ERROR_SESSION_ERROR,
  CRYPTO_ERROR_ENCRYPTION_FAILED,
  CRYPTO_ERROR_DECRYPTION_FAILED,
  CRYPTO_ERROR_NO_SESSION,
  CRYPTO_ERROR_ONION_ERROR,
  CRYPTO_ERROR_INVALID_FORMAT,
  CRYPTO_ERROR_BUFFER_OVERFLOW
} CryptoError;

// DecryptResult enumeration and structure
typedef enum 
{
  DECRYPT_RESULT_PLAINTEXT,
  DECRYPT_RESULT_FORWARD
} DecryptResultType;

typedef struct 
{
  DecryptResultType type;
  union 
  {
    struct 
    {
      uint8_t data[MAX_HEAP_VEC_SIZE];
      size_t len;
    } plaintext;
    struct 
    {
      uint16_t next_hop;
      uint8_t data[MAX_ONION_DATA_SIZE];
      size_t len;
    } forward;
  };
} DecryptResult;

// NodeIdentity structure
typedef struct 
{
  uint32_t node_id;
  uint8_t mac_address[6];
  uint8_t hardware_serial[8];
  uint8_t public_key[32];
  uint8_t private_key[32];
} NodeIdentity;

// Function declarations for NodeIdentity
static void node_identity_read_mac_address(uint8_t mac[6]);
static void node_identity_read_hardware_serial(uint8_t serial[8]);
static void node_identity_load_or_create_identity(const uint8_t hardware_serial[8], 
                                                   uint32_t *node_id, uint8_t private_key[32]);
static uint32_t node_identity_generate_random_node_id(void);
static void node_identity_generate_random_private_key(const uint8_t hardware_serial[8], 
                                                       uint8_t private_key[32]);
static NodeIdentity node_identity_from_hardware(void);
static NodeIdentity* node_identity_factory_reset(void);
static void node_identity_x25519_pubkey(const NodeIdentity *self, uint8_t pubkey[32]);

// BatteryState structure
typedef struct 
{
  uint32_t voltage_mv;
  uint8_t percentage;
  bool is_charging;
  bool is_low;
  bool is_critical;
} BatteryState;

// Function declarations for BatteryState
static void battery_state_new(BatteryState *battery);
static void battery_state_update(BatteryState *battery, uint32_t adc_value);
static uint8_t battery_state_voltage_to_percentage(uint32_t mv);

// Stats structure
typedef struct 
{
  uint32_t tx_packets;
  uint32_t rx_packets;
  uint32_t tx_errors;
  uint32_t rx_errors;
  uint32_t protocol_switches;
  uint64_t tx_bytes;
  uint64_t rx_bytes;
  uint32_t uptime_seconds;
  int16_t last_rssi;
  int8_t last_snr;
  uint64_t airtime_ms;
} Stats;

// Function declarations for Stats
static void stats_new(Stats *stats);
static void stats_record_rx(Stats *stats, size_t len, int16_t rssi, int8_t snr);
static void stats_record_tx(Stats *stats, size_t len, uint32_t airtime_ms);

// LedController structure
typedef struct 
{
  bool is_on;
  uint32_t last_toggle;
  uint32_t interval_ms;
  uint8_t blink_count;
  uint8_t remaining;
} LedController;

// Function declarations for LedController
static void led_controller_new(LedController *led);
static void led_controller_set_idle(LedController *led);
static void led_controller_set_active(LedController *led);
static void led_controller_set_error(LedController *led);
static void led_controller_flash(LedController *led, uint8_t count);
static bool led_controller_update(LedController *led, uint32_t current_time);

// LunarCore structure
typedef struct 
{
  Sx1262 radio;
  ProtocolRouter router;
  FrameParser meshcore_parser;
  MeshtasticHandler meshtastic;
  RNodeHandler rnode;
  //BleManager ble;
  Stats stats;
  NodeIdentity identity;
  BatteryState battery;
  LedController led;
  bool rx_active;
  Protocol serial_protocol;
  bool vext_enabled;
  uint32_t last_battery_check;
  uint8_t at_buffer[MAX_AT_BUFFER_SIZE];
  size_t at_buffer_len;
  SessionManager session_manager;
  OnionRouter onion_router;
  RouteBuilder route_builder;
  UniversalAddress our_address;
} LunarCore;

// Function declarations for LunarCore
static void lunar_core_new(LunarCore *core, Sx1262 *radio, const NodeIdentity *identity);
static const NodeIdentity* lunar_core_identity(const LunarCore *core);
static void lunar_core_update_battery(LunarCore *core, uint32_t adc_value);
static void lunar_core_handle_dio1_interrupt(LunarCore *core);
static void lunar_core_process_radio_events(LunarCore *core);
static void lunar_core_process_at_command(LunarCore *core, uart_port_t uart);
static void lunar_core_process_serial_byte(LunarCore *core, uint8_t byte, uart_port_t uart);
static void lunar_core_configure_radio_for_protocol(LunarCore *core, Protocol protocol);
static void lunar_core_handle_meshcore_frame(LunarCore *core, const Frame *frame, uart_port_t uart);
static void lunar_core_handle_meshtastic_frame(LunarCore *core, const MeshtasticFrame *frame, uart_port_t uart);
static void lunar_core_handle_rnode_frame(LunarCore *core, const KissFrame *frame, uart_port_t uart);

// Helper function declarations
static void write_hex8(uart_port_t uart, uint8_t value);
static void write_hex32(uart_port_t uart, uint32_t value);
static void write_u32(uart_port_t uart, uint32_t value);
static void write_i16(uart_port_t uart, int16_t value);
static const char* format_battery(const BatteryState *battery, char *buf);
static bool parse_u32_from_cmd(const uint8_t *cmd, size_t len, uint32_t *result);
static bool parse_i8_from_cmd(const uint8_t *cmd, size_t len, int8_t *result);

// Implementation: NodeIdentity functions

static void node_identity_read_mac_address(uint8_t mac[6]) 
{
  // Read MAC address from ESP32 eFuse
  esp_efuse_mac_get_default(mac); //to be changed!!! I never use ESP
}

static void node_identity_read_hardware_serial(uint8_t serial[8]) 
{
  // Read hardware serial from eFuse registers
  volatile uint32_t *efuse_base = (volatile uint32_t *)0x6001A044; //to be changed!!! I never use ESP
  uint32_t word0 = *efuse_base;
  uint32_t word1 = *(efuse_base + 1);
    
  memcpy(&serial[0], &word0, 4);
  memcpy(&serial[4], &word1, 4);
}

static uint32_t node_identity_generate_random_node_id(void) //to be changed!!! I never use ESP 
{
  uint8_t random_bytes[4];
  esp_fill_random(random_bytes, 4);
    
  uint32_t id;
  memcpy(&id, random_bytes, 4);
  // Set high bit to indicate random ID
  id |= 0x80000000;
  return id;
}

static void node_identity_generate_random_private_key(const uint8_t hardware_serial[8],uint8_t private_key[32]) 
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

static void node_identity_load_or_create_identity(const uint8_t hardware_serial[8],uint32_t *node_id, uint8_t private_key[32]) 
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

static NodeIdentity node_identity_from_hardware(void) 
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

static NodeIdentity* node_identity_factory_reset(void) 
{
  // Erase NVS partition
  nvs_flash_erase_partition(NVS_NAMESPACE);
    
  ESP_LOGI("NodeIdentity", "Factory reset: erased old identity, generating new one");
    
  // Create new identity
  NodeIdentity *identity = malloc(sizeof(NodeIdentity));
  if (identity) *identity = node_identity_from_hardware();
  return identity;
}

static void node_identity_x25519_pubkey(const NodeIdentity *self, uint8_t pubkey[32]) 
{
  x25519_base(self->private_key, pubkey);
}

// Implementation: BatteryState functions

static void battery_state_new(BatteryState *battery) 
{
  battery->voltage_mv = 0;
  battery->percentage = 0;
  battery->is_charging = false;
  battery->is_low = false;
  battery->is_critical = false;
}

static uint8_t battery_state_voltage_to_percentage(uint32_t mv) 
{
  // Voltage to percentage lookup curve
  typedef struct 
  {
    uint32_t voltage;
    uint8_t percentage;
  } CurvePoint;
    
  static const CurvePoint curve[11] = 
  {
    {4200, 100}, {4100, 90}, {4000, 80}, {3900, 70}, {3800, 60},
    {3700, 50}, {3600, 40}, {3500, 30}, {3400, 20}, {3300, 10}, {3000, 0}
  };
    
    if (mv >= curve[0].voltage) return 100;
    if (mv <= curve[10].voltage) return 0;
    
    // Linear interpolation between curve points
    for (int i = 0; i < 10; i++) 
    {
      if (mv >= curve[i + 1].voltage && mv <= curve[i].voltage) {
      uint32_t v_range = curve[i].voltage - curve[i + 1].voltage;
      uint8_t p_range = curve[i].percentage - curve[i + 1].percentage;
      uint32_t v_offset = mv - curve[i + 1].voltage;
  
      return curve[i + 1].percentage + ((v_offset * p_range) / v_range);
    }
  }
  return 50;
}

static void battery_state_update(BatteryState *battery, uint32_t adc_value) 
{
  // Convert ADC reading to voltage
  uint32_t vadc_mv = (adc_value * ADC_VREF_MV) / ADC_MAX_VALUE;
  battery->voltage_mv = (uint32_t)(vadc_mv * BATTERY_DIVIDER_RATIO);
    
  // Calculate percentage
  battery->percentage = battery_state_voltage_to_percentage(battery->voltage_mv);
    
  // Update status flags
  battery->is_low = battery->voltage_mv < BATTERY_LOW_MV;
  battery->is_critical = battery->voltage_mv < BATTERY_CRITICAL_MV;
  battery->is_charging = battery->voltage_mv > 4200;
}

// Implementation: Stats functions

static void stats_new(Stats *stats) 
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

static void stats_record_rx(Stats *stats, size_t len, int16_t rssi, int8_t snr) 
{
  stats->rx_packets++;
  stats->rx_bytes += len;
  stats->last_rssi = rssi;
  stats->last_snr = snr;
}

static void stats_record_tx(Stats *stats, size_t len, uint32_t airtime_ms) 
{
  stats->tx_packets++;
  stats->tx_bytes += len;
  stats->airtime_ms += airtime_ms;
}

// Implementation: LedController functions

static void led_controller_new(LedController *led) 
{
  led->is_on = false;
  led->last_toggle = 0;
  led->interval_ms = LED_BLINK_IDLE;
  led->blink_count = 0;
  led->remaining = 0;
}

static void led_controller_set_idle(LedController *led) 
{
  led->interval_ms = LED_BLINK_IDLE;
  led->blink_count = 0;
}

static void led_controller_set_active(LedController *led) 
{
  led->interval_ms = LED_BLINK_ACTIVE;
  led->blink_count = 0;
}

static void led_controller_set_error(LedController *led) 
{
  led->interval_ms = LED_BLINK_ERROR;
  led->blink_count = 0;
}

static void led_controller_flash(LedController *led, uint8_t count) 
{
  led->blink_count = count;
  led->remaining = count * 2;
  led->interval_ms = 100;
}

static bool led_controller_update(LedController *led, uint32_t current_time) 
{
  uint32_t elapsed = current_time - led->last_toggle;
  if (elapsed >= led->interval_ms) 
  {
    led->last_toggle = current_time;
        
    if (led->blink_count > 0) 
    {
      if (led->remaining > 0) 
      {
        led->remaining--;
        led->is_on = !led->is_on;
        return true;
      } else 
      {
        // Blink sequence complete, return to idle
        led_controller_set_idle(led);
      }
    } 
    else 
    {
      led->is_on = !led->is_on;
      return true;
    }
  }
  return false;
}

// Implementation: LunarCore functions

static void lunar_core_new(LunarCore *core, Sx1262 *radio, const NodeIdentity *identity) 
{
  // Initialize radio
  memcpy(&core->radio, radio, sizeof(Sx1262));
    
  // Initialize protocol handlers
  protocol_router_new(&core->router);
  frame_parser_new(&core->meshcore_parser);
  meshtastic_handler_new(&core->meshtastic, identity->node_id);
  rnode_handler_new(&core->rnode);
  ble_manager_new(&core->ble);
    
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

static const NodeIdentity* lunar_core_identity(const LunarCore *core) 
{
  return &core->identity;
}

static void lunar_core_update_battery(LunarCore *core, uint32_t adc_value) 
{
  battery_state_update(&core->battery, adc_value);
    
  if (core->battery.is_critical) 
  {
    led_controller_set_error(&core->led);
    ESP_LOGW("LunarCore", "Battery critical: %dmV", core->battery.voltage_mv);
  } 
  else if (core->battery.is_low) 
  {
    ESP_LOGI("LunarCore", "Battery low: %dmV (%d%%)", 
    core->battery.voltage_mv, core->battery.percentage);
  }
}

static void lunar_core_handle_dio1_interrupt(LunarCore *core) 
{
  // Clear interrupt flag
  atomic_store(&dio1_triggered, false);
    
  // Read interrupt status from radio
  uint16_t irq_status;
  if (sx1262_get_irq_status(&core->radio, &irq_status) == 0) 
  {
    // TX complete
    if (irq_status & 0x01) 
    {
      atomic_store(&tx_complete, true);
      atomic_store(&last_activity, atomic_load(&system_ticks));
    }
    // RX done
    if (irq_status & 0x02) 
    {
      atomic_store(&packet_pending, true);
      atomic_store(&last_activity, atomic_load(&system_ticks));
    }
    // Preamble detected (0x04) - no action needed
    // CRC error
    if (irq_status & 0x40) 
    {
      core->stats.rx_errors++;
      atomic_fetch_add(&error_count, 1);
    }
    // Clear interrupt
    sx1262_clear_irq(&core->radio, irq_status);
  }
}

static void lunar_core_process_radio_events(LunarCore *core) 
{
  // Handle TX complete
  if (atomic_exchange(&tx_complete, false)) 
  {
    // Restart RX mode
    if (sx1262_start_rx(&core->radio, 0) == 0) core->rx_active = true;
    led_controller_flash(&core->led, 1);
  }
  // Handle RX packet
  if (atomic_exchange(&packet_pending, false)) 
  {
    led_controller_flash(&core->led, 2);
  }
}

static void lunar_core_process_at_command(LunarCore *core, uart_port_t uart) 
{
  // Convert command to uppercase
  uint8_t cmd_upper[MAX_AT_BUFFER_SIZE];
  size_t len = core->at_buffer_len < MAX_AT_BUFFER_SIZE ? core->at_buffer_len : MAX_AT_BUFFER_SIZE;
  for (size_t i = 0; i < len; i++) 
  {
    cmd_upper[i] = toupper(core->at_buffer[i]);
  }
  // Send newline
  uart_write_bytes(uart, "\r\n", 2);
  // Process command
  if (memcmp(cmd_upper, "AT+VERSION", 10) == 0 || memcmp(cmd_upper, "ATI", 3) == 0) 
  {
    uart_write_bytes(uart, "LunarCore v1.0.0\r\n", 18);
    uart_write_bytes(uart, "Unified Mesh Bridge Firmware\r\n", 30);
    uart_write_bytes(uart, "Protocols: MeshCore, Meshtastic, RNode/KISS\r\n", 46);
    uart_write_bytes(uart, "OK\r\n", 4);
  }
  else if (memcmp(cmd_upper, "AT+STATUS", 9) == 0) 
  {
    uart_write_bytes(uart, "Status: ", 8);
    if (core->rx_active) uart_write_bytes(uart, "RX Active\r\n", 11);
    else uart_write_bytes(uart, "Idle\r\n", 6);
    char buf[32];
    const char *battery_str = format_battery(&core->battery, buf);
    uart_write_bytes(uart, battery_str, strlen(battery_str));
    uart_write_bytes(uart, "\r\n", 2);
        
    uart_write_bytes(uart, "TX: ", 4);
    write_u32(uart, core->stats.tx_packets);
    uart_write_bytes(uart, " RX: ", 5);
    write_u32(uart, core->stats.rx_packets);
    uart_write_bytes(uart, "\r\n", 2);
        
    uart_write_bytes(uart, "OK\r\n", 4);
  }
  else if (memcmp(cmd_upper, "AT+NODEID", 9) == 0) 
  {
    uart_write_bytes(uart, "Node ID: ", 9);
    write_hex32(uart, core->identity.node_id);
    uart_write_bytes(uart, "\r\n", 2);
    uart_write_bytes(uart, "OK\r\n", 4);
  }
  else if (memcmp(cmd_upper, "AT+MAC", 6) == 0) 
  {
    uart_write_bytes(uart, "MAC: ", 5);
    for (int i = 0; i < 6; i++) 
    {
      write_hex8(uart, core->identity.mac_address[i]);
      if (i < 5) uart_write_bytes(uart, ":", 1);
    }
    uart_write_bytes(uart, "\r\n", 2);
    uart_write_bytes(uart, "OK\r\n", 4);
  }
  else if (memcmp(cmd_upper, "AT+FREQ=", 8) == 0) 
  {
    uint32_t freq;
    if (parse_u32_from_cmd(&cmd_upper[8], len - 8, &freq)) 
    {
      ESP_LOGI("LunarCore", "Setting frequency to %d Hz", freq);
      RadioConfig config = core->radio.config;
      config.frequency = freq;
      if (sx1262_configure(&core->radio, &config) == 0) uart_write_bytes(uart, "OK\r\n", 4);
      else uart_write_bytes(uart, "ERROR\r\n", 7);
    } 
    else uart_write_bytes(uart, "ERROR: Invalid frequency\r\n", 26);
  }
  else if (memcmp(cmd_upper, "AT+SF=", 6) == 0) 
  {
    uint32_t sf;
    if (parse_u32_from_cmd(&cmd_upper[6], len - 6, &sf)) 
    {
      if (sf >= 7 && sf <= 12) 
      {
        ESP_LOGI("LunarCore", "Setting SF to %d", sf);
        RadioConfig config = core->radio.config;
        config.spreading_factor = (uint8_t)sf;
        if (sx1262_configure(&core->radio, &config) == 0) uart_write_bytes(uart, "OK\r\n", 4);
        else uart_write_bytes(uart, "ERROR\r\n", 7);
      } 
      else uart_write_bytes(uart, "ERROR: SF must be 7-12\r\n", 24);
    } 
    else uart_write_bytes(uart, "ERROR: Invalid SF\r\n", 19);
  }
  else if (memcmp(cmd_upper, "AT+TXPOWER=", 11) == 0) 
  {
    int8_t power;
    if (parse_i8_from_cmd(&cmd_upper[11], len - 11, &power)) 
    {
      if (power >= -9 && power <= 22) 
      {
        ESP_LOGI("LunarCore", "Setting TX power to %d dBm", power);
        RadioConfig config = core->radio.config;
        config.tx_power = power;
        if (sx1262_configure(&core->radio, &config) == 0) uart_write_bytes(uart, "OK\r\n", 4);
        else uart_write_bytes(uart, "ERROR\r\n", 7);
      } 
      else uart_write_bytes(uart, "ERROR: Power must be -9 to +22\r\n", 33);
    } 
    else uart_write_bytes(uart, "ERROR: Invalid power\r\n", 22);
  }
  else if (memcmp(cmd_upper, "AT+RESET", 8) == 0) 
  {
    if (sx1262_init(&core->radio) == 0) uart_write_bytes(uart, "OK\r\n", 4);
    else uart_write_bytes(uart, "ERROR\r\n", 7);
  }
  else if (memcmp(cmd_upper, "AT+RX", 5) == 0) 
  {
    if (sx1262_start_rx(&core->radio, 0) == 0) 
    {
      core->rx_active = true;
      uart_write_bytes(uart, "OK\r\n", 4);
    } 
    else uart_write_bytes(uart, "ERROR\r\n", 7);
  }
  else if (memcmp(cmd_upper, "AT+RSSI", 7) == 0) 
  {
    int16_t rssi;
    if (sx1262_get_rssi(&core->radio, &rssi) == 0) 
    {
      uart_write_bytes(uart, "RSSI: ", 6);
      write_i16(uart, rssi);
      uart_write_bytes(uart, " dBm\r\n", 6);
      uart_write_bytes(uart, "OK\r\n", 4);
    } 
    else uart_write_bytes(uart, "ERROR\r\n", 7);
  }
  else if (memcmp(cmd_upper, "AT", 2) == 0 && len == 2) uart_write_bytes(uart, "OK\r\n", 4);
  else if (memcmp(cmd_upper, "AT+HELP", 7) == 0 || memcmp(cmd_upper, "AT?", 3) == 0) 
  {
    uart_write_bytes(uart, "Available commands:\r\n", 21);
    uart_write_bytes(uart, "  AT          - Test\r\n", 22);
    uart_write_bytes(uart, "  ATI         - Version info\r\n", 30);
    uart_write_bytes(uart, "  AT+STATUS   - System status\r\n", 31);
    uart_write_bytes(uart, "  AT+NODEID   - Node ID\r\n", 25);
    uart_write_bytes(uart, "  AT+MAC      - MAC address\r\n", 29);
    uart_write_bytes(uart, "  AT+FREQ=Hz  - Set frequency\r\n", 31);
    uart_write_bytes(uart, "  AT+SF=n     - Set spreading factor\r\n", 38);
    uart_write_bytes(uart, "  AT+TXPOWER=n - Set TX power\r\n", 32);
    uart_write_bytes(uart, "  AT+RX       - Start RX mode\r\n", 31);
    uart_write_bytes(uart, "  AT+RSSI     - Get RSSI\r\n", 26);
    uart_write_bytes(uart, "  AT+RESET    - Reset radio\r\n", 29);
    uart_write_bytes(uart, "OK\r\n", 4);
  }
  else uart_write_bytes(uart, "ERROR: Unknown command\r\n", 24);
}

static void lunar_core_configure_radio_for_protocol(LunarCore *core, Protocol protocol) 
{
  // Configure radio parameters based on detected protocol
  // This would contain protocol-specific configuration
  // Implementation depends on protocol specifics
}

static void lunar_core_handle_meshcore_frame(LunarCore *core, const Frame *frame, uart_port_t uart) 
{
  // Handle MeshCore protocol frame
  // Implementation depends on MeshCore protocol specifics
}

static void lunar_core_handle_meshtastic_frame(LunarCore *core, const MeshtasticFrame *frame, uart_port_t uart) 
{
  // Handle Meshtastic protocol frame
  // Implementation depends on Meshtastic protocol specifics
}

static void lunar_core_handle_rnode_frame(LunarCore *core, const KissFrame *frame, uart_port_t uart) 
{
  // Handle RNode/KISS protocol frame
  // Implementation depends on RNode protocol specifics
}

static void lunar_core_process_serial_byte(LunarCore *core, uint8_t byte, uart_port_t uart) 
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

// Helper function implementations

static void write_hex8(uart_port_t uart, uint8_t value) 
{
  char buf[3];
  sprintf(buf, "%02X", value);
  uart_write_bytes(uart, buf, 2);
}

static void write_hex32(uart_port_t uart, uint32_t value) 
{
  char buf[9];
  sprintf(buf, "%08X", value);
  uart_write_bytes(uart, buf, 8);
}

static void write_u32(uart_port_t uart, uint32_t value) 
{
  char buf[12];
  sprintf(buf, "%u", value);
  uart_write_bytes(uart, buf, strlen(buf));
}

static void write_i16(uart_port_t uart, int16_t value) 
{
  char buf[8];
  sprintf(buf, "%d", value);
  uart_write_bytes(uart, buf, strlen(buf));
}

static const char* format_battery(const BatteryState *battery, char *buf) 
{
  sprintf(buf, "Battery: %dmV (%d%%)", battery->voltage_mv, battery->percentage);
  return buf;
}

static bool parse_u32_from_cmd(const uint8_t *cmd, size_t len, uint32_t *result) 
{
  *result = 0;
  for (size_t i = 0; i < len; i++) 
  {
    if (cmd[i] >= '0' && cmd[i] <= '9') *result = *result * 10 + (cmd[i] - '0');
    else if (cmd[i] == '\r' || cmd[i] == '\n' || cmd[i] == 0) return true;
    else return false;
  }
  return true;
}

static bool parse_i8_from_cmd(const uint8_t *cmd, size_t len, int8_t *result) 
{
  bool negative = false;
  size_t start = 0;
    
  if (len > 0 && cmd[0] == '-') 
  {
    negative = true;
    start = 1;
  }
    
  int32_t value = 0;
  for (size_t i = start; i < len; i++) 
  {
    if (cmd[i] >= '0' && cmd[i] <= '9') value = value * 10 + (cmd[i] - '0');
    else if (cmd[i] == '\r' || cmd[i] == '\n' || cmd[i] == 0) break;
    else return false;
  }
    
  if (negative) value = -value;
  if (value < -128 || value > 127) return false;
    
  *result = (int8_t)value;
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////

// Forward declaration of the external function run_lunarcore
extern void run_lunarcore(void);

/**
 * A null implementation of vprintf that does nothing and returns 0.
 * Matches the signature of vprintf_like_t.
 */
int null_vprintf(const char *_fmt,va_list _args) 
{
  return 0;
}

//Main lunarcore function

void main(void) 
{
  run_lunarcore();
}

/* Placeholder constants based on Rust code context */
static const char *TAG = "LUNAR_CORE";
#define BAUD_RATE 115200
#define ADC_ATTEN ADC_ATTEN_DB_11
#define I2C_MASTER_FREQ_HZ 400000

/* Forward declarations for structures used in the logic */
typedef struct 
{
  uint32_t node_id;
  uint8_t public_key[32];
} node_identity_t;

typedef struct 
{
  uint32_t rx_packets;
  uint32_t tx_packets;
  uint32_t rx_errors;
  int16_t last_rssi;
  uint32_t uptime_seconds;
} lunar_stats_t;

typedef struct 
{
  uint8_t percentage;
} battery_status_t;

typedef enum 
{
  Protocol_Unknown = 0,
  Protocol_MeshCore,
  Protocol_Meshtastic,
  Protocol_RNode,
  Protocol_AtCommand
} protocol_t;

typedef struct 
{
  bool is_on;
  // Implementation details for flashing
} led_controller_t;

typedef struct 
{
    // BLE stack handles
} ble_stack_t;

typedef struct 
{
  // SPI handles and GPIO pins for SX1262
  spi_device_handle_t spi;
  gpio_num_t nss;
  gpio_num_t reset;
  gpio_num_t busy;
  gpio_num_t dio1;
} sx1262_t;

typedef struct 
{
  sx1262_t radio;
  node_identity_t identity;
  lunar_stats_t stats;
  battery_status_t battery;
  led_controller_t led;
  //ble_stack_t ble;
  protocol_t serial_protocol;
  bool repeater_enabled;
  bool rx_active;
  uint32_t last_serial_rx;
  uint32_t prev_ble_connections;
  uint32_t relay_count;
} lunar_core_t;

typedef struct 
{
  // OLED / I2C handle
  i2c_port_t i2c_port;
} status_display_t;

typedef struct 
{
  const char* protocol;
  uint32_t node_id;
  uint8_t battery_pct;
  uint32_t rx_count;
  uint32_t tx_count;
  int16_t rssi;
  bool connected;
  uint16_t irq_status;
  uint16_t last_irq;
  uint32_t dio1_count;
  uint8_t chip_mode;
  uint16_t device_errors;
  bool repeater_active;
  uint32_t relay_count;
} status_content_t;

typedef struct 
{
  uint32_t frequency;
  uint8_t spreading_factor;
  uint8_t bandwidth;
  uint8_t coding_rate;
  int8_t tx_power;
  uint16_t sync_word;
  uint16_t preamble_length;
  bool crc_enabled;
  bool implicit_header;
  bool ldro;
} radio_config_t;

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

/* Function prototypes for external logic referenced in run_lunarcore */
node_identity_t node_identity_from_hardware(void);
bool load_repeater_setting(void);
void init_watchdog(void);
void feed_watchdog(void);
esp_err_t status_display_init(status_display_t* disp);
esp_err_t status_display_boot_animation(status_display_t* disp, void (*delay_fn)(uint32_t));
esp_err_t status_display_show_status(status_display_t* disp, const status_content_t* status);
esp_err_t sx1262_init(sx1262_t* radio);
esp_err_t sx1262_configure(sx1262_t* radio, const radio_config_t* config);
esp_err_t sx1262_start_rx(sx1262_t* radio, uint32_t timeout);
esp_err_t sx1262_transmit(sx1262_t* radio, const uint8_t* data, size_t len);
esp_err_t sx1262_get_irq_status(sx1262_t* radio, uint16_t* irq);
esp_err_t sx1262_clear_irq(sx1262_t* radio, uint16_t irq);
esp_err_t sx1262_read_packet(sx1262_t* radio, uint8_t* buffer, size_t* len, int16_t* rssi, int8_t* snr);
esp_err_t sx1262_get_status(sx1262_t* radio, uint8_t* chip_mode, uint8_t* cmd_status);
uint16_t sx1262_get_errors(sx1262_t* radio);
esp_err_t ble_init(ble_stack_t* ble, const char* name);
esp_err_t ble_start_advertising(ble_stack_t* ble);
uint32_t ble_connection_count(ble_stack_t* ble);
void lunar_core_process_radio_events(lunar_core_t* core, uart_port_t uart);
void lunar_core_process_serial_byte(lunar_core_t* core, uint8_t byte, uart_port_t uart);
void lunar_core_route_rx_packet(lunar_core_t* core, const uint8_t* data, size_t len, int16_t rssi, int8_t snr, uart_port_t uart);
bool lunar_core_maybe_relay_packet(lunar_core_t* core, const uint8_t* data, size_t len, uint32_t now);
bool lunar_core_app_connected(lunar_core_t* core);
bool lunar_core_repeater_active(lunar_core_t* core);
void lunar_core_reset_protocol(lunar_core_t* core);
void lunar_core_update_battery(lunar_core_t* core, uint32_t adc_value);
bool led_update(led_controller_t* led, uint32_t now);
void led_flash(led_controller_t* led, uint32_t count);

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
        // RxError
        if (irq & 0x40) lunarcore.stats.rx_errors += 1;
        // RxDone and no error
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
    {
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
    }
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
        ESP_LOGI(TAG, "Uptime: %lus, RX: %lu, TX: %lu",
        (unsigned long)current_second,
        (unsigned long)lunarcore.stats.rx_packets,
        (unsigned long)lunarcore.stats.tx_packets);
      }
    }
  vTaskDelay(pdMS_TO_TICKS(1));
  }
}
