#ifndef _MAIN_H_
#define _MAIN_H_

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

// BatteryState structure
typedef struct 
{
  uint32_t voltage_mv;
  uint8_t percentage;
  bool is_charging;
  bool is_low;
  bool is_critical;
} BatteryState;

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

// LedController structure
typedef struct 
{
  bool is_on;
  uint32_t last_toggle;
  uint32_t interval_ms;
  uint8_t blink_count;
  uint8_t remaining;
} LedController;

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

#endif
