/* # Conversion Notes: Rust to C Translation

## Overview
This translation converts a complex embedded Rust firmware project for ESP32 into C. The original code implements a multi-protocol mesh network node with support for MeshCore, Meshtastic, and RNode/KISS protocols.

## Major Translation Decisions

### 1. Module System
- **Rust**: Uses `mod` declarations and separate files
- **C**: Converted to `#include` directives for header files
- **Note**: Each module (crypto, sx1262, protocol, etc.) would need separate .h/.c file pairs in C

### 2. Traits and Generic Types
- **Rust**: Uses embedded-hal traits with generic type parameters (SPI, NSS, RESET, BUSY, DIO1)
- **C**: Removed generic constraints, used concrete types from ESP-IDF
- **Implementation**: Function pointers or interface structs would be needed for full trait behavior

### 3. Error Handling
- **Rust**: Uses `Result<T, E>` and `Option<T>`
- **C**: Converted to return codes (0 for success, non-zero for errors) and NULL pointers
- **Pattern**: `if let Ok(result) = func()` becomes `if (func(&result) == 0)`

### 4. Memory Safety
- **Rust**: Ownership, borrowing, and lifetimes enforce memory safety at compile time
- **C**: Manual memory management required
- **Note**: Caller must ensure proper lifetime management and avoid use-after-free

### 5. Atomic Operations
- **Rust**: `AtomicBool` and `AtomicU32` with `Ordering` semantics
- **C**: C11 `<stdatomic.h>` types: `atomic_bool`, `atomic_uint_least32_t`
- **Functions**: `atomic_store()`, `atomic_load()`, `atomic_exchange()`, `atomic_fetch_add()`

### 6. Static Variables
- **Rust**: `static` with `AtomicBool::new(false)`
- **C**: Static global variables with `ATOMIC_VAR_INIT()` initializer

### 7. Strings and String Formatting
- **Rust**: `&str` type and format macros
- **C**: `const char*` and `sprintf()`/`snprintf()` for formatting
- **Buffer Management**: Fixed-size buffers used instead of heap allocation

### 8. Enums
- **Rust**: Rich enums with associated data (e.g., `DecryptResult`)
- **C**: Split into enum + union struct pattern for data-carrying enums

### 9. Arrays and Collections
- **Rust**: Fixed-size arrays `[u8; N]` and `heapless::Vec<T, N>`
- **C**: Plain arrays with separate length tracking

### 10. Method Syntax
- **Rust**: `object.method(args)`
- **C**: `type_method(&object, args)` or `type_method(object, args)`

### 11. Constructors
- **Rust**: `new()` associated functions
- **C**: `type_new()` functions that initialize a struct pointer

### 12. Constants
- **Rust**: `const` items in code
- **C**: `#define` macros or `const` variables
- **Choice**: Used `#define` for configuration constants, `const` for lookup tables

### 13. Unsafe Code
- **Rust**: Explicit `unsafe {}` blocks for FFI and raw pointer access
- **C**: All code is effectively "unsafe" - no special marking needed
- **eFuse Access**: Direct memory-mapped register reads preserved

### 14. NVS (Non-Volatile Storage)
- **Rust**: FFI calls to ESP-IDF functions through `esp_idf_sys`
- **C**: Direct ESP-IDF API calls
- **Error Handling**: Checked `esp_err_t` return values

### 15. Logging
- **Rust**: `log::info!()`, `log::warn!()` macros
- **C**: `ESP_LOGI()`, `ESP_LOGW()` macros from `esp_log.h`

### 16. Hardware Abstraction
- **Rust**: embedded-hal traits abstract hardware interfaces
- **C**: Direct ESP-IDF driver API calls (uart, spi, gpio, adc, etc.)

### 17. Interrupt Handling
- **Rust**: Atomic flags set in interrupt context
- **C**: Same pattern using C11 atomics

## Incomplete Translation Areas

### The code was cut off at the end
The original Rust code ends mid-statement: `self.handle_rnode_frame(&fram`
This is reflected in the C translation which ends at the same logical point.

### Module Dependencies
The following modules are referenced but not provided in the original code:
- `crypto` (sha256, ed25519, x25519)
- `rng`
- `sx1262` (radio driver)
- `protocol` (frame parsing)
- `protocol_router`
- `meshtastic`
- `rnode`
- `ble`
- `display`
- `transport`
- `session`
- `onion`

Each would require a separate C implementation with appropriate headers.

### Function Implementations
Several functions are declared but have minimal/placeholder implementations:
- `lunar_core_configure_radio_for_protocol()` - needs protocol-specific logic
- `lunar_core_handle_meshcore_frame()` - needs MeshCore protocol handler
- `lunar_core_handle_meshtastic_frame()` - needs Meshtastic protocol handler
- `lunar_core_handle_rnode_frame()` - needs RNode protocol handler

These would be fully implemented once the dependent modules are translated.

## Type System Differences

### Rust's Ownership System
The original Rust code uses ownership and borrowing to prevent:
- Use-after-free
- Double-free
- Data races

In C, the programmer must manually ensure:
- Pointers remain valid for their entire use
- Memory is freed exactly once
- Concurrent access is properly synchronized

### Size Types
- **Rust**: `usize` (platform-dependent)
- **C**: `size_t` (platform-dependent)

### Integer Types
- **Rust**: Explicit sized types (`u8`, `u32`, `i16`, etc.)
- **C**: `<stdint.h>` types (`uint8_t`, `uint32_t`, `int16_t`, etc.)

## Platform-Specific Notes

### ESP-IDF Integration
- Uses FreeRTOS for task management
- ESP-IDF drivers for peripherals
- NVS for persistent storage
- Hardware random number generation

### Memory Constraints
- Embedded environment with limited RAM
- Stack-allocated buffers preferred over heap
- Fixed-size arrays for predictable memory usage

### Interrupt Safety
- Atomic operations ensure ISR-safe flag updates
- Minimal work in interrupt handlers
- Event flags processed in main loop

## Testing Recommendations

1. Verify all atomic operations maintain proper ordering
2. Test protocol detection with all supported formats
3. Validate battery voltage calculations
4. Check NVS persistence across reboots
5. Test error paths (CRC errors, buffer overflows, etc.)
6. Verify radio configuration changes take effect
7. Test AT command parsing edge cases

## Future Improvements

1. Add comprehensive error codes instead of simple 0/non-zero
2. Implement proper buffer overflow protection
3. Add bounds checking on all array accesses
4. Consider using FreeRTOS queues for inter-task communication
5. Add watchdog timer implementation
6. Implement complete protocol handlers
7. Add unit tests for critical functions
 */

// Required ESP-IDF and standard library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_efuse.h"
#include "esp_random.h"

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

// Pin definitions
#define PIN_SPI_MOSI 10
#define PIN_SPI_MISO 11
#define PIN_SPI_SCK 9
#define PIN_LORA_NSS 8
#define PIN_LORA_RST 12
#define PIN_LORA_BUSY 13
#define PIN_LORA_DIO1 14
#define PIN_LED 35
#define PIN_VEXT 36
#define PIN_BATTERY_ADC 1
#define PIN_I2C_SDA 17
#define PIN_I2C_SCL 18
#define PIN_OLED_RST 21

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
typedef enum {
    CRYPTO_ERROR_SESSION_ERROR,
    CRYPTO_ERROR_ENCRYPTION_FAILED,
    CRYPTO_ERROR_DECRYPTION_FAILED,
    CRYPTO_ERROR_NO_SESSION,
    CRYPTO_ERROR_ONION_ERROR,
    CRYPTO_ERROR_INVALID_FORMAT,
    CRYPTO_ERROR_BUFFER_OVERFLOW
} CryptoError;

// DecryptResult enumeration and structure
typedef enum {
    DECRYPT_RESULT_PLAINTEXT,
    DECRYPT_RESULT_FORWARD
} DecryptResultType;

typedef struct {
    DecryptResultType type;
    union {
        struct {
            uint8_t data[MAX_HEAP_VEC_SIZE];
            size_t len;
        } plaintext;
        struct {
            uint16_t next_hop;
            uint8_t data[MAX_ONION_DATA_SIZE];
            size_t len;
        } forward;
    };
} DecryptResult;

// NodeIdentity structure
typedef struct {
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
typedef struct {
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
typedef struct {
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
typedef struct {
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
typedef struct {
    Sx1262 radio;
    ProtocolRouter router;
    FrameParser meshcore_parser;
    MeshtasticHandler meshtastic;
    RNodeHandler rnode;
    BleManager ble;
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

static void node_identity_read_mac_address(uint8_t mac[6]) {
    // Read MAC address from ESP32 eFuse
    esp_efuse_mac_get_default(mac);
}

static void node_identity_read_hardware_serial(uint8_t serial[8]) {
    // Read hardware serial from eFuse registers
    volatile uint32_t *efuse_base = (volatile uint32_t *)0x6001A044;
    uint32_t word0 = *efuse_base;
    uint32_t word1 = *(efuse_base + 1);
    
    memcpy(&serial[0], &word0, 4);
    memcpy(&serial[4], &word1, 4);
}

static uint32_t node_identity_generate_random_node_id(void) {
    uint8_t random_bytes[4];
    esp_fill_random(random_bytes, 4);
    
    uint32_t id;
    memcpy(&id, random_bytes, 4);
    // Set high bit to indicate random ID
    id |= 0x80000000;
    return id;
}

static void node_identity_generate_random_private_key(const uint8_t hardware_serial[8], 
                                                       uint8_t private_key[32]) {
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

static void node_identity_load_or_create_identity(const uint8_t hardware_serial[8], 
                                                   uint32_t *node_id, uint8_t private_key[32]) {
    nvs_handle_t handle;
    esp_err_t err;
    
    // Try to open NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        // Initialize NVS flash and retry
        nvs_flash_init();
        err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
        if (err != ESP_OK) {
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
    bool has_private_key = (nvs_get_blob(handle, NVS_KEY_PRIVATE_KEY, private_key, &key_len) == ESP_OK) 
                           && (key_len == 32);
    
    if (has_node_id && has_private_key) {
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

static NodeIdentity node_identity_from_hardware(void) {
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

static NodeIdentity* node_identity_factory_reset(void) {
    // Erase NVS partition
    nvs_flash_erase_partition(NVS_NAMESPACE);
    
    ESP_LOGI("NodeIdentity", "Factory reset: erased old identity, generating new one");
    
    // Create new identity
    NodeIdentity *identity = malloc(sizeof(NodeIdentity));
    if (identity) {
        *identity = node_identity_from_hardware();
    }
    return identity;
}

static void node_identity_x25519_pubkey(const NodeIdentity *self, uint8_t pubkey[32]) {
    x25519_base(self->private_key, pubkey);
}

// Implementation: BatteryState functions

static void battery_state_new(BatteryState *battery) {
    battery->voltage_mv = 0;
    battery->percentage = 0;
    battery->is_charging = false;
    battery->is_low = false;
    battery->is_critical = false;
}

static uint8_t battery_state_voltage_to_percentage(uint32_t mv) {
    // Voltage to percentage lookup curve
    typedef struct {
        uint32_t voltage;
        uint8_t percentage;
    } CurvePoint;
    
    static const CurvePoint curve[11] = {
        {4200, 100}, {4100, 90}, {4000, 80}, {3900, 70}, {3800, 60},
        {3700, 50}, {3600, 40}, {3500, 30}, {3400, 20}, {3300, 10}, {3000, 0}
    };
    
    if (mv >= curve[0].voltage) return 100;
    if (mv <= curve[10].voltage) return 0;
    
    // Linear interpolation between curve points
    for (int i = 0; i < 10; i++) {
        if (mv >= curve[i + 1].voltage && mv <= curve[i].voltage) {
            uint32_t v_range = curve[i].voltage - curve[i + 1].voltage;
            uint8_t p_range = curve[i].percentage - curve[i + 1].percentage;
            uint32_t v_offset = mv - curve[i + 1].voltage;
            
            return curve[i + 1].percentage + ((v_offset * p_range) / v_range);
        }
    }
    
    return 50;
}

static void battery_state_update(BatteryState *battery, uint32_t adc_value) {
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

static void stats_new(Stats *stats) {
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

static void stats_record_rx(Stats *stats, size_t len, int16_t rssi, int8_t snr) {
    stats->rx_packets++;
    stats->rx_bytes += len;
    stats->last_rssi = rssi;
    stats->last_snr = snr;
}

static void stats_record_tx(Stats *stats, size_t len, uint32_t airtime_ms) {
    stats->tx_packets++;
    stats->tx_bytes += len;
    stats->airtime_ms += airtime_ms;
}

// Implementation: LedController functions

static void led_controller_new(LedController *led) {
    led->is_on = false;
    led->last_toggle = 0;
    led->interval_ms = LED_BLINK_IDLE;
    led->blink_count = 0;
    led->remaining = 0;
}

static void led_controller_set_idle(LedController *led) {
    led->interval_ms = LED_BLINK_IDLE;
    led->blink_count = 0;
}

static void led_controller_set_active(LedController *led) {
    led->interval_ms = LED_BLINK_ACTIVE;
    led->blink_count = 0;
}

static void led_controller_set_error(LedController *led) {
    led->interval_ms = LED_BLINK_ERROR;
    led->blink_count = 0;
}

static void led_controller_flash(LedController *led, uint8_t count) {
    led->blink_count = count;
    led->remaining = count * 2;
    led->interval_ms = 100;
}

static bool led_controller_update(LedController *led, uint32_t current_time) {
    uint32_t elapsed = current_time - led->last_toggle;
    if (elapsed >= led->interval_ms) {
        led->last_toggle = current_time;
        
        if (led->blink_count > 0) {
            if (led->remaining > 0) {
                led->remaining--;
                led->is_on = !led->is_on;
                return true;
            } else {
                // Blink sequence complete, return to idle
                led_controller_set_idle(led);
            }
        } else {
            led->is_on = !led->is_on;
            return true;
        }
    }
    return false;
}

// Implementation: LunarCore functions

static void lunar_core_new(LunarCore *core, Sx1262 *radio, const NodeIdentity *identity) {
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

static const NodeIdentity* lunar_core_identity(const LunarCore *core) {
    return &core->identity;
}

static void lunar_core_update_battery(LunarCore *core, uint32_t adc_value) {
    battery_state_update(&core->battery, adc_value);
    
    if (core->battery.is_critical) {
        led_controller_set_error(&core->led);
        ESP_LOGW("LunarCore", "Battery critical: %dmV", core->battery.voltage_mv);
    } else if (core->battery.is_low) {
        ESP_LOGI("LunarCore", "Battery low: %dmV (%d%%)", 
                 core->battery.voltage_mv, core->battery.percentage);
    }
}

static void lunar_core_handle_dio1_interrupt(LunarCore *core) {
    // Clear interrupt flag
    atomic_store(&dio1_triggered, false);
    
    // Read interrupt status from radio
    uint16_t irq_status;
    if (sx1262_get_irq_status(&core->radio, &irq_status) == 0) {
        // TX complete
        if (irq_status & 0x01) {
            atomic_store(&tx_complete, true);
            atomic_store(&last_activity, atomic_load(&system_ticks));
        }
        
        // RX done
        if (irq_status & 0x02) {
            atomic_store(&packet_pending, true);
            atomic_store(&last_activity, atomic_load(&system_ticks));
        }
        
        // Preamble detected (0x04) - no action needed
        
        // CRC error
        if (irq_status & 0x40) {
            core->stats.rx_errors++;
            atomic_fetch_add(&error_count, 1);
        }
        
        // Clear interrupt
        sx1262_clear_irq(&core->radio, irq_status);
    }
}

static void lunar_core_process_radio_events(LunarCore *core) {
    // Handle TX complete
    if (atomic_exchange(&tx_complete, false)) {
        // Restart RX mode
        if (sx1262_start_rx(&core->radio, 0) == 0) {
            core->rx_active = true;
        }
        led_controller_flash(&core->led, 1);
    }
    
    // Handle RX packet
    if (atomic_exchange(&packet_pending, false)) {
        led_controller_flash(&core->led, 2);
    }
}

static void lunar_core_process_at_command(LunarCore *core, uart_port_t uart) {
    // Convert command to uppercase
    uint8_t cmd_upper[MAX_AT_BUFFER_SIZE];
    size_t len = core->at_buffer_len < MAX_AT_BUFFER_SIZE ? core->at_buffer_len : MAX_AT_BUFFER_SIZE;
    for (size_t i = 0; i < len; i++) {
        cmd_upper[i] = toupper(core->at_buffer[i]);
    }
    
    // Send newline
    uart_write_bytes(uart, "\r\n", 2);
    
    // Process command
    if (memcmp(cmd_upper, "AT+VERSION", 10) == 0 || memcmp(cmd_upper, "ATI", 3) == 0) {
        uart_write_bytes(uart, "LunarCore v1.0.0\r\n", 18);
        uart_write_bytes(uart, "Unified Mesh Bridge Firmware\r\n", 30);
        uart_write_bytes(uart, "Protocols: MeshCore, Meshtastic, RNode/KISS\r\n", 46);
        uart_write_bytes(uart, "OK\r\n", 4);
    }
    else if (memcmp(cmd_upper, "AT+STATUS", 9) == 0) {
        uart_write_bytes(uart, "Status: ", 8);
        if (core->rx_active) {
            uart_write_bytes(uart, "RX Active\r\n", 11);
        } else {
            uart_write_bytes(uart, "Idle\r\n", 6);
        }
        
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
    else if (memcmp(cmd_upper, "AT+NODEID", 9) == 0) {
        uart_write_bytes(uart, "Node ID: ", 9);
        write_hex32(uart, core->identity.node_id);
        uart_write_bytes(uart, "\r\n", 2);
        uart_write_bytes(uart, "OK\r\n", 4);
    }
    else if (memcmp(cmd_upper, "AT+MAC", 6) == 0) {
        uart_write_bytes(uart, "MAC: ", 5);
        for (int i = 0; i < 6; i++) {
            write_hex8(uart, core->identity.mac_address[i]);
            if (i < 5) {
                uart_write_bytes(uart, ":", 1);
            }
        }
        uart_write_bytes(uart, "\r\n", 2);
        uart_write_bytes(uart, "OK\r\n", 4);
    }
    else if (memcmp(cmd_upper, "AT+FREQ=", 8) == 0) {
        uint32_t freq;
        if (parse_u32_from_cmd(&cmd_upper[8], len - 8, &freq)) {
            ESP_LOGI("LunarCore", "Setting frequency to %d Hz", freq);
            RadioConfig config = core->radio.config;
            config.frequency = freq;
            if (sx1262_configure(&core->radio, &config) == 0) {
                uart_write_bytes(uart, "OK\r\n", 4);
            } else {
                uart_write_bytes(uart, "ERROR\r\n", 7);
            }
        } else {
            uart_write_bytes(uart, "ERROR: Invalid frequency\r\n", 26);
        }
    }
    else if (memcmp(cmd_upper, "AT+SF=", 6) == 0) {
        uint32_t sf;
        if (parse_u32_from_cmd(&cmd_upper[6], len - 6, &sf)) {
            if (sf >= 7 && sf <= 12) {
                ESP_LOGI("LunarCore", "Setting SF to %d", sf);
                RadioConfig config = core->radio.config;
                config.spreading_factor = (uint8_t)sf;
                if (sx1262_configure(&core->radio, &config) == 0) {
                    uart_write_bytes(uart, "OK\r\n", 4);
                } else {
                    uart_write_bytes(uart, "ERROR\r\n", 7);
                }
            } else {
                uart_write_bytes(uart, "ERROR: SF must be 7-12\r\n", 24);
            }
        } else {
            uart_write_bytes(uart, "ERROR: Invalid SF\r\n", 19);
        }
    }
    else if (memcmp(cmd_upper, "AT+TXPOWER=", 11) == 0) {
        int8_t power;
        if (parse_i8_from_cmd(&cmd_upper[11], len - 11, &power)) {
            if (power >= -9 && power <= 22) {
                ESP_LOGI("LunarCore", "Setting TX power to %d dBm", power);
                RadioConfig config = core->radio.config;
                config.tx_power = power;
                if (sx1262_configure(&core->radio, &config) == 0) {
                    uart_write_bytes(uart, "OK\r\n", 4);
                } else {
                    uart_write_bytes(uart, "ERROR\r\n", 7);
                }
            } else {
                uart_write_bytes(uart, "ERROR: Power must be -9 to +22\r\n", 33);
            }
        } else {
            uart_write_bytes(uart, "ERROR: Invalid power\r\n", 22);
        }
    }
    else if (memcmp(cmd_upper, "AT+RESET", 8) == 0) {
        if (sx1262_init(&core->radio) == 0) {
            uart_write_bytes(uart, "OK\r\n", 4);
        } else {
            uart_write_bytes(uart, "ERROR\r\n", 7);
        }
    }
    else if (memcmp(cmd_upper, "AT+RX", 5) == 0) {
        if (sx1262_start_rx(&core->radio, 0) == 0) {
            core->rx_active = true;
            uart_write_bytes(uart, "OK\r\n", 4);
        } else {
            uart_write_bytes(uart, "ERROR\r\n", 7);
        }
    }
    else if (memcmp(cmd_upper, "AT+RSSI", 7) == 0) {
        int16_t rssi;
        if (sx1262_get_rssi(&core->radio, &rssi) == 0) {
            uart_write_bytes(uart, "RSSI: ", 6);
            write_i16(uart, rssi);
            uart_write_bytes(uart, " dBm\r\n", 6);
            uart_write_bytes(uart, "OK\r\n", 4);
        } else {
            uart_write_bytes(uart, "ERROR\r\n", 7);
        }
    }
    else if (memcmp(cmd_upper, "AT", 2) == 0 && len == 2) {
        uart_write_bytes(uart, "OK\r\n", 4);
    }
    else if (memcmp(cmd_upper, "AT+HELP", 7) == 0 || memcmp(cmd_upper, "AT?", 3) == 0) {
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
    else {
        uart_write_bytes(uart, "ERROR: Unknown command\r\n", 24);
    }
}

static void lunar_core_configure_radio_for_protocol(LunarCore *core, Protocol protocol) {
    // Configure radio parameters based on detected protocol
    // This would contain protocol-specific configuration
    // Implementation depends on protocol specifics
}

static void lunar_core_handle_meshcore_frame(LunarCore *core, const Frame *frame, uart_port_t uart) {
    // Handle MeshCore protocol frame
    // Implementation depends on MeshCore protocol specifics
}

static void lunar_core_handle_meshtastic_frame(LunarCore *core, const MeshtasticFrame *frame, uart_port_t uart) {
    // Handle Meshtastic protocol frame
    // Implementation depends on Meshtastic protocol specifics
}

static void lunar_core_handle_rnode_frame(LunarCore *core, const KissFrame *frame, uart_port_t uart) {
    // Handle RNode/KISS protocol frame
    // Implementation depends on RNode protocol specifics
}

static void lunar_core_process_serial_byte(LunarCore *core, uint8_t byte, uart_port_t uart) {
    // Protocol detection
    if (core->serial_protocol == PROTOCOL_UNKNOWN) {
        TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
        Protocol detected = protocol_detector_feed(&transport->detector, byte);
        if (detected != PROTOCOL_UNKNOWN) {
            core->serial_protocol = detected;
            core->stats.protocol_switches++;
            ESP_LOGI("LunarCore", "Protocol detected: %s", protocol_name(detected));
            
            lunar_core_configure_radio_for_protocol(core, detected);
        }
    }
    
    // Process byte according to detected protocol
    switch (core->serial_protocol) {
        case PROTOCOL_MESHCORE: {
            Frame *frame = frame_parser_feed(&core->meshcore_parser, byte);
            if (frame != NULL) {
                lunar_core_handle_meshcore_frame(core, frame, uart);
                TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
                protocol_detector_confirm_frame(&transport->detector);
            }
            break;
        }
        
        case PROTOCOL_MESHTASTIC: {
            MeshtasticFrame *frame = meshtastic_handler_feed_serial(&core->meshtastic, byte);
            if (frame != NULL) {
                lunar_core_handle_meshtastic_frame(core, frame, uart);
                TransportHandler *transport = protocol_router_get_transport(&core->router, TRANSPORT_USB_SERIAL);
                protocol_detector_confirm_frame(&transport->detector);
            }
            break;
        }
        
        case PROTOCOL_RNODE: {
            KissFrame *frame = rnode_handler_feed_serial(&core->rnode, byte);
            if (frame != NULL) {
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

static void write_hex8(uart_port_t uart, uint8_t value) {
    char buf[3];
    sprintf(buf, "%02X", value);
    uart_write_bytes(uart, buf, 2);
}

static void write_hex32(uart_port_t uart, uint32_t value) {
    char buf[9];
    sprintf(buf, "%08X", value);
    uart_write_bytes(uart, buf, 8);
}

static void write_u32(uart_port_t uart, uint32_t value) {
    char buf[12];
    sprintf(buf, "%u", value);
    uart_write_bytes(uart, buf, strlen(buf));
}

static void write_i16(uart_port_t uart, int16_t value) {
    char buf[8];
    sprintf(buf, "%d", value);
    uart_write_bytes(uart, buf, strlen(buf));
}

static const char* format_battery(const BatteryState *battery, char *buf) {
    sprintf(buf, "Battery: %dmV (%d%%)", battery->voltage_mv, battery->percentage);
    return buf;
}

static bool parse_u32_from_cmd(const uint8_t *cmd, size_t len, uint32_t *result) {
    *result = 0;
    for (size_t i = 0; i < len; i++) {
        if (cmd[i] >= '0' && cmd[i] <= '9') {
            *result = *result * 10 + (cmd[i] - '0');
        } else if (cmd[i] == '\r' || cmd[i] == '\n' || cmd[i] == 0) {
            return true;
        } else {
            return false;
        }
    }
    return true;
}

static bool parse_i8_from_cmd(const uint8_t *cmd, size_t len, int8_t *result) {
    bool negative = false;
    size_t start = 0;
    
    if (len > 0 && cmd[0] == '-') {
        negative = true;
        start = 1;
    }
    
    int32_t value = 0;
    for (size_t i = start; i < len; i++) {
        if (cmd[i] >= '0' && cmd[i] <= '9') {
            value = value * 10 + (cmd[i] - '0');
        } else if (cmd[i] == '\r' || cmd[i] == '\n' || cmd[i] == 0) {
            break;
        } else {
            return false;
        }
    }
    
    if (negative) value = -value;
    if (value < -128 || value > 127) return false;
    
    *result = (int8_t)value;
    return true;
}
