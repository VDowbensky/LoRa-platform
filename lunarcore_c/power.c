/* # Conversion Notes: Rust to C Translation

## Language-Specific Feature Adaptations

### 1. Option<T> Type
- Rust's `Option<GpioWakeConfig>` was translated to a C struct `OptionGpioWakeConfig` containing:
  - `bool is_some`: Indicates if the option contains a value
  - `GpioWakeConfig value`: The actual value when present
- Helper functions `option_gpio_wake_config_none()` and `option_gpio_wake_config_some()` provide clean initialization

### 2. Enums with Associated Data
- Rust's `WakeCause` enum with `Gpio(u8)` variant was translated using a tagged union pattern:
  - `WakeCauseType` enum for discriminant
  - Union containing possible data fields
  - Constructor functions for each variant (e.g., `wake_cause_gpio(uint8_t pin)`)

### 3. Result<T, E> Error Handling
- Rust's `Result<(), PowerError>` returns were translated to:
  - Functions return `PowerError` (0 for success, error code otherwise)
  - Output parameters passed by pointer where needed (e.g., `WakeCause* out_cause`)
- `Result<WakeCause, PowerError>` became a function returning error with output parameter

### 4. Const Functions and Static Methods
- Rust's `const fn` functions were translated to `static inline` functions for compile-time efficiency
- Static methods were translated to regular functions with appropriate naming conventions

### 5. Trait Implementations
- `Default` trait → `_default()` and `_new()` constructor functions
- `BitOr` trait → `wake_sources_bitor()` function
- Methods are translated to functions taking pointer to struct as first parameter

### 6. Never Return Type (!)
- Rust's `!` return type in `deep_sleep` translates to `void` in C
- Added infinite loop at the end as safety measure (though `esp_deep_sleep_start()` should never return)

## Type System Differences

### 1. Integer Types
- Rust's `u64`, `u32`, `u8` → C's `uint64_t`, `uint32_t`, `uint8_t`
- Maintained exact bit widths using `stdint.h` types

### 2. Boolean Type
- Rust's `bool` → C's `bool` from `stdbool.h`

### 3. Struct Initialization
- Rust's struct literal syntax → C's designated initializers or manual field assignment

### 4. Arrays
- Rust's `[Option<GpioWakeConfig>; 8]` → C's `OptionGpioWakeConfig[8]`
- Manual initialization in loop required for Option array

## Standard Library Equivalents

### 1. Bitwise Operations
- Direct translation: `|`, `&`, `<<` operators work identically
- `trailing_zeros()` → `__builtin_ctzll()` (count trailing zeros) for uint64_t

### 2. Memory Operations
- `core::ptr::write_volatile()` → Direct dereference of `volatile` pointer
- `core::ptr::read_volatile()` → Direct read from `volatile` pointer

### 3. Hints
- `core::hint::spin_loop()` → `__asm__ __volatile__("nop")` or simple loop

## ESP-IDF FFI Calls

All `esp_idf_sys::` calls were translated to direct function calls:
- Removed `unsafe` blocks (not needed in C)
- Used ESP-IDF types directly (e.g., `esp_pm_config_esp32s3_t`, `esp_sleep_wakeup_cause_t`)
- Cast handling: Rust's explicit casts → C's implicit/explicit casts where needed

## Access Modifiers

- Rust's `pub` → C functions without `static` (external linkage)
- Private methods → `static` functions (internal linkage)
- `pub const` → `#define` or `static const` depending on usage

## Implementation Notes

### 1. WakeSources Bitflags
- Implemented as struct with `uint32_t value` field
- Constants defined as `static const` for type safety
- Methods implemented as inline functions for efficiency

### 2. Array Initialization
- `[None; 8]` pattern translated to loop-based initialization
- Ensures all elements are properly set to "none" state

### 3. RTC Memory Access
- Hardcoded memory address (0x50000000) preserved
- Used `volatile` qualifier to prevent compiler optimizations

### 4. Power Management Configuration
- ESP32-S3 specific structure used (`esp_pm_config_esp32s3_t`)
- Error checking preserved from original code

### 5. Function Naming Convention
- Rust methods → C functions prefixed with struct name (e.g., `power_manager_init`)
- Internal methods → `static` functions with same naming pattern
- Maintains clear namespace separation

## Potential Issues and Limitations

### 1. Thread Safety
- Original Rust code doesn't use locks; C translation maintains same behavior
- Users must ensure proper synchronization if used in multi-threaded context

### 2. Memory Layout
- Struct field ordering preserved but may differ in padding/alignment
- Consider using `__attribute__((packed))` if exact layout is critical

### 3. ESP-IDF Version Compatibility
- Code assumes ESP-IDF v4.x or v5.x API
- Some constants/functions may differ in older versions

### 4. Constructor Pattern
- C doesn't have constructors; users must explicitly call `power_manager_new()` or `power_manager_default()`
- No RAII cleanup (users must manage resources manually)

### 5. Enum Exhaustiveness
- C doesn't enforce exhaustive pattern matching
- Added `default` cases in switches for robustness

### 6. Integer Overflow
- Rust checks for overflow in debug mode; C doesn't
- Consider adding explicit overflow checks if critical

## Testing Recommendations

1. Verify all wake sources function correctly
2. Test boundary conditions (min/max sleep durations)
3. Validate GPIO wake configuration with all 8 slots
4. Test RTC memory persistence across deep sleep
5. Verify CPU frequency changes take effect
6. Test all power modes and their transitions
7. Validate wake cause detection for all supported sources
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <esp_pm.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

// Constants
#define MIN_DEEP_SLEEP_US 1000ULL
#define MAX_DEEP_SLEEP_US 86400000000ULL
#define DEFAULT_LIGHT_SLEEP_MS 100U
#define LOW_BATTERY_THRESHOLD_MV 3400U
#define CRITICAL_BATTERY_THRESHOLD_MV 3200U

// PowerMode enum
typedef enum {
    POWER_MODE_PERFORMANCE,
    POWER_MODE_BALANCED,
    POWER_MODE_LOW_POWER,
    POWER_MODE_ULTRA_LOW
} PowerMode;

// PowerMode default
static inline PowerMode power_mode_default(void) {
    return POWER_MODE_BALANCED;
}

// WakeSources struct
typedef struct {
    uint32_t value;
} WakeSources;

// WakeSources constants
static const WakeSources WAKE_SOURCES_NONE = { 0 };
static const WakeSources WAKE_SOURCES_TIMER = { 1 << 0 };
static const WakeSources WAKE_SOURCES_GPIO = { 1 << 1 };
static const WakeSources WAKE_SOURCES_UART = { 1 << 2 };
static const WakeSources WAKE_SOURCES_TOUCH = { 1 << 3 };
static const WakeSources WAKE_SOURCES_ULP = { 1 << 4 };
static const WakeSources WAKE_SOURCES_BLE = { 1 << 5 };
static const WakeSources WAKE_SOURCES_WIFI = { 1 << 6 };
static const WakeSources WAKE_SOURCES_EXT0 = { 1 << 7 };
static const WakeSources WAKE_SOURCES_EXT1 = { 1 << 8 };
static const WakeSources WAKE_SOURCES_ALL = { 0x1FF };

// WakeSources methods
static inline WakeSources wake_sources_or(WakeSources self, WakeSources other) {
    WakeSources result = { self.value | other.value };
    return result;
}

static inline bool wake_sources_has(WakeSources self, WakeSources source) {
    return (self.value & source.value) != 0;
}

static inline WakeSources wake_sources_bitor(WakeSources self, WakeSources rhs) {
    WakeSources result = { self.value | rhs.value };
    return result;
}

// WakeCause enum with tagged union
typedef enum {
    WAKE_CAUSE_POWER_ON,
    WAKE_CAUSE_TIMER,
    WAKE_CAUSE_GPIO,
    WAKE_CAUSE_UART,
    WAKE_CAUSE_TOUCH,
    WAKE_CAUSE_ULP,
    WAKE_CAUSE_BLE,
    WAKE_CAUSE_WIFI,
    WAKE_CAUSE_EXT0,
    WAKE_CAUSE_EXT1,
    WAKE_CAUSE_UNKNOWN
} WakeCauseType;

typedef struct {
    WakeCauseType type;
    union {
        uint8_t gpio_pin;
    } data;
} WakeCause;

// WakeCause constructors
static inline WakeCause wake_cause_power_on(void) {
    WakeCause cause = { WAKE_CAUSE_POWER_ON, {0} };
    return cause;
}

static inline WakeCause wake_cause_timer(void) {
    WakeCause cause = { WAKE_CAUSE_TIMER, {0} };
    return cause;
}

static inline WakeCause wake_cause_gpio(uint8_t pin) {
    WakeCause cause = { WAKE_CAUSE_GPIO, {0} };
    cause.data.gpio_pin = pin;
    return cause;
}

static inline WakeCause wake_cause_uart(void) {
    WakeCause cause = { WAKE_CAUSE_UART, {0} };
    return cause;
}

static inline WakeCause wake_cause_touch(void) {
    WakeCause cause = { WAKE_CAUSE_TOUCH, {0} };
    return cause;
}

static inline WakeCause wake_cause_ulp(void) {
    WakeCause cause = { WAKE_CAUSE_ULP, {0} };
    return cause;
}

static inline WakeCause wake_cause_ble(void) {
    WakeCause cause = { WAKE_CAUSE_BLE, {0} };
    return cause;
}

static inline WakeCause wake_cause_wifi(void) {
    WakeCause cause = { WAKE_CAUSE_WIFI, {0} };
    return cause;
}

static inline WakeCause wake_cause_ext0(void) {
    WakeCause cause = { WAKE_CAUSE_EXT0, {0} };
    return cause;
}

static inline WakeCause wake_cause_ext1(void) {
    WakeCause cause = { WAKE_CAUSE_EXT1, {0} };
    return cause;
}

static inline WakeCause wake_cause_unknown(void) {
    WakeCause cause = { WAKE_CAUSE_UNKNOWN, {0} };
    return cause;
}

// GpioWakeConfig struct
typedef struct {
    uint8_t pin;
    bool level_high;
} GpioWakeConfig;

// Option<GpioWakeConfig> implementation
typedef struct {
    bool is_some;
    GpioWakeConfig value;
} OptionGpioWakeConfig;

static inline OptionGpioWakeConfig option_gpio_wake_config_none(void) {
    OptionGpioWakeConfig opt = { false, {0, false} };
    return opt;
}

static inline OptionGpioWakeConfig option_gpio_wake_config_some(GpioWakeConfig value) {
    OptionGpioWakeConfig opt = { true, value };
    return opt;
}

// PowerError enum
typedef enum {
    POWER_ERROR_OK = 0,
    POWER_ERROR_INVALID_FREQUENCY,
    POWER_ERROR_CONFIG_FAILED,
    POWER_ERROR_INVALID_DURATION,
    POWER_ERROR_SLEEP_FAILED,
    POWER_ERROR_TOO_MANY_WAKE_PINS
} PowerError;

// PowerManager struct
typedef struct {
    PowerMode mode;
    WakeSources light_sleep_wake;
    WakeSources deep_sleep_wake;
    OptionGpioWakeConfig gpio_wake_pins[8];
    uint32_t cpu_freq_mhz;
    WakeCause last_wake_cause;
    uint64_t total_sleep_us;
    uint32_t sleep_count;
} PowerManager;

// Forward declarations for internal functions
static WakeCause power_manager_read_wake_cause(const PowerManager* self);
static PowerError power_manager_apply_mode(PowerManager* self);
static PowerError power_manager_configure_gpio_wake_internal(const PowerManager* self);
static void power_manager_enable_wifi_power_save(const PowerManager* self);
static void power_manager_disable_wifi_power_save(const PowerManager* self);

// PowerManager constructor
static inline PowerManager power_manager_new(void) {
    PowerManager pm;
    pm.mode = POWER_MODE_BALANCED;
    pm.light_sleep_wake = wake_sources_or(wake_sources_or(WAKE_SOURCES_TIMER, WAKE_SOURCES_GPIO), WAKE_SOURCES_UART);
    pm.deep_sleep_wake = wake_sources_or(WAKE_SOURCES_TIMER, WAKE_SOURCES_GPIO);
    
    // Initialize gpio_wake_pins array with None
    for (int i = 0; i < 8; i++) {
        pm.gpio_wake_pins[i] = option_gpio_wake_config_none();
    }
    
    pm.cpu_freq_mhz = 240;
    pm.last_wake_cause = wake_cause_power_on();
    pm.total_sleep_us = 0;
    pm.sleep_count = 0;
    
    return pm;
}

// PowerManager init
PowerError power_manager_init(PowerManager* self) {
    // Read wake cause
    self->last_wake_cause = power_manager_read_wake_cause(self);
    
    // Apply mode
    PowerError err = power_manager_apply_mode(self);
    if (err != POWER_ERROR_OK) {
        return err;
    }
    
    return POWER_ERROR_OK;
}

// PowerManager set_mode
PowerError power_manager_set_mode(PowerManager* self, PowerMode mode) {
    self->mode = mode;
    return power_manager_apply_mode(self);
}

// PowerManager apply_mode (internal)
static PowerError power_manager_apply_mode(PowerManager* self) {
    uint32_t cpu_freq;
    bool wifi_ps;
    bool bt_power;
    
    switch (self->mode) {
        case POWER_MODE_PERFORMANCE:
            cpu_freq = 240;
            wifi_ps = false;
            bt_power = true;
            break;
        case POWER_MODE_BALANCED:
            cpu_freq = 160;
            wifi_ps = true;
            bt_power = true;
            break;
        case POWER_MODE_LOW_POWER:
            cpu_freq = 80;
            wifi_ps = true;
            bt_power = false;
            break;
        case POWER_MODE_ULTRA_LOW:
            cpu_freq = 40;
            wifi_ps = true;
            bt_power = false;
            break;
        default:
            cpu_freq = 160;
            wifi_ps = true;
            bt_power = true;
            break;
    }
    
    // Set CPU frequency
    PowerError err = power_manager_set_cpu_frequency(self, cpu_freq);
    if (err != POWER_ERROR_OK) {
        return err;
    }
    
    // Handle WiFi power save
    if (wifi_ps) {
        power_manager_enable_wifi_power_save(self);
    } else {
        power_manager_disable_wifi_power_save(self);
    }
    
    return POWER_ERROR_OK;
}

// PowerManager set_cpu_frequency
PowerError power_manager_set_cpu_frequency(PowerManager* self, uint32_t mhz) {
    // Validate frequency
    switch (mhz) {
        case 240:
        case 160:
        case 80:
        case 40:
        case 20:
        case 10:
            break;
        default:
            return POWER_ERROR_INVALID_FREQUENCY;
    }
    
    // Configure power management
    esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = (int32_t)mhz,
        .min_freq_mhz = 10,
        .light_sleep_enable = false
    };
    
    esp_err_t ret = esp_pm_configure(&pm_config);
    if (ret != ESP_OK) {
        return POWER_ERROR_CONFIG_FAILED;
    }
    
    self->cpu_freq_mhz = mhz;
    return POWER_ERROR_OK;
}

// PowerManager configure_gpio_wake
PowerError power_manager_configure_gpio_wake(PowerManager* self, GpioWakeConfig config) {
    // Find an empty slot or existing slot for this pin
    for (int i = 0; i < 8; i++) {
        if (!self->gpio_wake_pins[i].is_some || 
            (self->gpio_wake_pins[i].is_some && self->gpio_wake_pins[i].value.pin == config.pin)) {
            self->gpio_wake_pins[i] = option_gpio_wake_config_some(config);
            return POWER_ERROR_OK;
        }
    }
    
    return POWER_ERROR_TOO_MANY_WAKE_PINS;
}

// PowerManager light_sleep
PowerError power_manager_light_sleep(PowerManager* self, uint32_t duration_ms, WakeCause* out_cause) {
    if (duration_ms == 0) {
        return POWER_ERROR_INVALID_DURATION;
    }
    
    uint64_t duration_us = (uint64_t)duration_ms * 1000;
    
    // Enable timer wakeup
    if (wake_sources_has(self->light_sleep_wake, WAKE_SOURCES_TIMER)) {
        esp_sleep_enable_timer_wakeup(duration_us);
    }
    
    // Enable UART wakeup
    if (wake_sources_has(self->light_sleep_wake, WAKE_SOURCES_UART)) {
        esp_sleep_enable_uart_wakeup(0);
    }
    
    // Configure GPIO wakeup
    if (wake_sources_has(self->light_sleep_wake, WAKE_SOURCES_GPIO)) {
        PowerError err = power_manager_configure_gpio_wake_internal(self);
        if (err != POWER_ERROR_OK) {
            return err;
        }
    }
    
    // Enter light sleep
    esp_err_t ret = esp_light_sleep_start();
    if (ret != ESP_OK) {
        return POWER_ERROR_SLEEP_FAILED;
    }
    
    // Update statistics
    self->sleep_count += 1;
    self->total_sleep_us += duration_us;
    
    // Read and store wake cause
    WakeCause cause = power_manager_read_wake_cause(self);
    self->last_wake_cause = cause;
    
    if (out_cause != NULL) {
        *out_cause = cause;
    }
    
    return POWER_ERROR_OK;
}

// PowerManager deep_sleep (never returns)
void power_manager_deep_sleep(PowerManager* self, uint64_t duration_us) {
    // Clamp duration to valid range
    if (duration_us < MIN_DEEP_SLEEP_US) {
        esp_sleep_enable_timer_wakeup(MIN_DEEP_SLEEP_US);
    } else if (duration_us > MAX_DEEP_SLEEP_US) {
        esp_sleep_enable_timer_wakeup(MAX_DEEP_SLEEP_US);
    } else {
        esp_sleep_enable_timer_wakeup(duration_us);
    }
    
    // Configure GPIO wakeup if enabled
    if (wake_sources_has(self->deep_sleep_wake, WAKE_SOURCES_GPIO)) {
        power_manager_configure_gpio_wake_internal(self);
    }
    
    // Enter deep sleep (never returns)
    esp_deep_sleep_start();
    
    // Should never reach here, but loop forever just in case
    while (1) {
        __asm__ __volatile__("nop");
    }
}

// PowerManager configure_gpio_wake_internal (internal)
static PowerError power_manager_configure_gpio_wake_internal(const PowerManager* self) {
    uint64_t mask = 0;
    esp_sleep_ext1_wakeup_mode_t mode = ESP_EXT1_WAKEUP_ANY_HIGH;
    
    // Build GPIO mask from configured pins
    for (int i = 0; i < 8; i++) {
        if (self->gpio_wake_pins[i].is_some) {
            GpioWakeConfig cfg = self->gpio_wake_pins[i].value;
            if (cfg.pin < 64) {
                mask |= (1ULL << cfg.pin);
                if (!cfg.level_high) {
                    mode = ESP_EXT1_WAKEUP_ALL_LOW;
                }
            }
        }
    }
    
    // Enable EXT1 wakeup if any pins are configured
    if (mask != 0) {
        esp_sleep_enable_ext1_wakeup(mask, mode);
    }
    
    return POWER_ERROR_OK;
}

// PowerManager read_wake_cause (internal)
static WakeCause power_manager_read_wake_cause(const PowerManager* self) {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    
    switch (cause) {
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            return wake_cause_power_on();
        case ESP_SLEEP_WAKEUP_TIMER:
            return wake_cause_timer();
        case ESP_SLEEP_WAKEUP_EXT0:
            return wake_cause_ext0();
        case ESP_SLEEP_WAKEUP_EXT1: {
            // Get GPIO status to determine which pin woke up
            uint64_t gpio_mask = esp_sleep_get_ext1_wakeup_status();
            uint8_t gpio = (uint8_t)__builtin_ctzll(gpio_mask);
            return wake_cause_gpio(gpio);
        }
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            return wake_cause_touch();
        case ESP_SLEEP_WAKEUP_ULP:
            return wake_cause_ulp();
        case ESP_SLEEP_WAKEUP_GPIO:
            return wake_cause_gpio(0);
        case ESP_SLEEP_WAKEUP_UART:
            return wake_cause_uart();
        case ESP_SLEEP_WAKEUP_WIFI:
            return wake_cause_wifi();
        case ESP_SLEEP_WAKEUP_BT:
            return wake_cause_ble();
        default:
            return wake_cause_unknown();
    }
}

// PowerManager enable_wifi_power_save (internal)
static void power_manager_enable_wifi_power_save(const PowerManager* self) {
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

// PowerManager disable_wifi_power_save (internal)
static void power_manager_disable_wifi_power_save(const PowerManager* self) {
    esp_wifi_set_ps(WIFI_PS_NONE);
}

// PowerManager getters
PowerMode power_manager_mode(const PowerManager* self) {
    return self->mode;
}

WakeCause power_manager_last_wake_cause(const PowerManager* self) {
    return self->last_wake_cause;
}

uint64_t power_manager_total_sleep_us(const PowerManager* self) {
    return self->total_sleep_us;
}

uint32_t power_manager_sleep_count(const PowerManager* self) {
    return self->sleep_count;
}

uint32_t power_manager_cpu_freq_mhz(const PowerManager* self) {
    return self->cpu_freq_mhz;
}

// PowerManager static utility functions
bool power_manager_is_battery_low(uint32_t voltage_mv) {
    return voltage_mv < LOW_BATTERY_THRESHOLD_MV;
}

bool power_manager_is_battery_critical(uint32_t voltage_mv) {
    return voltage_mv < CRITICAL_BATTERY_THRESHOLD_MV;
}

// PowerManager RTC data access
void power_manager_set_rtc_data(const PowerManager* self, uint8_t slot, uint32_t value) {
    if (slot >= 8) {
        return;
    }
    
    // Write to RTC memory
    volatile uint32_t* rtc_mem = (volatile uint32_t*)(0x50000000 + (uint32_t)slot * 4);
    *rtc_mem = value;
}

uint32_t power_manager_get_rtc_data(const PowerManager* self, uint8_t slot) {
    if (slot >= 8) {
        return 0;
    }
    
    // Read from RTC memory
    volatile uint32_t* rtc_mem = (volatile uint32_t*)(0x50000000 + (uint32_t)slot * 4);
    return *rtc_mem;
}

// PowerManager default constructor
static inline PowerManager power_manager_default(void) {
    return power_manager_new();
}
