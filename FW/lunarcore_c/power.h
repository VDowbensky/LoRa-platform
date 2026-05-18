#ifndef _POWER_H_
#define _POWER_H_

// Constants
#define MIN_DEEP_SLEEP_US 1000ULL
#define MAX_DEEP_SLEEP_US 86400000000ULL
#define DEFAULT_LIGHT_SLEEP_MS 100U
#define LOW_BATTERY_THRESHOLD_MV 3400U
#define CRITICAL_BATTERY_THRESHOLD_MV 3200U

// PowerMode enum
typedef enum 
{
  POWER_MODE_PERFORMANCE,
  POWER_MODE_BALANCED,
  POWER_MODE_LOW_POWER,
  POWER_MODE_ULTRA_LOW
} PowerMode;

// WakeCause enum with tagged union
typedef enum 
{
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

typedef struct 
{
  WakeCauseType type;
  union 
  {
    uint8_t gpio_pin;
  } data;
} WakeCause;

// GpioWakeConfig struct
typedef struct 
{
  uint8_t pin;
  bool level_high;
} GpioWakeConfig;

// Option<GpioWakeConfig> implementation
typedef struct 
{
  bool is_some;
  GpioWakeConfig value;
} OptionGpioWakeConfig;

// PowerError enum
typedef enum 
{
  POWER_ERROR_OK = 0,
  POWER_ERROR_INVALID_FREQUENCY,
  POWER_ERROR_CONFIG_FAILED,
  POWER_ERROR_INVALID_DURATION,
  POWER_ERROR_SLEEP_FAILED,
  POWER_ERROR_TOO_MANY_WAKE_PINS
} PowerError;

// PowerManager struct
typedef struct 
{
  PowerMode mode;
  WakeSources light_sleep_wake;
  WakeSources deep_sleep_wake;
  OptionGpioWakeConfig gpio_wake_pins[8];
  uint32_t cpu_freq_mhz;
  WakeCause last_wake_cause;
  uint64_t total_sleep_us;
  uint32_t sleep_count;
} PowerManager;


#endif
