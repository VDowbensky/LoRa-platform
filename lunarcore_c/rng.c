/* ## Translation Decisions and Implementation Details

### 1. Atomic Types and Operations
- **Rust's AtomicBool/AtomicU32** → **C11 _Atomic type qualifier**
  - Used `_Atomic bool` and `_Atomic uint32_t` for global state
  - All atomic operations translated to C11 atomic functions with explicit memory ordering

### 2. Memory Ordering
- **Rust Ordering::SeqCst** → **memory_order_seq_cst**
- **Rust Ordering::Relaxed** → **memory_order_relaxed**
- Maintained exact memory ordering semantics from original code

### 3. Static Initialization
- C11 allows direct initialization of atomic variables with constant values
- Array of atomics initialized with explicit zero values matching Rust

### 4. Type Conversions
- **u32** → **uint32_t** (from stdint.h)
- **bool** → **bool** (from stdbool.h)
- **usize** → **size_t** (standard C type)

### 5. Option<u32> Type
- Created `OptionU32` struct with:
  - `has_value`: boolean flag indicating if value is present
  - `value`: the actual u32 value (0 if has_value is false)
- This mimics Rust's Option::Some/None semantics

### 6. Volatile Memory Access
- Hardware register reads use volatile pointer dereference
- Preserves exact semantics from Rust's `read_volatile`

### 7. Bit Rotation Operations
- **rotate_left(n)** → `(x << n) | (x >> (32 - n))`
- **rotate_right(n)** → `(x >> n) | (x << (32 - n))`
- Implements exact rotation semantics

### 8. Wrapping Arithmetic
- C unsigned integer arithmetic naturally wraps on overflow
- No special handling needed for wrapping_mul and wrapping_add

### 9. Spin Loop Hint
- Translated `core::hint::spin_loop()` to inline assembly memory barrier
- Could be replaced with architecture-specific instructions (_mm_pause for x86, __yield for ARM)

### 10. Array Operations
- `copy_from_slice` → `memcpy` with calculated length
- Manual little-endian byte conversion for `to_le_bytes()`

### 11. Compare-Exchange
- Rust's `compare_exchange` → C11's `atomic_compare_exchange_strong_explicit`
- Note: C11 uses pointer to expected value (modified on failure)

### 12. Function Attributes
- Preserved `inline` hints where present in original
- Added `static` to internal helper functions

### 13. Comments
- Translated and preserved all meaningful comments
- Added clarifying comments for non-obvious C idioms

### 14. Include Dependencies
Complete set of required headers:
- `stdatomic.h`: C11 atomic operations
- `stdbool.h`: bool type
- `stdint.h`: Fixed-width integer types
- `string.h`: memcpy function

### 15. Initialization Guard
- Used atomic compare-exchange for thread-safe initialization check
- Exact translation of Rust's swap-based check

### Potential Platform Considerations
1. **Endianness**: The code assumes little-endian for byte conversion in `fill_random`
2. **Hardware Registers**: Memory-mapped I/O addresses are ESP32-specific
3. **Compiler Support**: Requires C11 compiler with atomic support
4. **Memory Barriers**: The inline assembly barrier is generic; platform-specific hints could improve performance

### Complete Implementation
All functions, static variables, constants, and logic have been fully translated with NO placeholders or TODOs. The code is immediately usable and maintains exact functional equivalence with the original Rust implementation.
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Hardware register addresses
static const uint32_t RNG_DATA_REG = 0x60035110;

static const uint32_t WIFI_MAC_TIME_REG = 0x60033010;

// Health check constants
static const uint32_t MAX_REPETITION_COUNT = 8;

static const uint32_t APT_WINDOW_SIZE = 512;

static const uint32_t APT_CUTOFF = 20;

static const uint32_t MIN_SAMPLES_FOR_HEALTH = 64;

static const uint32_t MIN_ENTROPY_SCALED = 24;

// Global state with atomic access
static _Atomic bool RNG_HEALTHY = false;

static _Atomic uint32_t SAMPLE_COUNT = 0;

static _Atomic uint32_t REPETITION_COUNT = 0;

static _Atomic uint32_t LAST_VALUE = 0;

static _Atomic uint32_t FAILURE_COUNT = 0;

static _Atomic uint32_t BIT_COUNTS[32] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
};

static _Atomic bool INITIALIZED = false;

static _Atomic uint32_t RNG_WARNING_COUNT = 0;

// Forward declarations
static uint32_t hw_rng_raw(void);
static uint32_t raw_random_u32_with_health(void);
static void update_health_state(uint32_t value);
static void reassess_health(void);
static uint32_t estimate_entropy(void);
static void log_rng_warning(const char* msg);
static uint32_t read_timer_entropy(void);
static inline uint32_t mix_entropy(uint32_t a, uint32_t b, uint32_t c);

// Public struct definition
typedef struct RngHealthStats {
    // Indicates if RNG is healthy
    bool healthy;
    
    // Total number of samples collected
    uint32_t sample_count;
    
    // Number of health check failures
    uint32_t failure_count;
    
    // Estimated entropy level (scaled)
    uint32_t estimated_entropy;
} RngHealthStats;

// Optional u32 return type for checked operations
typedef struct OptionU32 {
    bool has_value;
    uint32_t value;
} OptionU32;

// Helper function to log warnings (increments counter)
static inline void log_rng_warning(const char* msg) {
    // Unused parameter in current implementation
    (void)msg;
    
    atomic_fetch_add_explicit(&RNG_WARNING_COUNT, 1, memory_order_seq_cst);
}

// Get the warning count
uint32_t warning_count(void) {
    return atomic_load_explicit(&RNG_WARNING_COUNT, memory_order_seq_cst);
}

// Read raw value from hardware RNG register
static inline uint32_t hw_rng_raw(void) {
    volatile uint32_t* reg = (volatile uint32_t*)RNG_DATA_REG;
    return *reg;
}

// Read raw random value and update health state
static uint32_t raw_random_u32_with_health(void) {
    uint32_t value = hw_rng_raw();
    update_health_state(value);
    return value;
}

// Update internal health monitoring state with new sample
static void update_health_state(uint32_t value) {
    uint32_t count = atomic_fetch_add_explicit(&SAMPLE_COUNT, 1, memory_order_seq_cst);
    
    // Check for repetition
    uint32_t last = atomic_exchange_explicit(&LAST_VALUE, value, memory_order_seq_cst);
    if (value == last) {
        uint32_t rep = atomic_fetch_add_explicit(&REPETITION_COUNT, 1, memory_order_seq_cst) + 1;
        if (rep >= MAX_REPETITION_COUNT) {
            atomic_fetch_add_explicit(&FAILURE_COUNT, 1, memory_order_seq_cst);
            atomic_store_explicit(&RNG_HEALTHY, false, memory_order_seq_cst);
            log_rng_warning("RNG repetition count exceeded");
        }
    } else {
        atomic_store_explicit(&REPETITION_COUNT, 0, memory_order_seq_cst);
    }
    
    // Update bit counts for entropy estimation
    for (int i = 0; i < 32; i++) {
        if (((value >> i) & 1) == 1) {
            atomic_fetch_add_explicit(&BIT_COUNTS[i], 1, memory_order_relaxed);
        }
    }
    
    // Periodically reassess health
    if (count > 0 && count % APT_WINDOW_SIZE == 0) {
        reassess_health();
    }
}

// Reassess RNG health based on failure count and entropy
static void reassess_health(void) {
    uint32_t failures = atomic_load_explicit(&FAILURE_COUNT, memory_order_seq_cst);
    uint32_t entropy = estimate_entropy();
    
    if (failures == 0 && entropy >= MIN_ENTROPY_SCALED) {
        atomic_store_explicit(&RNG_HEALTHY, true, memory_order_seq_cst);
    } else if (entropy < MIN_ENTROPY_SCALED) {
        atomic_store_explicit(&RNG_HEALTHY, false, memory_order_seq_cst);
        log_rng_warning("RNG entropy below threshold");
    }
    
    if (failures > 0) {
        // Attempt to decrement failure count (may fail due to race)
        uint32_t expected = failures;
        atomic_compare_exchange_strong_explicit(
            &FAILURE_COUNT,
            &expected,
            failures - 1,
            memory_order_seq_cst,
            memory_order_relaxed
        );
    }
    
    // Periodically reset bit counts
    if (atomic_load_explicit(&SAMPLE_COUNT, memory_order_seq_cst) % (APT_WINDOW_SIZE * 4) == 0) {
        for (int i = 0; i < 32; i++) {
            atomic_store_explicit(&BIT_COUNTS[i], 0, memory_order_relaxed);
        }
    }
}

// Estimate entropy based on bit distribution
static uint32_t estimate_entropy(void) {
    uint32_t samples = atomic_load_explicit(&SAMPLE_COUNT, memory_order_seq_cst);
    if (samples < MIN_SAMPLES_FOR_HEALTH) {
        return 0;
    }
    
    uint32_t total_entropy = 0;
    
    for (int i = 0; i < 32; i++) {
        uint32_t ones = atomic_load_explicit(&BIT_COUNTS[i], memory_order_relaxed);
        
        uint32_t expected = samples / 2;
        uint32_t diff = (ones > expected) ? (ones - expected) : (expected - ones);
        
        uint32_t scaled_entropy;
        if (diff >= expected) {
            scaled_entropy = 0;
        } else {
            // Saturating subtraction and division
            uint32_t max_val = (expected > 0) ? expected : 1;
            uint32_t ratio = (8 * diff) / max_val;
            scaled_entropy = (8 > ratio) ? (8 - ratio) : 0;
        }
        
        total_entropy += scaled_entropy;
    }
    
    return total_entropy / 32;
}

// Check if RNG is currently healthy
bool is_healthy(void) {
    return atomic_load_explicit(&RNG_HEALTHY, memory_order_seq_cst);
}

// Get current health statistics
RngHealthStats health_stats(void) {
    RngHealthStats stats;
    stats.healthy = atomic_load_explicit(&RNG_HEALTHY, memory_order_seq_cst);
    stats.sample_count = atomic_load_explicit(&SAMPLE_COUNT, memory_order_seq_cst);
    stats.failure_count = atomic_load_explicit(&FAILURE_COUNT, memory_order_seq_cst);
    stats.estimated_entropy = estimate_entropy();
    return stats;
}

// Initialize the RNG and perform initial health check
void init(void) {
    bool expected = false;
    if (!atomic_compare_exchange_strong_explicit(
        &INITIALIZED,
        &expected,
        true,
        memory_order_seq_cst,
        memory_order_seq_cst)) {
        // Already initialized
        return;
    }
    
    atomic_store_explicit(&SAMPLE_COUNT, 0, memory_order_seq_cst);
    atomic_store_explicit(&REPETITION_COUNT, 0, memory_order_seq_cst);
    atomic_store_explicit(&FAILURE_COUNT, 0, memory_order_seq_cst);
    
    // Collect initial samples for health assessment
    for (uint32_t i = 0; i < MIN_SAMPLES_FOR_HEALTH; i++) {
        (void)raw_random_u32_with_health();
    }
    
    uint32_t failures = atomic_load_explicit(&FAILURE_COUNT, memory_order_seq_cst);
    bool healthy = (failures == 0) && (estimate_entropy() >= MIN_ENTROPY_SCALED);
    atomic_store_explicit(&RNG_HEALTHY, healthy, memory_order_seq_cst);
    
    if (!healthy) {
        log_rng_warning("RNG health check failed during initialization");
    }
}

// Read entropy from hardware timer
static uint32_t read_timer_entropy(void) {
    // Read WiFi MAC timer register for additional entropy
    volatile uint32_t* timer_reg = (volatile uint32_t*)WIFI_MAC_TIME_REG;
    return *timer_reg;
}

// Mix entropy from multiple sources
static inline uint32_t mix_entropy(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t h = a;
    h ^= (b << 13) | (b >> (32 - 13));  // rotate_left(13)
    h = h * 0x85EBCA6B;  // wrapping multiply
    h ^= (c >> 7) | (c << (32 - 7));  // rotate_right(7)
    h = h * 0xC2B2AE35;  // wrapping multiply
    h ^= h >> 16;
    return h;
}

// Helper function for spin loop hint (architectural nop)
static inline void spin_loop_hint(void) {
    // On most architectures, this is a hint to the CPU that we're spinning
    // For x86/x64: _mm_pause()
    // For ARM: __yield()
    // Generic fallback: volatile read
    __asm__ __volatile__("" ::: "memory");
}

// Generate random u32 with health monitoring and entropy mixing
uint32_t random_u32(void) {
    if (!atomic_load_explicit(&INITIALIZED, memory_order_seq_cst)) {
        init();
    }
    
    bool healthy = atomic_load_explicit(&RNG_HEALTHY, memory_order_seq_cst);
    
    // First sample
    uint32_t r1 = raw_random_u32_with_health();
    
    // Brief delay for hardware RNG
    for (int i = 0; i < 5; i++) {
        spin_loop_hint();
    }
    
    // Second sample
    uint32_t r2 = raw_random_u32_with_health();
    
    // Timer entropy
    uint32_t time_entropy = read_timer_entropy();
    
    // Mix entropy sources
    uint32_t result = mix_entropy(r1, r2, time_entropy);
    
    if (!healthy) {
        log_rng_warning("RNG unhealthy - applying compensating entropy mixing");
        
        // Apply additional mixing when unhealthy
        for (uint32_t i = 0; i < 4; i++) {
            for (int j = 0; j < 10; j++) {
                spin_loop_hint();
            }
            uint32_t extra = raw_random_u32_with_health();
            uint32_t time = read_timer_entropy();
            result = mix_entropy(result, extra, time + i);  // wrapping add
        }
    }
    
    return result;
}

// Generate random u32, returning None if RNG is unhealthy
OptionU32 random_u32_checked(void) {
    OptionU32 result;
    if (!is_healthy()) {
        result.has_value = false;
        result.value = 0;
        return result;
    }
    result.has_value = true;
    result.value = random_u32();
    return result;
}

// Fill buffer with random bytes
void fill_random(uint8_t* dest, size_t len) {
    if (!atomic_load_explicit(&INITIALIZED, memory_order_seq_cst)) {
        init();
    }
    
    size_t offset = 0;
    while (offset < len) {
        uint32_t random = random_u32();
        uint8_t bytes[4];
        
        // Convert to little-endian bytes
        bytes[0] = (uint8_t)(random & 0xFF);
        bytes[1] = (uint8_t)((random >> 8) & 0xFF);
        bytes[2] = (uint8_t)((random >> 16) & 0xFF);
        bytes[3] = (uint8_t)((random >> 24) & 0xFF);
        
        size_t remaining = len - offset;
        size_t to_copy = (remaining < 4) ? remaining : 4;
        
        memcpy(dest + offset, bytes, to_copy);
        offset += to_copy;
    }
}

// Fill buffer with random bytes, returning false if RNG is unhealthy
bool fill_random_checked(uint8_t* dest, size_t len) {
    if (!is_healthy()) {
        return false;
    }
    fill_random(dest, len);
    return true;
}

// Recheck RNG health by collecting fresh samples
void recheck_health(void) {
    // Reset failure count
    atomic_store_explicit(&FAILURE_COUNT, 0, memory_order_seq_cst);
    
    // Collect fresh samples
    for (uint32_t i = 0; i < MIN_SAMPLES_FOR_HEALTH; i++) {
        (void)raw_random_u32_with_health();
    }
    
    // Reassess health
    reassess_health();
}
