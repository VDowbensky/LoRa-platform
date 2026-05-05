#include "bsp.h"

// External RNG function declaration (from separate module)
extern uint32_t random_u32(void);

// Base address for RTC slow memory
#define RTC_SLOW_MEM_BASE 0x50000000U

// RTC slot indices
#define RTC_SLOT_SESSION_HI 8U
#define RTC_SLOT_SESSION_LO 9U
#define RTC_SLOT_SEQUENCE 10U
#define RTC_SLOT_BOOT_COUNT 11U
#define RTC_SLOT_MAGIC 12U

// Magic number for RTC validation
#define RTC_MAGIC 0x4C554E42U

// Maximum sequence value before rotation
#define MAX_SEQUENCE 0xFFFFFFFEU

// Sequence skip value on wake
#define WAKE_SEQUENCE_SKIP 256U

// Hardware RNG data register
#define RNG_DATA_REG 0x60035110U

// Packet ID state structure
typedef struct 
{
  uint32_t session_id;
  uint32_t sequence;
  uint32_t boot_count;
} PacketIdState;

// Static atomic variables for packet ID management
static atomic_uint PACKET_ID_SESSION = ATOMIC_VAR_INIT(0);
static atomic_uint PACKET_ID_SEQUENCE = ATOMIC_VAR_INIT(0);
static atomic_uint BOOT_COUNT = ATOMIC_VAR_INIT(0);
static atomic_bool INITIALIZED = ATOMIC_VAR_INIT(false);

// Forward declarations
static void rotate_session_internal(void);
static uint32_t generate_session_id(uint32_t previous, uint32_t boot_count);

// Read from RTC slow memory slot
static inline uint32_t rtc_read(uint32_t slot) 
{
  volatile uint32_t *addr = (volatile uint32_t *)(RTC_SLOW_MEM_BASE + slot * 4);
  return *addr;
}

// Write to RTC slow memory slot
static inline void rtc_write(uint32_t slot, uint32_t value) 
{
  volatile uint32_t *addr = (volatile uint32_t *)(RTC_SLOW_MEM_BASE + slot * 4);
  *addr = value;
}

// Get hardware random number (wrapper for external RNG)
static inline uint32_t hw_rng_u32(void) 
{
  return random_u32();
}

// Raw hardware RNG read (unused but included for completeness)
static inline uint32_t hw_rng_raw(void) __attribute__((unused));
static inline uint32_t hw_rng_raw(void) 
{
  volatile uint32_t *rng_reg = (volatile uint32_t *)RNG_DATA_REG;
  uint32_t r1 = *rng_reg;
  // Spin delay
  for (int i = 0; i < 10; i++) 
  {
    // Spin loop hint (compiler optimization barrier)
    __asm__ volatile("" ::: "memory");
  }
  uint32_t r2 = *rng_reg;
  // Rotate left by 13 bits
  uint32_t rotated = (r2 << 13) | (r2 >> (32 - 13));
  return r1 ^ rotated;
}

// Read RTC time register
static uint32_t read_rtc_time(void) 
{
  volatile uint32_t *rtc_time_low = (volatile uint32_t *)0x60008048U;
  return *rtc_time_low;
}

// 32-bit mixing function for random number enhancement
static inline uint32_t mix32(uint32_t x) 
{
  x ^= x >> 16;
  x = x * 0x85EBCA6BU;
  x ^= x >> 13;
  x = x * 0xC2B2AE35U;
  x ^= x >> 16;
  return x;
}

// Create new session for PacketIdState
static void PacketIdState_new_session(PacketIdState *self) 
{
  uint32_t hw_random1 = hw_rng_u32();
  uint32_t hw_random2 = hw_rng_u32();
  uint32_t boot_entropy = self->boot_count * 0x9E3779B9U;
  uint32_t timestamp = read_rtc_time();
  uint32_t mixed = hw_random1 ^ boot_entropy ^ timestamp ^ (self->session_id * 0x85EBCA6BU);
  mixed = mix32(mixed);
  mixed = mix32(mixed ^ hw_random2);
  self->session_id = mixed;
  if (self->session_id == 0) self->session_id = mix32(hw_rng_u32()) | 1;
  self->sequence = 0;
  // Persist state to RTC memory
  rtc_write(RTC_SLOT_SESSION_HI, self->session_id);
  rtc_write(RTC_SLOT_SEQUENCE, self->sequence);
}

// Persist PacketIdState to RTC memory
static void PacketIdState_persist(const PacketIdState *self) 
{
  rtc_write(RTC_SLOT_SESSION_HI, self->session_id);
  rtc_write(RTC_SLOT_SEQUENCE, self->sequence);
}

// Convert PacketIdState to 64-bit value
static uint64_t PacketIdState_to_u64(const PacketIdState *self) 
{
  return ((uint64_t)self->session_id << 32) | (uint64_t)self->sequence;
}

// Generate a new session ID
static uint32_t generate_session_id(uint32_t previous, uint32_t boot_count) 
{
  uint32_t hw_random1 = hw_rng_u32();
  uint32_t hw_random2 = hw_rng_u32();
  uint32_t boot_entropy = boot_count * 0x9E3779B9U;
  uint32_t timestamp = read_rtc_time();
  uint32_t mixed = hw_random1 ^ boot_entropy ^ timestamp ^ (previous * 0x85EBCA6BU);
  mixed = mix32(mixed);
  mixed = mix32(mixed ^ hw_random2);
  if (mixed == 0) return mix32(hw_rng_u32()) | 1;
  else return mixed;
}

// Rotate to a new session (internal implementation)
static void rotate_session_internal(void) 
{
  uint32_t boot_count = atomic_load_explicit(&BOOT_COUNT, memory_order_seq_cst);
  uint32_t old_session = atomic_load_explicit(&PACKET_ID_SESSION, memory_order_seq_cst);
  uint32_t new_session = generate_session_id(old_session, boot_count);
  atomic_store_explicit(&PACKET_ID_SESSION, new_session, memory_order_seq_cst);
  atomic_store_explicit(&PACKET_ID_SEQUENCE, 0, memory_order_seq_cst);
  rtc_write(RTC_SLOT_SESSION_HI, new_session);
  rtc_write(RTC_SLOT_SEQUENCE, 0);
}

// Initialize the packet ID system
void init(void) 
{
  // Check if already initialized
  if (atomic_load_explicit(&INITIALIZED, memory_order_acquire)) return;
  // Try to atomically set initialized flag
  bool expected = false;
  if (!atomic_compare_exchange_strong_explicit(&INITIALIZED, &expected, true,memory_order_seq_cst,memory_order_seq_cst)) 
  {
    // Another thread is initializing, wait for completion
    while (!atomic_load_explicit(&INITIALIZED, memory_order_acquire)) 
    {
      // Spin loop hint
      __asm__ volatile("" ::: "memory");
    }
    return;
  }
  // Read magic value from RTC
  uint32_t magic = rtc_read(RTC_SLOT_MAGIC);
  uint32_t session_id;
  uint32_t sequence;
  uint32_t boot_count;
  if (magic == RTC_MAGIC) 
  {
    // Valid RTC data exists, restore state
    uint32_t stored_session = rtc_read(RTC_SLOT_SESSION_HI);
    uint32_t stored_sequence = rtc_read(RTC_SLOT_SEQUENCE);
    uint32_t stored_boot_count = rtc_read(RTC_SLOT_BOOT_COUNT);
    boot_count = stored_boot_count + 1;
    if (stored_sequence >= MAX_SEQUENCE) 
    {
      // Sequence exhausted, generate new session
      uint32_t new_session = generate_session_id(stored_session, boot_count);
      session_id = new_session;
      sequence = 0;
    } 
    else 
    {
      // Continue with existing session, skip ahead in sequence
      uint32_t new_sequence = stored_sequence + WAKE_SEQUENCE_SKIP;
      // Saturating add (clamp to MAX_SEQUENCE if overflow would occur)
      if (new_sequence < stored_sequence) new_sequence = MAX_SEQUENCE;
      session_id = stored_session;
      sequence = new_sequence;
    }
  } 
  else 
  {
    // First boot or invalid RTC data, initialize fresh
    uint32_t new_session = generate_session_id(0, 0);
    rtc_write(RTC_SLOT_MAGIC, RTC_MAGIC);
    session_id = new_session;
    sequence = 0;
    boot_count = 0;
  }
  // Store boot count
  atomic_store_explicit(&BOOT_COUNT, boot_count, memory_order_seq_cst);
  rtc_write(RTC_SLOT_BOOT_COUNT, boot_count);
  // Store session and sequence
  atomic_store_explicit(&PACKET_ID_SESSION, session_id, memory_order_seq_cst);
  atomic_store_explicit(&PACKET_ID_SEQUENCE, sequence, memory_order_seq_cst);
  // Persist to RTC
  rtc_write(RTC_SLOT_SESSION_HI, session_id);
  rtc_write(RTC_SLOT_SEQUENCE, sequence);
}

// Get next packet ID (32-bit)
uint32_t next_packet_id(void) 
{
  init();
  uint32_t sequence = atomic_fetch_add_explicit(&PACKET_ID_SEQUENCE, 1, memory_order_seq_cst);
  uint32_t session_id = atomic_load_explicit(&PACKET_ID_SESSION, memory_order_seq_cst);
  if (sequence >= MAX_SEQUENCE) rotate_session_internal();// Rotate to new session
  // Periodically persist state to RTC
  if ((sequence & 0xFF) == 0) 
  {
    rtc_write(RTC_SLOT_SESSION_HI, session_id);
    rtc_write(RTC_SLOT_SEQUENCE, sequence);
  }
  return mix32(session_id ^ (sequence * 0x85EBCA6BU));
}

// Manually rotate to a new session
void rotate_session(void) 
{
  init();
  rotate_session_internal();
}

// Get current session information
void session_info(uint32_t *session_id, uint32_t *sequence, uint32_t *boot_count) 
{
  init();
  *session_id = atomic_load_explicit(&PACKET_ID_SESSION, memory_order_seq_cst);
  *sequence = atomic_load_explicit(&PACKET_ID_SEQUENCE, memory_order_seq_cst);
  *boot_count = atomic_load_explicit(&BOOT_COUNT, memory_order_seq_cst);
}

// Get next packet ID (64-bit)
uint64_t next_packet_id_64(void) 
{
  init();
  uint32_t sequence = atomic_fetch_add_explicit(&PACKET_ID_SEQUENCE, 1, memory_order_seq_cst);
  uint32_t session_id = atomic_load_explicit(&PACKET_ID_SESSION, memory_order_seq_cst);
  if (sequence >= MAX_SEQUENCE) rotate_session_internal();
  return ((uint64_t)session_id << 32) | (uint64_t)sequence;
}

// Invalidate RTC data
void invalidate_rtc(void) 
{
  rtc_write(RTC_SLOT_MAGIC, 0);
}
