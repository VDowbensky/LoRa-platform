#include "packet.h"
 

// Required type definitions from parent module
typedef enum 
{
  PRIORITY_DEFAULT,
  PRIORITY_HIGH,
  PRIORITY_LOW
} Priority;

// Maximum capacity for heapless Vec equivalent
#define VEC_MAX_CAPACITY 256

// Vec structure to replace heapless::Vec
typedef struct 
{
  uint8_t data[VEC_MAX_CAPACITY];
  size_t len;
  size_t capacity;
} Vec;

// PacketPayload enum and union
typedef enum 
{
  PACKET_PAYLOAD_ENCRYPTED,
  PACKET_PAYLOAD_DECRYPTED
} PacketPayloadType;

typedef struct 
{
  PacketPayloadType type;
  Vec data;
} PacketPayload;

// MeshPacket structure
typedef struct 
{
  uint32_t from;
  uint32_t to;
  uint8_t channel;
  uint32_t id;
  uint8_t hop_limit;
  bool want_ack;
  Priority priority;
  uint32_t rx_time;
  float rx_snr;
  int32_t rx_rssi;
  PacketPayload payload;
} MeshPacket;

// Constants from parent module
#define LORA_HEADER_SIZE 17
#define MAX_LORA_PAYLOAD 237
#define MIC_SIZE 4
#define DEFAULT_HOP_LIMIT 3

// SHA256 hash function declaration
void sha256_hash(const uint8_t* data, size_t len, uint8_t* hash);

// Constants
#define OFFSET_TO 0
#define OFFSET_FROM 4
#define OFFSET_ID 8
#define OFFSET_FLAGS 12
#define OFFSET_CHANNEL_HASH 13

#define FLAG_WANT_ACK 0x01
#define FLAG_HOP_LIMIT_MASK 0x0E
#define FLAG_HOP_LIMIT_SHIFT 1
#define FLAG_CHANNEL_MASK 0xF0
#define FLAG_CHANNEL_SHIFT 4

// Vec helper functions
static void vec_init(Vec* vec) 
{
  vec->len = 0;
  vec->capacity = VEC_MAX_CAPACITY;
  memset(vec->data, 0, VEC_MAX_CAPACITY);
}

static bool vec_extend_from_slice(Vec* vec, const uint8_t* data, size_t len) 
{
  if (vec->len + len > vec->capacity) return false;
  memcpy(vec->data + vec->len, data, len);
  vec->len += len;
  return true;
}

static bool vec_push(Vec* vec, uint8_t byte) 
{
  if (vec->len >= vec->capacity) return false;
  vec->data[vec->len] = byte;
  vec->len++;
  return true;
}

// Helper function to read u32 from little-endian bytes
static uint32_t read_u32_le(const uint8_t* data) 
{
  return (uint32_t)data[0] |  ((uint32_t)data[1] << 8) |  ((uint32_t)data[2] << 16) |  ((uint32_t)data[3] << 24);
}

// Helper function to write u32 to little-endian bytes
static void write_u32_le(uint8_t* dest, uint32_t value) 
{
  dest[0] = (uint8_t)(value & 0xFF);
  dest[1] = (uint8_t)((value >> 8) & 0xFF);
  dest[2] = (uint8_t)((value >> 16) & 0xFF);
  dest[3] = (uint8_t)((value >> 24) & 0xFF);
}

// Compute MIC (Message Integrity Code)
static void compute_mic(const uint8_t* data, size_t len, uint8_t mic[MIC_SIZE]) 
{
  uint8_t hash[32]; // SHA256 produces 32 bytes
  sha256_hash(data, len, hash);
  memcpy(mic, hash, MIC_SIZE);
}

// Compute channel hash
static uint8_t compute_channel_hash(uint8_t channel) 
{
  // Simple hash function
  uint8_t h = (uint8_t)((channel * 0x9E) + 0x37);
  h ^= h >> 4;
  return (uint8_t)(h * 0xB5);
}

// Constant time comparison for security
static bool constant_time_eq(const uint8_t* a, const uint8_t* b, size_t len) 
{
  uint8_t result = 0;
  for (size_t i = 0; i < len; i++) result |= a[i] ^ b[i];
  return result == 0;
}

// Parse LoRa packet from raw bytes
bool parse_lora_packet(const uint8_t* data, size_t data_len, MeshPacket* out_packet) 
{
  // Check minimum size
  if (data_len < LORA_HEADER_SIZE + MIC_SIZE) return false;
  // Extract header fields
  uint32_t to = read_u32_le(&data[OFFSET_TO]);
  uint32_t from = read_u32_le(&data[OFFSET_FROM]);
  uint32_t id = read_u32_le(&data[OFFSET_ID]);
  uint8_t flags = data[OFFSET_FLAGS];
  uint8_t channel_hash = data[OFFSET_CHANNEL_HASH];
  // Parse flags
  bool want_ack = (flags & FLAG_WANT_ACK) != 0;
  uint8_t hop_limit = (flags & FLAG_HOP_LIMIT_MASK) >> FLAG_HOP_LIMIT_SHIFT;
  uint8_t channel = (flags & FLAG_CHANNEL_MASK) >> FLAG_CHANNEL_SHIFT;
  // Calculate payload boundaries
  size_t payload_end = data_len - MIC_SIZE;
  size_t payload_start = LORA_HEADER_SIZE;
  if (payload_end <= payload_start) return false;
  // Extract payload
  Vec payload_data;
  vec_init(&payload_data);
  if (!vec_extend_from_slice(&payload_data, &data[payload_start], payload_end - payload_start)) return false;
  // Verify MIC
  const uint8_t* received_mic = &data[payload_end];
  uint8_t computed_mic[MIC_SIZE];
  compute_mic(data, payload_end, computed_mic);
  if (!constant_time_eq(received_mic, computed_mic, MIC_SIZE)) 
  {
      // MIC verification failed, but we continue (as in original Rust code)
  }
  // Populate output packet
  out_packet->from = from;
  out_packet->to = to;
  out_packet->channel = channel;
  out_packet->id = id;
  out_packet->hop_limit = hop_limit;
  out_packet->want_ack = want_ack;
  out_packet->priority = PRIORITY_DEFAULT;
  out_packet->rx_time = 0;
  out_packet->rx_snr = 0.0f;
  out_packet->rx_rssi = 0;
  out_packet->payload.type = PACKET_PAYLOAD_ENCRYPTED;
  out_packet->payload.data = payload_data;
  return true;
}

// Build LoRa packet from components
bool build_lora_packet(uint32_t from,uint32_t to,uint32_t id,uint8_t channel,uint8_t hop_limit,
                        bool want_ack,const uint8_t* payload,size_t payload_len,Vec* out_packet) 
{
  size_t total_len = LORA_HEADER_SIZE + payload_len + MIC_SIZE;
  if (total_len > 256 || payload_len > MAX_LORA_PAYLOAD) return false;
  vec_init(out_packet);
  // Add 'to' field
  uint8_t to_bytes[4];
  write_u32_le(to_bytes, to);
  if (!vec_extend_from_slice(out_packet, to_bytes, 4)) return false;
  // Add 'from' field
  uint8_t from_bytes[4];
  write_u32_le(from_bytes, from);
  if (!vec_extend_from_slice(out_packet, from_bytes, 4)) return false;
  // Add 'id' field
  uint8_t id_bytes[4];
  write_u32_le(id_bytes, id);
  if (!vec_extend_from_slice(out_packet, id_bytes, 4)) return false;
  // Build and add flags
  uint8_t flags = (want_ack ? FLAG_WANT_ACK : 0) | ((hop_limit & 0x07) << FLAG_HOP_LIMIT_SHIFT) | ((channel & 0x0F) << FLAG_CHANNEL_SHIFT);
  if (!vec_push(out_packet, flags)) return false;
  // Add channel hash
  if (!vec_push(out_packet, compute_channel_hash(channel))) return false;
  // Add two reserved bytes
  if (!vec_push(out_packet, 0)) return false;
  if (!vec_push(out_packet, 0)) return false;
  // Add payload
  if (!vec_extend_from_slice(out_packet, payload, payload_len)) return false;
  // Compute and add MIC
  uint8_t mic[MIC_SIZE];
  compute_mic(out_packet->data, out_packet->len, mic);
  if (!vec_extend_from_slice(out_packet, mic, MIC_SIZE)) return false;
  return true;
}

// PacketCache structure and functions
#define PACKET_CACHE_SIZE 32

typedef struct 
{
  uint32_t entries[PACKET_CACHE_SIZE][3]; // [from, packet_id, timestamp]
  size_t index;
} PacketCache;

// Initialize PacketCache
void packet_cache_init(PacketCache* cache) 
{
  memset(cache->entries, 0, sizeof(cache->entries));
  cache->index = 0;
}

// Check if packet is duplicate
bool packet_cache_is_duplicate(const PacketCache* cache, uint32_t from, uint32_t packet_id) 
{
  for (size_t i = 0; i < PACKET_CACHE_SIZE; i++) 
  {
    uint32_t f = cache->entries[i][0];
    uint32_t id = cache->entries[i][1];
    if (f == from && id == packet_id && f != 0) return true;
  }
  return false;
}

// Add packet to cache
void packet_cache_add(PacketCache* cache, uint32_t from, uint32_t packet_id, uint32_t timestamp) 
{
  cache->entries[cache->index][0] = from;
  cache->entries[cache->index][1] = packet_id;
  cache->entries[cache->index][2] = timestamp;
  cache->index = (cache->index + 1) % PACKET_CACHE_SIZE;
}

// Check if duplicate and add if not
bool packet_cache_check_and_add(PacketCache* cache, uint32_t from, uint32_t packet_id, uint32_t timestamp) 
{
  if (packet_cache_is_duplicate(cache, from, packet_id)) return true;
  packet_cache_add(cache, from, packet_id, timestamp);
  return false;
}

// Clear old entries from cache
void packet_cache_clear_old(PacketCache* cache, uint32_t current_time, uint32_t max_age) 
{
  for (size_t i = 0; i < PACKET_CACHE_SIZE; i++) 
  {
    if (cache->entries[i][0] != 0) 
    {
      uint32_t entry_time = cache->entries[i][2];
      uint32_t age = current_time > entry_time ? current_time - entry_time : 0;
      if (age > max_age) 
      {
        cache->entries[i][0] = 0;
        cache->entries[i][1] = 0;
        cache->entries[i][2] = 0;
      }
    }
  }
}

// RoutingDecision enum
typedef enum 
{
  ROUTING_DECISION_LOCAL,    // Packet is for us
  ROUTING_DECISION_FORWARD,  // Forward the packet
  ROUTING_DECISION_DROP      // Drop the packet
} RoutingDecision;

// Route packet and determine what to do with it
RoutingDecision route_packet(const MeshPacket* packet,uint32_t our_node_id,PacketCache* cache,uint32_t timestamp) 
{
  // Check if this is a duplicate
  if (packet_cache_check_and_add(cache, packet->from, packet->id, timestamp)) return ROUTING_DECISION_DROP;
  bool is_broadcast = (packet->to == 0xFFFFFFFF);
  bool is_for_us = (packet->to == our_node_id);
  if (is_for_us || is_broadcast) 
  {
      // If it's a broadcast and has hops left, forward it
      if (is_broadcast && packet->hop_limit > 0) return ROUTING_DECISION_FORWARD;
      return ROUTING_DECISION_LOCAL;
  }
  // Not for us - forward if hops remain
  if (packet->hop_limit > 0) return ROUTING_DECISION_FORWARD;
  else return ROUTING_DECISION_DROP;
}

// Create forward packet with decremented hop limit
bool create_forward_packet(const MeshPacket* original,const uint8_t* payload,size_t payload_len,Vec* out_packet) 
{
  if (original->hop_limit == 0) return false;
  return build_lora_packet(original->from,original->to,original->id,original->channel,original->hop_limit - 1,
                            original->want_ack,payload,payload_len,out_packet);
}

// PacketStats structure
typedef struct 
{
  uint32_t rx_total;          // Total packets received
  uint32_t rx_local;          // Packets for us
  uint32_t rx_forwarded;      // Packets forwarded
  uint32_t rx_dropped_dup;    // Packets dropped (duplicate)
  uint32_t rx_dropped_expired;// Packets dropped (expired)
  uint32_t rx_bad;            // Bad packets
  uint32_t tx_total;          // Total packets transmitted
  uint32_t tx_retransmit;     // Retransmission count
  uint32_t tx_fail;           // Failed transmissions
} PacketStats;

// Initialize PacketStats
void packet_stats_init(PacketStats* stats) 
{
  stats->rx_total = 0;
  stats->rx_local = 0;
  stats->rx_forwarded = 0;
  stats->rx_dropped_dup = 0;
  stats->rx_dropped_expired = 0;
  stats->rx_bad = 0;
  stats->tx_total = 0;
  stats->tx_retransmit = 0;
  stats->tx_fail = 0;
}

// Record received packet
void packet_stats_record_rx(PacketStats* stats, RoutingDecision decision) 
{
  stats->rx_total++;
  switch (decision) 
  {
    case ROUTING_DECISION_LOCAL:
    stats->rx_local++;
    break;
        
    case ROUTING_DECISION_FORWARD:
    stats->rx_forwarded++;
    break;
        
    case ROUTING_DECISION_DROP:
    stats->rx_dropped_dup++;
    break;
  }
}

// Record bad packet
void packet_stats_record_rx_bad(PacketStats* stats) 
{
  stats->rx_total++;
  stats->rx_bad++;
}

// Record transmission
void packet_stats_record_tx(PacketStats* stats, bool success) 
{
  stats->tx_total++;
  if (!success) stats->tx_fail++;
}

// Record retransmission
void packet_stats_record_retransmit(PacketStats* stats) 
{
  stats->tx_retransmit++;
}

// PendingAck structure
typedef struct 
{
  uint32_t packet_id;           // Packet ID
  uint32_t to;                  // Destination node
  Vec packet_data;              // Packet data for retransmission
  uint8_t tx_count;             // Transmission count
  uint8_t max_retransmit;       // Maximum retransmission attempts
  uint32_t last_tx_time;        // Last transmission time
  uint32_t retransmit_timeout;  // Timeout for retransmission
} PendingAck;

// AckTracker structure
#define ACK_TRACKER_SIZE 8

typedef struct 
{
  PendingAck pending[ACK_TRACKER_SIZE];
  bool pending_valid[ACK_TRACKER_SIZE]; // Track which slots are valid
} AckTracker;

// Initialize AckTracker
void ack_tracker_init(AckTracker* tracker) 
{
  memset(tracker->pending, 0, sizeof(tracker->pending));
  memset(tracker->pending_valid, 0, sizeof(tracker->pending_valid));
  for (size_t i = 0; i < ACK_TRACKER_SIZE; i++) vec_init(&tracker->pending[i].packet_data);
}

// Add pending ACK
bool ack_tracker_add(AckTracker* tracker, const PendingAck* ack) 
{
  for (size_t i = 0; i < ACK_TRACKER_SIZE; i++) 
  {
    if (!tracker->pending_valid[i]) 
    {
      tracker->pending[i] = *ack;
      tracker->pending_valid[i] = true;
      return true;
    }
  }
  return false;
}

// Check if ACK is pending
bool ack_tracker_is_pending(const AckTracker* tracker, uint32_t to, uint32_t packet_id) 
{
  for (size_t i = 0; i < ACK_TRACKER_SIZE; i++) 
  {
    if (tracker->pending_valid[i]) 
    {
      const PendingAck* ack = &tracker->pending[i];
      if (ack->to == to && ack->packet_id == packet_id) return true;
    }
  }
  return false;
}

// Handle received ACK
bool ack_tracker_handle_ack(AckTracker* tracker, uint32_t from, uint32_t request_id) 
{
  for (size_t i = 0; i < ACK_TRACKER_SIZE; i++) 
  {
    if (tracker->pending_valid[i]) 
    {
      const PendingAck* ack = &tracker->pending[i];
      if (ack->to == from && ack->packet_id == request_id) 
      {
        tracker->pending_valid[i] = false;
        return true;
      }
    }
  }
  return false;
}

// Get packet to retransmit
bool ack_tracker_get_retransmit(AckTracker* tracker, uint32_t current_time, Vec* out_packet) 
{
  for (size_t i = 0; i < ACK_TRACKER_SIZE; i++) 
  {
    if (tracker->pending_valid[i]) 
    {
      PendingAck* ack = &tracker->pending[i];
      uint32_t elapsed = current_time > ack->last_tx_time ? current_time - ack->last_tx_time : 0;
      if (elapsed >= ack->retransmit_timeout) 
      {
        if (ack->tx_count < ack->max_retransmit) 
        {
          ack->tx_count++;
          ack->last_tx_time = current_time;
          // Clone packet data
          *out_packet = ack->packet_data;
          return true;
        } 
        else tracker->pending_valid[i] = false;// Max retransmits reached, clear the slot
      }
    }
  }
  return false;
}

// Clear all pending ACKs
void ack_tracker_clear(AckTracker* tracker) 
{
  memset(tracker->pending_valid, 0, sizeof(tracker->pending_valid));
}

// Get count of pending ACKs
size_t ack_tracker_pending_count(const AckTracker* tracker) 
{
  size_t count = 0;
  for (size_t i = 0; i < ACK_TRACKER_SIZE; i++) 
  {
    if (tracker->pending_valid[i]) count++;
  }
  return count;
}

/* // SHA256 implementation (simple placeholder - replace with actual implementation)
void sha256_hash(const uint8_t* data, size_t len, uint8_t* hash) 
{
    // This is a placeholder implementation
    // In a real implementation, this should use a proper SHA256 library
    // For now, we'll create a simple hash that matches the interface
    
    // Initialize hash with a pattern
    for (size_t i = 0; i < 32; i++) {
        hash[i] = (uint8_t)(i * 7);
    }
    
    // Simple mixing based on input data
    for (size_t i = 0; i < len; i++) {
        hash[i % 32] ^= data[i];
        hash[(i + 1) % 32] = (uint8_t)((hash[(i + 1) % 32] + data[i]) * 0x9E);
        hash[(i + 2) % 32] ^= (uint8_t)(hash[(i + 2) % 32] >> 4);
    }
    
    // Additional mixing passes
    for (int pass = 0; pass < 4; pass++) {
        for (size_t i = 0; i < 32; i++) {
            hash[i] = (uint8_t)(hash[i] ^ (hash[(i + 7) % 32] + hash[(i + 13) % 32]));
        }
    }
} */
