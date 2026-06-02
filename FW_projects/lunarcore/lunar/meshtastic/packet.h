
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

// PacketCache structure and functions
#define PACKET_CACHE_SIZE 32

typedef struct 
{
  uint32_t entries[PACKET_CACHE_SIZE][3]; // [from, packet_id, timestamp]
  size_t index;
} PacketCache;

// RoutingDecision enum
typedef enum 
{
  ROUTING_DECISION_LOCAL,    // Packet is for us
  ROUTING_DECISION_FORWARD,  // Forward the packet
  ROUTING_DECISION_DROP      // Drop the packet
} RoutingDecision;

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

