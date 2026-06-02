#ifndef _PROTOCOL_ROUTER_H_
#define _PROTOCOL_ROUTER_H_

// Protocol enum definition
typedef enum 
{
  PROTOCOL_UNKNOWN,
  PROTOCOL_MESHCORE,
  PROTOCOL_MESHTASTIC,
  PROTOCOL_RNODE,
  PROTOCOL_ATCOMMAND
} Protocol;

// Magic constants module
#define MESHCORE_SYNC1 0xAA
#define MESHCORE_SYNC2 0x55
#define MESHTASTIC_SYNC1 0x94
#define MESHTASTIC_SYNC2 0xC3
#define KISS_FEND 0xC0
static const uint8_t AT_PREFIX[2] = {'A', 'T'};

// DetectState enum definition
typedef enum 
{
  DETECT_STATE_IDLE,
  DETECT_STATE_MESHCORE1,
  DETECT_STATE_MESHTASTIC1,
  DETECT_STATE_AT1
} DetectState;

// Sync timeout constant
#define SYNC_TIMEOUT_BYTES 256

// ProtocolDetector structure
typedef struct 
{
  DetectState state;
  Protocol detected;
  uint16_t bytes_seen;
  uint8_t lock_threshold;
  uint8_t lock_count;
  uint16_t state_bytes;
  uint32_t last_detect_ms;
} ProtocolDetector;

// ProtocolDetector constructor
ProtocolDetector protocol_detector_new(void) 
{
  ProtocolDetector detector;
  detector.state = DETECT_STATE_IDLE;
  detector.detected = PROTOCOL_UNKNOWN;
  detector.bytes_seen = 0;
  detector.lock_threshold = 3;
  detector.lock_count = 0;
  detector.state_bytes = 0;
  detector.last_detect_ms = 0;
  return detector;
}

// Option type for Protocol
typedef struct 
{
  bool has_value;
  Protocol value;
} OptionProtocol;

// Maximum transports constant
#define MAX_TRANSPORTS 3

// TransportType enum definition
typedef enum 
{
  TRANSPORT_TYPE_USB_SERIAL,
  TRANSPORT_TYPE_BLE,
  TRANSPORT_TYPE_WIFI
} TransportType;

// TransportState structure
typedef struct 
{
  TransportType transport;
  ProtocolDetector detector;
  bool active;
} TransportState;

// Option type for TransportType
typedef struct 
{
  bool has_value;
  TransportType value;
} OptionTransportType;

// ProtocolRouter structure
typedef struct 
{
  TransportState transports[MAX_TRANSPORTS];
  Protocol lora_protocol;
  bool lora_shared;
  OptionTransportType priority_transport;
} ProtocolRouter;

// Status tuple structure
typedef struct 
{
  TransportType transport;
  Protocol protocol;
  bool active;
} TransportStatus;

// Vec implementation for u8 with capacity 256
typedef struct 
{
  uint8_t data[256];
  size_t len;
} VecU8_256;

// Vec implementation for u8 with capacity 237
typedef struct 
{
  uint8_t data[237];
  size_t len;
} VecU8_237;

// LoRaPacket structure
typedef struct 
{
  Protocol protocol;
  VecU8_256 data;
  int16_t rssi;
  int8_t snr;
} LoRaPacket;

// UnifiedPacket structure
typedef struct 
{
  Protocol source_protocol;
  Protocol dest_protocol;
  VecU8_237 payload;
  uint8_t source_addr[32];
  uint8_t dest_addr[32];
  uint8_t hops;
  int16_t rssi;
  int8_t snr;
} UnifiedPacket;



#endif
