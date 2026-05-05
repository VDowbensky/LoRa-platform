#include <transport.h>


/* --- Dependencies / Standard Library Mocking --- */

/**
 * Since the original code relies on a Sha256 implementation from the crate, 
 * we define the interface here. The implementation must be provided 
 * by the user or linked from a crypto library.
 */
typedef struct 
{
  uint8_t hash[32];
} Sha256Result;

void Sha256_hash(const uint8_t *data, size_t len, uint8_t out[32]);

/* --- Constants --- */

#define MAX_PACKET_SIZE 237
#define NODE_HINT_SIZE 2
#define SESSION_HINT_SIZE 4
#define AUTH_TAG_SIZE 16
#define FLAGS_SIZE 1
#define DATA_OVERHEAD (FLAGS_SIZE + NODE_HINT_SIZE + SESSION_HINT_SIZE + AUTH_TAG_SIZE)
#define DATA_MAX_PAYLOAD (MAX_PACKET_SIZE - DATA_OVERHEAD)
#define PADDED_MESSAGE_SIZE 200

/* --- HeaplessVec Equivalents --- */

typedef struct 
{
  uint8_t data[128];
  size_t len;
} Vec128;

typedef struct 
{
  uint8_t data[214];
  size_t len;
} Vec214;

typedef struct {
    uint8_t data[237];
    size_t len;
} Vec237;

typedef struct 
{
  uint8_t data[32];
  size_t len;
} Vec32;

typedef struct 
{
  uint8_t data[16];
  size_t len;
} Vec16;

/* --- UniversalAddress --- */

typedef struct 
{
  Vec128 did;
  uint8_t public_key[32];
  uint16_t meshcore_addr;
  uint32_t meshtastic_id;
  uint8_t reticulum_hash[16];
} UniversalAddress;

/* --- AddressTranslator --- */

uint16_t AddressTranslator_derive_meshcore_address(const uint8_t public_key[32]) 
{
  uint8_t hash[32];
  Sha256_hash(public_key, 32, hash);
  return ((uint16_t)hash[0] << 8) | (uint16_t)hash[1];
}

uint32_t AddressTranslator_derive_meshtastic_id(const uint8_t public_key[32]) 
{
  uint8_t hash[32];
  Sha256_hash(public_key, 32, hash);
  return ((uint32_t)hash[0] << 24)
           | ((uint32_t)hash[1] << 16)
           | ((uint32_t)hash[2] << 8)
           | (uint32_t)hash[3];
}

void AddressTranslator_derive_reticulum_hash(const uint8_t public_key[32], const uint8_t *app_name, size_t app_name_len, uint8_t out[16]) 
{
  uint8_t app_hash[32];
  Sha256_hash(app_name, app_name_len, app_hash);
  uint8_t combined[64];
  memcpy(&combined[0], app_hash, 32);
  memcpy(&combined[32], public_key, 32);
  uint8_t full_hash[32];
  Sha256_hash(combined, 64, full_hash);
  memcpy(out, full_hash, 16);
}

UniversalAddress AddressTranslator_from_public_key(const uint8_t public_key[32]) 
{
  UniversalAddress addr;
  uint8_t pubkey_hash[32];
  Sha256_hash(public_key, 32, pubkey_hash);
  addr.meshcore_addr = ((uint16_t)pubkey_hash[0] << 8) | (uint16_t)pubkey_hash[1];
  addr.meshtastic_id = ((uint32_t)pubkey_hash[0] << 24)
                         | ((uint32_t)pubkey_hash[1] << 16)
                         | ((uint32_t)pubkey_hash[2] << 8)
                         | (uint32_t)pubkey_hash[3];
  const char *app_str = "yours.messaging";
  uint8_t app_hash[32];
  Sha256_hash((const uint8_t*)app_str, 15, app_hash);
  uint8_t combined[64];
  memcpy(&combined[0], app_hash, 32);
  memcpy(&combined[32], public_key, 32);
  uint8_t reticulum_full[32];
  Sha256_hash(combined, 64, reticulum_full);
  memcpy(addr.reticulum_hash, reticulum_full, 16);
  const char *did_prefix = "did:offgrid:z";
  addr.did.len = 13; // length of "did:offgrid:z"
  memcpy(addr.did.data, did_prefix, addr.did.len);
  memcpy(addr.public_key, public_key, 32);
  return addr;
}

/* --- PacketType --- */

typedef enum 
{
  PacketType_Data = 0x00,      // 0b00
  PacketType_Handshake = 0x01, // 0b01
  PacketType_Control = 0x02,   // 0b10
  PacketType_Cover = 0x03      // 0b11
} PacketType;

PacketType PacketType_from_flags(uint8_t flags) 
{
  uint8_t val = (flags >> 6) & 0x03;
  switch (val) 
  {
    case 0x00: return PacketType_Data;
    case 0x01: return PacketType_Handshake;
    case 0x02: return PacketType_Control;
    case 0x03: return PacketType_Cover;
    default: return PacketType_Data; // Should be unreachable given & 0x03
  }
}

/* --- WirePacket --- */

typedef struct 
{
  PacketType packet_type;
  uint8_t hop_count;
  uint16_t next_hop_hint;
  uint32_t session_hint;
  Vec214 payload;
} WirePacket;

bool WirePacket_new_data(WirePacket *out, uint16_t next_hop, uint32_t session, const uint8_t *payload, size_t payload_len) 
{
  if (payload_len > DATA_MAX_PAYLOAD) return false;
  out->packet_type = PacketType_Data;
  out->hop_count = 0;
  out->next_hop_hint = next_hop;
  out->session_hint = session;
  out->payload.len = payload_len;
  memcpy(out->payload.data, payload, payload_len);
  return true;
}

Vec237 WirePacket_encode(const WirePacket *self) 
{
  Vec237 buf;
  buf.len = 0;
  uint8_t flags = (((uint8_t)self->packet_type) << 6) | (self->hop_count & 0x0F);
  buf.data[buf.len++] = flags;
  buf.data[buf.len++] = (uint8_t)(self->next_hop_hint >> 8);
  buf.data[buf.len++] = (uint8_t)(self->next_hop_hint);
  buf.data[buf.len++] = (uint8_t)(self->session_hint >> 24);
  buf.data[buf.len++] = (uint8_t)(self->session_hint >> 16);
  buf.data[buf.len++] = (uint8_t)(self->session_hint >> 8);
  buf.data[buf.len++] = (uint8_t)(self->session_hint);
  if (self->payload.len > 0) 
  {
    memcpy(&buf.data[buf.len], self->payload.data, self->payload.len);
    buf.len += self->payload.len;
  }
  return buf;
}

bool WirePacket_decode(WirePacket *out, const uint8_t *data, size_t data_len) 
{
  if (data_len < 7) return false;
  uint8_t flags = data[0];
  out->packet_type = PacketType_from_flags(flags);
  out->hop_count = flags & 0x0F;
  out->next_hop_hint = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
  out->session_hint = ((uint32_t)data[3] << 24)
                         | ((uint32_t)data[4] << 16)
                         | ((uint32_t)data[5] << 8)
                         | (uint32_t)data[6];
  out->payload.len = 0;
  if (data_len > 7) 
  {
    size_t payload_len = data_len - 7;
    if (payload_len > 214) payload_len = 214; // Boundary check for heapless equivalent
    memcpy(out->payload.data, &data[7], payload_len);
    out->payload.len = payload_len;
  }
  return true;
}

bool WirePacket_increment_hop(WirePacket *self) 
{
  if (self->hop_count < 15) 
  {
    self->hop_count += 1;
    return true;
  } 
  else return false;
}

/* --- MessagePriority --- */

typedef enum 
{
  MessagePriority_Low = 0,
  MessagePriority_Normal = 1,
  MessagePriority_High = 2,
  MessagePriority_Critical = 3
} MessagePriority;

/* --- UniversalMessage --- */

typedef struct 
{
  uint8_t id[8];
  UniversalAddress recipient;
  Vec237 payload;
  MessagePriority priority;
  uint64_t timestamp;
} UniversalMessage;

/* --- ConnectionState --- */

typedef enum 
{
  ConnectionState_Disconnected,
  ConnectionState_Connecting,
  ConnectionState_Connected,
  ConnectionState_Error
} ConnectionState;

/* --- SignalQuality --- */

typedef struct 
{
  int16_t rssi;
  int8_t snr;
  uint8_t quality;
} SignalQuality;

SignalQuality SignalQuality_new(int16_t rssi, int8_t snr) 
{
  SignalQuality sq;
  sq.rssi = rssi;
  sq.snr = snr;
  int16_t rssi_clamped = rssi;
  if (rssi_clamped < -120) rssi_clamped = -120;
  if (rssi_clamped > -50) rssi_clamped = -50;
  uint8_t rssi_norm = (uint8_t)(((uint16_t)(rssi_clamped + 120) * 100) / 70);
  int16_t snr_clamped = (int16_t)snr;
  if (snr_clamped < -20) snr_clamped = -20;
  if (snr_clamped > 10) snr_clamped = 10;
  uint8_t snr_norm = (uint8_t)(((uint16_t)(snr_clamped + 20) * 100) / 30);
  sq.quality = (rssi_norm + snr_norm) / 2;
  return sq;
}

/* --- MeshDeviceInfo --- */

typedef struct 
{
  bool meshcore;
  bool meshtastic;
  bool reticulum;
} ProtocolSupport;

typedef struct 
{
  Vec32 name;
  Vec16 firmware_version;
  Vec32 hardware_model;
  ProtocolSupport protocols;
  uint8_t battery_level;
} MeshDeviceInfo;

/* --- TransportError --- */

typedef enum 
{
  TransportError_NotConnected,
  TransportError_ConnectionFailed,
  TransportError_SendFailed,
  TransportError_ReceiveTimeout,
  TransportError_InvalidMessage,
  TransportError_BufferOverflow,
  TransportError_DeviceBusy,
  TransportError_ProtocolError,
  TransportError_Unknown
} TransportError;

/* --- UniversalMeshTransport (Interface via VTable) --- */

typedef struct UniversalMeshTransport UniversalMeshTransport;

struct UniversalMeshTransportVTable 
{
  TransportError (*connect)(UniversalMeshTransport *self);
  void (*disconnect)(UniversalMeshTransport *self);
  TransportError (*send_message)(UniversalMeshTransport *self, const UniversalMessage *message, uint8_t out_id[8]);
  bool (*poll_message)(UniversalMeshTransport *self, UniversalMessage *out_message);
  TransportError (*get_device_info)(const UniversalMeshTransport *self, MeshDeviceInfo *out_info);
  ConnectionState (*connection_state)(const UniversalMeshTransport *self);
  SignalQuality (*signal_quality)(const UniversalMeshTransport *self);
  size_t (*discover_peers)(UniversalMeshTransport *self, uint32_t timeout_ms, UniversalAddress *out_addrs, size_t max_addrs);
  TransportError (*ping_peer)(UniversalMeshTransport *self, const UniversalAddress *address, uint32_t *out_latency);
};

struct UniversalMeshTransport 
{
  const struct UniversalMeshTransportVTable *vtable;
  void *implementation_data;
};

/* --- Protocol --- */

typedef enum 
{
  Protocol_MeshCore,
  Protocol_Meshtastic,
  Protocol_Reticulum
} Protocol;

void Protocol_magic_bytes(Protocol self, uint8_t out[2]) 
{
  switch (self) 
  {
    case Protocol_MeshCore:
    out[0] = 0xAA; out[1] = 0x55;
    break;
    
    case Protocol_Meshtastic:
    out[0] = 0x94; out[1] = 0xC3;
    break;
    
    case Protocol_Reticulum:
    out[0] = 0xC0; out[1] = 0x00;
    break;
  }
}

bool Protocol_detect(const uint8_t *data, size_t len, Protocol *out_protocol) {
  if (len < 2) return false;
  if (data[0] == 0xAA && data[1] == 0x55) 
  {
    *out_protocol = Protocol_MeshCore;
    return true;
  }
  if (data[0] == 0x94 && data[1] == 0xC3) 
  {
    *out_protocol = Protocol_Meshtastic;
    return true;
  }
  if (data[0] == 0xC0) 
  {
    *out_protocol = Protocol_Reticulum;
    return true;
  }
  // (b'A', b'T') => None handling implicit
  return false;
}

/* --- AddressLookupTable --- */

#define MAX_KNOWN_ADDRESSES 64

typedef enum 
{
  ProtocolAddressType_MeshCore,
  ProtocolAddressType_Meshtastic,
  ProtocolAddressType_Reticulum
} ProtocolAddressType;

typedef struct 
{
  ProtocolAddressType type;
  union 
  {
    uint16_t meshcore;
    uint32_t meshtastic;
    uint8_t reticulum[16];
  } value;
} ProtocolAddress;

typedef struct 
{
  struct 
  {
    uint16_t key;
    uint8_t value[32];
    bool occupied;
  } meshcore_index[MAX_KNOWN_ADDRESSES];

  struct 
  {
    uint32_t key;
    uint8_t value[32];
    bool occupied;
  } meshtastic_index[MAX_KNOWN_ADDRESSES];

  struct 
  {
    uint8_t key[8];
    uint8_t value[32];
    bool occupied;
  } reticulum_index[MAX_KNOWN_ADDRESSES];
  
  size_t count;
} AddressLookupTable;

void AddressLookupTable_init(AddressLookupTable *self) 
{
  memset(self, 0, sizeof(AddressLookupTable));
}

void AddressLookupTable_register(AddressLookupTable *self, const uint8_t public_key[32]) 
{
  UniversalAddress addr = AddressTranslator_from_public_key(public_key);
  // Meshcore Index (Linear search for empty/existing slot as simple FnvIndexMap equivalent)
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    if (!self->meshcore_index[i].occupied || self->meshcore_index[i].key == addr.meshcore_addr) 
    {
      if (!self->meshcore_index[i].occupied) self->count++;
      self->meshcore_index[i].key = addr.meshcore_addr;
      memcpy(self->meshcore_index[i].value, public_key, 32);
      self->meshcore_index[i].occupied = true;
      break;
    }
  }
  // Meshtastic Index
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    if (!self->meshtastic_index[i].occupied || self->meshtastic_index[i].key == addr.meshtastic_id) 
    {
      self->meshtastic_index[i].key = addr.meshtastic_id;
      memcpy(self->meshtastic_index[i].value, public_key, 32);
      self->meshtastic_index[i].occupied = true;
      break;
    }
  }
  // Reticulum Index
  uint8_t ret_key[8];
  memcpy(ret_key, addr.reticulum_hash, 8);
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    bool match = self->reticulum_index[i].occupied && (memcmp(self->reticulum_index[i].key, ret_key, 8) == 0);
    if (!self->reticulum_index[i].occupied || match) 
    {
      memcpy(self->reticulum_index[i].key, ret_key, 8);
      memcpy(self->reticulum_index[i].value, public_key, 32);
      self->reticulum_index[i].occupied = true;
      break;
    }
  }
}

void AddressLookupTable_unregister(AddressLookupTable *self, const uint8_t public_key[32]) 
{
  UniversalAddress addr = AddressTranslator_from_public_key(public_key);
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    if (self->meshcore_index[i].occupied && self->meshcore_index[i].key == addr.meshcore_addr) 
    {
      self->meshcore_index[i].occupied = false;
      self->count--;
      break;
    }
  }
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    if (self->meshtastic_index[i].occupied && self->meshtastic_index[i].key == addr.meshtastic_id) 
    {
      self->meshtastic_index[i].occupied = false;
      break;
    }
  }
  uint8_t ret_key[8];
  memcpy(ret_key, addr.reticulum_hash, 8);
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    if (self->reticulum_index[i].occupied && memcmp(self->reticulum_index[i].key, ret_key, 8) == 0) 
    {
      self->reticulum_index[i].occupied = false;
      break;
    }
  }
}

const uint8_t* AddressLookupTable_lookup_meshcore(const AddressLookupTable *self, uint16_t addr) 
{
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    if (self->meshcore_index[i].occupied && self->meshcore_index[i].key == addr) return self->meshcore_index[i].value;
  }
  return NULL;
}

const uint8_t* AddressLookupTable_lookup_meshtastic(const AddressLookupTable *self, uint32_t id) 
{
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    if (self->meshtastic_index[i].occupied && self->meshtastic_index[i].key == id) return self->meshtastic_index[i].value;
  }
  return NULL;
}

const uint8_t* AddressLookupTable_lookup_reticulum(const AddressLookupTable *self, const uint8_t hash[16]) 
{
  uint8_t key[8];
  memcpy(key, hash, 8);
  for (int i = 0; i < MAX_KNOWN_ADDRESSES; i++) 
  {
    if (self->reticulum_index[i].occupied && memcmp(self->reticulum_index[i].key, key, 8) == 0) return self->reticulum_index[i].value;
  }
    return NULL;
}

const uint8_t* AddressLookupTable_lookup(const AddressLookupTable *self, ProtocolAddress addr) 
{
  switch (addr.type) 
  {
    case ProtocolAddressType_MeshCore:
    return AddressLookupTable_lookup_meshcore(self, addr.value.meshcore);
    
    case ProtocolAddressType_Meshtastic:
    return AddressLookupTable_lookup_meshtastic(self, addr.value.meshtastic);
    
    case ProtocolAddressType_Reticulum:
    return AddressLookupTable_lookup_reticulum(self, addr.value.reticulum);
    
    default:
    return NULL;
  }
}

size_t AddressLookupTable_len(const AddressLookupTable *self) 
{
  return self->count;
}

bool AddressLookupTable_is_empty(const AddressLookupTable *self) 
{
  return self->count == 0;
}
