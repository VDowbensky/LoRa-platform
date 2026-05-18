#include"protocol_router.h"



// Function to get protocol name
const char* protocol_name(Protocol protocol) 
{
  switch (protocol) 
  {
    case PROTOCOL_UNKNOWN:
    return "Unknown";
    
    case PROTOCOL_MESHCORE:
    return "MeshCore";
    
    case PROTOCOL_MESHTASTIC:
    return "Meshtastic";
    
    case PROTOCOL_RNODE:
    return "RNode/KISS";
    
    case PROTOCOL_ATCOMMAND:
    return "AT Command";
    
    default:
    return "Unknown";
  }
}



// ProtocolDetector reset method
void protocol_detector_reset(ProtocolDetector* self) 
{
  self->state = DETECT_STATE_IDLE;
  self->detected = PROTOCOL_UNKNOWN;
  self->bytes_seen = 0;
  self->lock_count = 0;
  self->state_bytes = 0;
}

// ProtocolDetector soft_reset method
void protocol_detector_soft_reset(ProtocolDetector* self) 
{
  self->state = DETECT_STATE_IDLE;
  self->state_bytes = 0;
}

// ProtocolDetector force_protocol method
void protocol_detector_force_protocol(ProtocolDetector* self, Protocol protocol) 
{
  self->detected = protocol;
  self->state = DETECT_STATE_IDLE;
}

// ProtocolDetector protocol getter
Protocol protocol_detector_protocol(const ProtocolDetector* self) 
{
  return self->detected;
}

// ProtocolDetector is_locked method
bool protocol_detector_is_locked(const ProtocolDetector* self) 
{
  return self->lock_count >= self->lock_threshold;
}

// ProtocolDetector confirm_frame method
void protocol_detector_confirm_frame(ProtocolDetector* self) 
{
  if (self->lock_count < 255) self->lock_count += 1;
}

// ProtocolDetector error_frame method
void protocol_detector_error_frame(ProtocolDetector* self) 
{
  if (self->lock_count > 0) 
  {
    uint8_t new_count = self->lock_count - 2;
    if (new_count > self->lock_count) self->lock_count = 0;// Check for underflow (saturating_sub behavior)
    else self->lock_count = new_count;
  }
  if (self->lock_count == 0 && self->detected != PROTOCOL_UNKNOWN) self->detected = PROTOCOL_UNKNOWN;
}

// ProtocolDetector check_timeout helper method
static void protocol_detector_check_timeout(ProtocolDetector* self) 
{
  if (self->state != DETECT_STATE_IDLE && self->state_bytes > SYNC_TIMEOUT_BYTES) 
  {
    self->state = DETECT_STATE_IDLE;
    self->state_bytes = 0;
  }
}


// Helper to create None option
static OptionProtocol option_protocol_none(void) 
{
  OptionProtocol opt;
  opt.has_value = false;
  opt.value = PROTOCOL_UNKNOWN;
  return opt;
}

// Helper to create Some option
static OptionProtocol option_protocol_some(Protocol value) 
{
  OptionProtocol opt;
  opt.has_value = true;
  opt.value = value;
  return opt;
}

// ProtocolDetector feed method
OptionProtocol protocol_detector_feed(ProtocolDetector* self, uint8_t byte) 
{
  self->bytes_seen += 1;
  self->state_bytes += 1;
  protocol_detector_check_timeout(self);
  if (protocol_detector_is_locked(self)) return option_protocol_some(self->detected);
  DetectState prev_state = self->state;
  switch (self->state) 
  {
    case DETECT_STATE_IDLE:
    switch (byte) 
    {
      case MESHCORE_SYNC1:
      self->state = DETECT_STATE_MESHCORE1;
      break;
      
      case MESHTASTIC_SYNC1:
      self->state = DETECT_STATE_MESHTASTIC1;
      break;
      
      case KISS_FEND:
      if (self->detected == PROTOCOL_UNKNOWN || self->detected == PROTOCOL_RNODE) 
      {
        self->detected = PROTOCOL_RNODE;
        return option_protocol_some(PROTOCOL_RNODE);
      }
      break;
      
      case 'A':
      self->state = DETECT_STATE_AT1;
      break;
      
      default:
      // Do nothing
      break;
    }
    break;

    case DETECT_STATE_MESHCORE1:
    if (byte == MESHCORE_SYNC2) 
    {
      self->detected = PROTOCOL_MESHCORE;
      self->state = DETECT_STATE_IDLE;
      return option_protocol_some(PROTOCOL_MESHCORE);
    } 
    else if (byte == MESHCORE_SYNC1) 
    {
      // Stay in MeshCore1 state
    } 
    else self->state = DETECT_STATE_IDLE;
    break;

    case DETECT_STATE_MESHTASTIC1:
    if (byte == MESHTASTIC_SYNC2) 
    {
      self->detected = PROTOCOL_MESHTASTIC;
      self->state = DETECT_STATE_IDLE;
      return option_protocol_some(PROTOCOL_MESHTASTIC);
    } 
    else self->state = DETECT_STATE_IDLE;
    break;

    case DETECT_STATE_AT1:
    if (byte == 'T') 
    {
      self->detected = PROTOCOL_ATCOMMAND;
      self->state = DETECT_STATE_IDLE;
      return option_protocol_some(PROTOCOL_ATCOMMAND);
    } 
    else self->state = DETECT_STATE_IDLE;
    break;
  }
  if (self->state != prev_state) self->state_bytes = 0;
  return option_protocol_none();
}



// TransportState constructor
TransportState transport_state_new(TransportType transport) 
{
  TransportState state;
  state.transport = transport;
  state.detector = protocol_detector_new();
  state.active = false;
  return state;
}


// Helper to create None option for TransportType
static OptionTransportType option_transport_type_none(void) 
{
  OptionTransportType opt;
  opt.has_value = false;
  opt.value = TRANSPORT_TYPE_USB_SERIAL;
  return opt;
}

// Helper to create Some option for TransportType
static OptionTransportType option_transport_type_some(TransportType value) 
{
  OptionTransportType opt;
  opt.has_value = true;
  opt.value = value;
  return opt;
}


// ProtocolRouter constructor
ProtocolRouter protocol_router_new(void) 
{
  ProtocolRouter router;
  router.transports[0] = transport_state_new(TRANSPORT_TYPE_USB_SERIAL);
  router.transports[1] = transport_state_new(TRANSPORT_TYPE_BLE);
  router.transports[2] = transport_state_new(TRANSPORT_TYPE_WIFI);
  router.lora_protocol = PROTOCOL_UNKNOWN;
  router.lora_shared = true;
  router.priority_transport = option_transport_type_none();
  return router;
}

// ProtocolRouter transport method (mutable)
TransportState* protocol_router_transport(ProtocolRouter* self, TransportType t) 
{
  switch (t) 
  {
    case TRANSPORT_TYPE_USB_SERIAL:
    return &self->transports[0];
    
    case TRANSPORT_TYPE_BLE:
    return &self->transports[1];
    
    case TRANSPORT_TYPE_WIFI:
    return &self->transports[2];
    
    default:
    return &self->transports[0];
  }
}

// ProtocolRouter transport_ref method (const)
const TransportState* protocol_router_transport_ref(const ProtocolRouter* self, TransportType t) 
{
  switch (t) 
  {
    case TRANSPORT_TYPE_USB_SERIAL:
    return &self->transports[0];
    
    case TRANSPORT_TYPE_BLE:
    return &self->transports[1];
    
    case TRANSPORT_TYPE_WIFI:
    return &self->transports[2];
    
    default:
    return &self->transports[0];
  }
}

// ProtocolRouter lora_protocol getter
Protocol protocol_router_lora_protocol(const ProtocolRouter* self) 
{
  return self->lora_protocol;
}

// ProtocolRouter set_lora_protocol method
void protocol_router_set_lora_protocol(ProtocolRouter* self, Protocol protocol) 
{
  self->lora_protocol = protocol;
}

// ProtocolRouter is_lora_shared getter
bool protocol_router_is_lora_shared(const ProtocolRouter* self) 
{
  return self->lora_shared;
}

// ProtocolRouter priority_transport getter
OptionTransportType protocol_router_priority_transport(const ProtocolRouter* self) 
{
  return self->priority_transport;
}

// ProtocolRouter release_lora_control method
void protocol_router_release_lora_control(ProtocolRouter* self, TransportType transport) 
{
  if (self->priority_transport.has_value && self->priority_transport.value == transport) self->priority_transport = option_transport_type_none();
  TransportState* state = protocol_router_transport(self, transport);
  protocol_detector_reset(&state->detector);
  state->active = false;
}

// ProtocolRouter can_claim_lora helper method
static bool protocol_router_can_claim_lora(const ProtocolRouter* self, TransportType transport, Protocol protocol) 
{
  if (!self->priority_transport.has_value) return true;
  if (self->priority_transport.has_value && self->priority_transport.value == transport) return true;
  if (self->lora_protocol == protocol) return true;
  return false;
}

// ProtocolRouter route_incoming method
Protocol protocol_router_route_incoming(ProtocolRouter* self, TransportType transport, uint8_t byte) 
{
  size_t idx;
  switch (transport) 
  {
    case TRANSPORT_TYPE_USB_SERIAL:
    idx = 0;
    break;
    
    case TRANSPORT_TYPE_BLE:
    idx = 1;
    break;
    
    case TRANSPORT_TYPE_WIFI:
    idx = 2;
    break;
    
    default:
    idx = 0;
    break;
  }
  self->transports[idx].active = true;
  OptionProtocol result = protocol_detector_feed(&self->transports[idx].detector, byte);
  if (result.has_value) 
  {
    Protocol protocol = result.value;
    bool can_claim = !self->priority_transport.has_value || (self->priority_transport.has_value && self->priority_transport.value == transport) || self->lora_protocol == protocol;
    if (can_claim) 
    {
      if (self->lora_protocol == PROTOCOL_UNKNOWN || self->lora_protocol == protocol) self->lora_protocol = protocol;
      if (!self->priority_transport.has_value && protocol_detector_is_locked(&self->transports[idx].detector)) self->priority_transport = option_transport_type_some(transport);
    }
    return protocol;
  } 
  else return protocol_detector_protocol(&self->transports[idx].detector);
}

// ProtocolRouter resolve_conflict method
bool protocol_router_resolve_conflict(ProtocolRouter* self, TransportType transport, Protocol new_protocol) 
{
  if (new_protocol == PROTOCOL_ATCOMMAND) return true;
  if (!self->priority_transport.has_value) return true;
  if (self->priority_transport.has_value && self->priority_transport.value == transport) return true;
  if (self->lora_protocol != new_protocol && self->lora_protocol != PROTOCOL_UNKNOWN) return false;
  return true;
}


// ProtocolRouter status method
void protocol_router_status(const ProtocolRouter* self, TransportStatus status[MAX_TRANSPORTS]) 
{
  status[0].transport = TRANSPORT_TYPE_USB_SERIAL;
  status[0].protocol = protocol_detector_protocol(&self->transports[0].detector);
  status[0].active = self->transports[0].active;
  status[1].transport = TRANSPORT_TYPE_BLE;
  status[1].protocol = protocol_detector_protocol(&self->transports[1].detector);
  status[1].active = self->transports[1].active;
  status[2].transport = TRANSPORT_TYPE_WIFI;
  status[2].protocol = protocol_detector_protocol(&self->transports[2].detector);
  status[2].active = self->transports[2].active;
}

// VecU8_256 constructor
VecU8_256 vec_u8_256_new(void) 
{
  VecU8_256 vec;
  vec.len = 0;
  memset(vec.data, 0, sizeof(vec.data));
  return vec;
}

// VecU8_237 constructor
VecU8_237 vec_u8_237_new(void) 
{
  VecU8_237 vec;
  vec.len = 0;
  memset(vec.data, 0, sizeof(vec.data));
  return vec;
}


// LoRaPacket detect_protocol method
Protocol lora_packet_detect_protocol(const uint8_t* data, size_t data_len) 
{
  if (data_len < 4) return PROTOCOL_UNKNOWN;
  if (data_len >= 20) 
  {
    uint8_t channel_hash = data[3];
    if (data_len >= 12) 
    {
      uint8_t flags = data[11];
      uint8_t hop_limit = flags & 0x07;
      if (hop_limit >= 1 && hop_limit <= 7 && channel_hash != 0) return PROTOCOL_MESHTASTIC;
    }
  }
  if (data_len >= 9) 
  {
    uint8_t flags = data[8];
    uint8_t msg_type = flags & 0x0F;
    uint8_t hop = (flags >> 4) & 0x0F;
    if (msg_type <= 15 && hop <= 7) return PROTOCOL_MESHCORE;
  }
  if (data_len >= 18) 
  {
    uint8_t context = data[16];
    uint8_t header_type = (context >> 6) & 0x03;
    uint8_t propagation_type = (context >> 4) & 0x03;
    if (header_type <= 3 && propagation_type <= 3) 
    {
      size_t zeros = 0;
      for (size_t i = 0; i < 16; i++) 
      {
        if (data[i] == 0) zeros++;
      }
      if (zeros < 8) return PROTOCOL_RNODE;
    }
  }
  return PROTOCOL_UNKNOWN;
}


// UnifiedPacket constructor
UnifiedPacket unified_packet_new(void) 
{
  UnifiedPacket packet;
  packet.source_protocol = PROTOCOL_UNKNOWN;
  packet.dest_protocol = PROTOCOL_UNKNOWN;
  packet.payload = vec_u8_237_new();
  memset(packet.source_addr, 0, sizeof(packet.source_addr));
  memset(packet.dest_addr, 0, sizeof(packet.dest_addr));
  packet.hops = 0;
  packet.rssi = 0;
  packet.snr = 0;
  return packet;
}
