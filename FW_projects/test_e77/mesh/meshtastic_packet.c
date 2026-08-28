#include "meshtastic_packet.h"

/**
 * Parse raw LoRa frame bytes into a MeshRxPacket.
 * Returns false if frame is too short to contain a valid header.
 */
bool meshParsePacket(const uint8_t *raw, uint16_t len, MeshRxPacket_t *pkt) 
{
  if (len < sizeof(MeshPacketHeader_t)) return false;
  const MeshPacketHeader_t *hdr = (const MeshPacketHeader_t *)raw;

  pkt->to           = hdr->to;
  pkt->from         = hdr->from;
  pkt->id           = hdr->id;
  pkt->hop_limit    = hdr->flags & MESH_FLAGS_HOP_LIMIT_MASK;
  pkt->want_ack     = (hdr->flags & MESH_FLAGS_WANT_ACK_MASK) != 0;
  pkt->via_mqtt     = (hdr->flags & MESH_FLAGS_VIA_MQTT_MASK) != 0;
  pkt->hop_start    = (hdr->flags & MESH_FLAGS_HOP_START_MASK) >> MESH_FLAGS_HOP_START_SHIFT;
  pkt->channel_hash = hdr->channel;
  pkt->next_hop     = hdr->next_hop;
  pkt->relay_node   = hdr->relay_node;
  pkt->payload_len  = len - sizeof(MeshPacketHeader_t);
  if (pkt->payload_len > sizeof(pkt->payload)) pkt->payload_len = sizeof(pkt->payload);
  memcpy(pkt->payload, raw + sizeof(MeshPacketHeader_t), pkt->payload_len);
  pkt->rssi = 0;
  pkt->snr  = 0;
  pkt->channel_index = -1;
  return true;
}

/**
 * Build a raw LoRa frame from components.
 * Returns total frame size (header + payload_len).
 * `out` must be at least 16 + payload_len bytes.
 */
uint16_t meshBuildPacket(uint8_t *out,
                                      uint32_t to, uint32_t from, uint32_t id,
                                      uint8_t hop_limit, uint8_t hop_start,
                                      bool want_ack, bool via_mqtt,
                                      uint8_t channel_hash,
                                      uint8_t next_hop, uint8_t relay_node,
                                      const uint8_t *payload, uint16_t payload_len)
{
  MeshPacketHeader *hdr = (MeshPacketHeader *)out;
  hdr->to         = to;
  hdr->from       = from;
  hdr->id         = id;
  hdr->flags      = (hop_limit & MESH_FLAGS_HOP_LIMIT_MASK)
                  | (want_ack ? MESH_FLAGS_WANT_ACK_MASK : 0)
                  | (via_mqtt ? MESH_FLAGS_VIA_MQTT_MASK : 0)
                  | ((hop_start << MESH_FLAGS_HOP_START_SHIFT) & MESH_FLAGS_HOP_START_MASK);
  hdr->channel    = channel_hash;
  hdr->next_hop   = next_hop;
  hdr->relay_node = relay_node;

  memcpy(out + sizeof(MeshPacketHeader), payload, payload_len);
  return sizeof(MeshPacketHeader) + payload_len;
}
