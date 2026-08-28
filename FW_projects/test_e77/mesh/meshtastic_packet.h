#ifndef __MESHTASTIC_PACKET_H__
#define __MESHTASTIC_PACKET_H__

#include <stdint.h>
#include <string.h>

// ─── Flag Masks (RadioInterface.h:24-28) ───────────────────────────────────────

#define MESH_FLAGS_HOP_LIMIT_MASK   0x07
#define MESH_FLAGS_WANT_ACK_MASK    0x08
#define MESH_FLAGS_VIA_MQTT_MASK    0x10
#define MESH_FLAGS_HOP_START_MASK   0xE0
#define MESH_FLAGS_HOP_START_SHIFT  5

// ─── Special Addresses ─────────────────────────────────────────────────────────

#define MESH_ADDR_BROADCAST  0xFFFFFFFF

// ─── Packet Header (16 bytes, matches RadioInterface.h:34-54) ──────────────────

/**
 * On-wire packet header. Packed, little-endian.
 * This is NOT a protobuf — it is raw bytes at the start of every LoRa frame.
 *
 * Layout (16 bytes):
 *   [0..3]   to          destination NodeNum (LE)
 *   [4..7]   from        source NodeNum (LE)
 *   [8..11]  id          packet ID (LE)
 *   [12]     flags       hop_limit[2:0], want_ack[3], via_mqtt[4], hop_start[7:5]
 *   [13]     channel     channel hash (XOR of name + PSK bytes)
 *   [14]     next_hop    last byte of next-hop NodeNum
 *   [15]     relay_node  last byte of relaying NodeNum
 */
typedef struct __attribute__((packed)) 
{
    uint32_t to;
    uint32_t from;
    uint32_t id;
    uint8_t  flags;
    uint8_t  channel;
    uint8_t  next_hop;
    uint8_t  relay_node;
} MeshPacketHeader_t;

// ─── Decoded Packet ────────────────────────────────────────────────────────────
//A received Meshtastic packet after header parsing and (optionally) decryption.
typedef struct  
{
  // Header fields
  uint32_t to;
  uint32_t from;
  uint32_t id;
  uint8_t  hop_limit;
  uint8_t  hop_start;
  bool     want_ack;
  bool     via_mqtt;
  uint8_t  channel_hash;
  uint8_t  next_hop;
  uint8_t  relay_node;
  // Payload (after header, before decryption this is ciphertext)
  uint8_t  payload[255 - 16];
  uint16_t   payload_len;
  // Radio metadata
  float    rssi;
  float    snr;
  // Set after successful decryption
  int8_t   channel_index;  // -1 if not yet matched
}MeshRxPacket_t;


bool meshParsePacket(const uint8_t *raw, uint16_t len, MeshRxPacket_t *pkt);
uint16_t meshBuildPacket(uint8_t *out,
                                      uint32_t to, uint32_t from, uint32_t id,
                                      uint8_t hop_limit, uint8_t hop_start,
                                      bool want_ack, bool via_mqtt,
                                      uint8_t channel_hash,
                                      uint8_t next_hop, uint8_t relay_node,
                                      const uint8_t *payload, uint16_t payload_len);
                                      
#endif

                                      