/**
 * meshtastic_radio.h — RadioLib integration for Meshtastic.
 *
 * Handles frequency selection (DJB2 hash), CSMA/CA, and provides a clean
 * interface between RadioLib's SX1262 driver and our protocol layer.
 *
 * Extracted from:
 *   - RadioInterface.cpp:732-741  (DJB2 hash for freq selection)
 *   - RadioInterface.cpp:842-867  (frequency calculation)
 *   - RadioInterface.cpp:571-633  (CSMA/CA delays)
 *   - SX126xInterface.cpp:94      (RadioLib init params)
 *
 * Part of meshtastic-lite.
 */
#pragma once

#include "meshtastic_config.h"
#include "meshtastic_packet.h"
#include "meshtastic_channel.h"
#include "meshtastic_crypto.h"
#include "meshtastic_pb.h"

#include <stdint.h>
#include <stdlib.h>  // for rand()
#include <math.h>

// ─── DJB2 Hash (RadioInterface.cpp:732-741) ───────────────────────────────────

/**
 * DJB2 hash used for channel→frequency mapping.
 * MUST match the firmware's implementation exactly for interop.
 */
uint32_t meshDjb2Hash(const char *str);

// ─── Frequency Calculation ─────────────────────────────────────────────────────

typedef struct MeshFreqConfig 
{
  float    frequency_mhz;    // center frequency
  uint32_t channel_num;      // which slot within the region
  uint32_t num_channels;     // total channel slots in region
}MeshFreqConfig_t;

/**
 * Calculate the LoRa center frequency for a given region, preset, and channel name.
 * Matches RadioInterface.cpp:842-867 exactly.
 *
 * `channel_name`: the effective channel name (preset name if default, e.g. "LongFast")
 * `slot_override`: if >= 0, use this slot instead of hash-based calculation
 *                  (matches Meshtastic's config.lora.channel_num)
 */
MeshFreqConfig_t meshCalcFrequency(const RegionDef_t *region,MeshModemPreset_t preset, const char *channel_name,int32_t slot_override = -1);

// ─── Slot Time Calculation ─────────────────────────────────────────────────────

/**
 * Compute slot time in ms for CSMA/CA.
 * From RadioInterface.cpp:888-899 (sub-GHz path).
 */
uint32_t meshSlotTimeMs(MeshModemPreset_t preset);

// ─── CSMA/CA Delays ────────────────────────────────────────────────────────────

/**
 * Random TX delay for CSMA/CA before sending.
 * From RadioInterface.cpp:571-581.
 *
 * `channel_util_pct`: current channel utilization 0-100
 */
uint32_t meshTxDelayMs(MeshModemPreset preset, float channel_util_pct = 0);
/**
 * SNR-weighted rebroadcast delay for flooding.
 * From RadioInterface.cpp:614-633.
 *
 * Lower SNR = shorter delay (farther nodes flood first).
 * ROUTER role: shorter delay (random within 2*CWsize slots).
 * CLIENT/ROUTER_LATE: offset by 2*CWmax*slot + random within CWsize slots.
 */
uint32_t meshRebroadcastDelayMs(MeshModemPreset preset, MeshRole role, float snr);

// ─── Packet Time Calculation ───────────────────────────────────────────────────

/**
 * Estimate airtime in ms for a LoRa packet of `total_bytes` length.
 * Uses the SX1276/SX1262 time-on-air formula from Semtech AN1200.13.
 */
uint32_t meshPacketAirtimeMs(MeshModemPreset preset, size_t total_bytes);

// ─── RadioLib Configuration Helper ─────────────────────────────────────────────

/**
 * Parameters to pass to RadioLib's SX1262::begin() or equivalent.
 * Matches the call in SX126xInterface.cpp:94.
 */
typedef struct MeshRadioConfig 
{
  float    frequency_mhz;
  float    bandwidth_khz;
  uint8_t  spreading_factor;
  uint8_t  coding_rate;       // denominator (5 = 4/5, 8 = 4/8)
  uint8_t  sync_word;
  int8_t   tx_power_dbm;
  uint16_t preamble_length;
}MeshRadioConfig_t;

/**
 * Build a complete radio config from region + preset + channel name.
 * `slot_override`: if >= 0, use this frequency slot instead of hash-based
 */
MeshRadioConfig_t meshBuildRadioConfig(MeshRegion_t region,
                                                     MeshModemPreset_t preset,
                                                     const char *channel_name,
                                                     int8_t tx_power = 0,
                                                     int32_t slot_override = -1);

// ─── Packet ID Generation ──────────────────────────────────────────────────────

/**
 * Generate a Meshtastic-compatible packet ID.
 * From mesh.proto and the firmware: lower 10 bits are a sequential counter,
 * upper 22 bits are random. Both combine to form a 32-bit ID.
 */
typedef struct MeshPacketIdGen 
{
  uint32_t counter;
  uint32_t random_base;
  void init();
  uint32_t next();
}MeshPacketIdGen_t;

// ─── Complete RX/TX Session ────────────────────────────────────────────────────

/**
 * High-level Meshtastic session state.
 * Ties together channels, radio config, role, and packet ID generation.
 *
 * Usage:
 *   MeshSession session;
 *   session.init(REGION_US, MODEM_LONG_FAST, ROLE_CLIENT, myNodeNum);
 *   session.channels.addDefaultChannel();
 *   MeshRadioConfig rc = session.radioConfig();
 *   // ... configure RadioLib with rc ...
 *   // On RX:
 *   MeshRxResult result;
 *   if (session.processRx(raw_bytes, raw_len, rssi, snr, &result)) { ... }
 */

typedef struct MeshRxResult 
{
  MeshRxPacket packet;      // parsed header + raw/decrypted payload
  MeshData     data;        // decoded Data envelope
  int8_t       channel_idx; // which channel matched (-1 if none / PKI)
  bool         decrypted;   // true if decryption succeeded
  bool         is_pki;      // true if decrypted via PKI (not channel)
}MeshRxResult_t;

typedef struct MeshSession 
{
    MeshRegion        region;
    MeshModemPreset   preset;
    MeshRole          role;
    uint32_t          node_num;     // our node address (bottom 4 bytes of MAC)
    MeshChannelTable  channels;
    MeshPacketIdGen   id_gen;
    int32_t           freq_slot;    // -1 = hash-based, >= 0 = explicit slot override

    // PKI state
    MeshPkiIdentity   pki;          // our x25519 keypair
    MeshNodeKeyStore  node_keys;    // known remote node public keys

    void init(MeshRegion r, MeshModemPreset p, MeshRole rl, uint32_t node,int32_t slot = -1);
    MeshRadioConfig_t radioConfig(int8_t tx_power = 0);
    bool processRx(const uint8_t *raw, size_t raw_len,
                    float rssi, float snr,
                    MeshRxResult *result);
    size_t buildTx(uint8_t channel_idx,
                    uint32_t to,
                    MeshPortNum portnum,
                    const uint8_t *payload, size_t payload_len,
                    bool want_ack, bool ok_to_mqtt,
                    uint8_t *out,
                    uint8_t tx_hop_limit = MESH_HOP_RELIABLE);
    size_t buildDmTx(uint32_t to_node,
                      MeshPortNum portnum,
                      const uint8_t *payload, size_t payload_len,
                      bool want_ack,
                      uint8_t *out,
                      uint32_t (*rand_fn)(void),
                      uint8_t tx_hop_limit = MESH_HOP_RELIABLE);
    size_t buildTextDm(uint32_t to_node, const char *text, bool want_ack,
                        uint8_t *out, uint32_t (*rand_fn)(void),
                        uint8_t tx_hop_limit = MESH_HOP_RELIABLE);
    size_t buildTextTx(uint8_t channel_idx, uint32_t to,
                        const char *text, bool want_ack, bool ok_to_mqtt,
                        uint8_t *out,
                        uint8_t tx_hop_limit = MESH_HOP_RELIABLE);
    size_t buildNodeInfoTx(uint8_t channel_idx, uint32_t to,
                            const char *id, const char *long_name,
                            const char *short_name, uint16_t hw_model,
                            bool want_ack, bool ok_to_mqtt,
                            uint8_t *out,
                            const uint8_t *public_key = nullptr,
                            uint8_t public_key_len = 0,
                            uint8_t tx_hop_limit = MESH_HOP_RELIABLE);
}MeshSession_t;

