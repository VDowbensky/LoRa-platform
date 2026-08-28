#ifndef _MESHTASTIC_RADIO_H_
#define _MESHTASTIC_RADIO_H_

#include "meshtastic_defs.h"

#include <stdint.h>
#include <stdlib.h>  // for rand()
#include <math.h>

// ─── DJB2 Hash (RadioInterface.cpp:732-741) ───────────────────────────────────
//DJB2 hash used for channel→frequency mapping.
//MUST match the firmware's implementation exactly for interop.
uint32_t meshDjb2Hash(const char *str);

// ─── Slot Time Calculation ─────────────────────────────────────────────────────
//Compute slot time in ms for CSMA/CA.
//From RadioInterface.cpp:888-899 (sub-GHz path).
uint32_t meshSlotTimeMs(MeshModemPreset_t preset);

// ─── CSMA/CA Delays ────────────────────────────────────────────────────────────
//Random TX delay for CSMA/CA before sending.
//From RadioInterface.cpp:571-581.
//channel_util_pct`: current channel utilization 0-100
uint32_t meshTxDelayMs(MeshModemPreset_t preset, float channel_util_pct);

//SNR-weighted rebroadcast delay for flooding.
//From RadioInterface.cpp:614-633.
//Lower SNR = shorter delay (farther nodes flood first).
//ROUTER role: shorter delay (random within 2*CWsize slots).
//CLIENT/ROUTER_LATE: offset by 2*CWmax*slot + random within CWsize slots.
uint32_t meshRebroadcastDelayMs(MeshModemPreset_t preset, MeshRole_t role, float snr);

// ─── Packet Time Calculation ───────────────────────────────────────────────────
//Estimate airtime in ms for a LoRa packet of `total_bytes` length.
//Uses the SX1276/SX1262 time-on-air formula from Semtech AN1200.13.
uint32_t meshPacketAirtimeMs(MeshModemPreset_t preset, uint16_t total_bytes);

// ─── Packet ID Generation ──────────────────────────────────────────────────────
/**
 * Generate a Meshtastic-compatible packet ID.
 * From mesh.proto and the firmware: lower 10 bits are a sequential counter,
 * upper 22 bits are random. Both combine to form a 32-bit ID.
 */
typedef struct
{
  uint32_t counter;
  uint32_t random_base;
/*   void init();
  uint32_t next(); */
}MeshPacketIdGen_t;

void MeshPacketIdGen_init(void);
uint32_t MeshPacketIdGen_next(void);

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

typedef struct
{
  MeshRxPacket_t packet;      // parsed header + raw/decrypted payload
  MeshData_t     data;        // decoded Data envelope
  int8_t       channel_idx; // which channel matched (-1 if none / PKI)
  bool         decrypted;   // true if decryption succeeded
  bool         is_pki;      // true if decrypted via PKI (not channel)
}MeshRxResult_t;

typedef struct
{
  float    frequency_mhz;
  float    bandwidth_khz;
  uint8_t  spreading_factor;
  uint8_t  coding_rate;       // denominator (5 = 4/5, 8 = 4/8)
  uint8_t  sync_word;
  int8_t   tx_power_dbm;
  uint16_t preamble_length;
}MeshRadioConfig_t;

typedef struct
{
  MeshRegion_t        region;
  MeshModemPreset_t   preset;
  MeshRole_t          role;
  uint32_t            node_num;     // our node address (bottom 4 bytes of MAC)
  MeshChannelTable_t  channels;
  MeshPacketIdGen_t   id_gen;
  int32_t             freq_slot;    // -1 = hash-based, >= 0 = explicit slot override
  // PKI state
  MeshPkiIdentity_t   pki;          // our x25519 keypair
  MeshNodeKeyStore_t  node_keys;    // known remote node public keys
  MeshRadioConfig_t   radioConfig;

}MeshSession_t;

void MeshSession_init(MeshRegion_t r,MeshModemPreset_t p,MeshRole_t rl,uint32_t node,int32_t slot);
bool MeshSession_processRx(const uint8_t *raw,uint16_t raw_len,float rssi,float snr,MeshRxResult_t *result);
uint16_t MeshSession_buildTx(uint8_t channel_idx,uint32_t to,MeshPortNum_t portnum,const uint8_t *payload, uint16_t payload_len,
                            bool want_ack,bool ok_to_mqtt,uint8_t *out,uint8_t tx_hop_limit);
uint16_t MeshSession_buildDmTx(uint32_t to_node,MeshPortNum_t portnum,const uint8_t *payload, uint16_t payload_len,
                              bool want_ack,uint8_t *out,uint32_t (*rand_fn)(void),uint8_t tx_hop_limit);
uint16_t MeshSession_buildTextDm(uint32_t to_node,const char *text,bool want_ack,uint8_t *out,uint32_t (*rand_fn)(void),
                                uint8_t tx_hop_limit);
uint16_t MeshSession_buildTextTx(uint8_t channel_idx,uint32_t to,const char *text,bool want_ack,bool ok_to_mqtt,
                                uint8_t *out,uint8_t tx_hop_limit);
uint16_t MeshSession_buildNodeInfoTx(uint8_t channel_idx,uint32_t to,const char *id, const char *long_name,const char *short_name, 
                                    uint16_t hw_model,bool want_ack,bool ok_to_mqtt,uint8_t *out,const uint8_t *public_key,
                                    uint8_t public_key_len,uint8_t tx_hop_limit);

// Probably it's not used anymore. I never use RadioLib or similar libraries.
// ─── Frequency Calculation ─────────────────────────────────────────────────────
/* typedef struct MeshFreqConfig 
{
  float    frequency_mhz;    // center frequency
  uint32_t channel_num;      // which slot within the region
  uint32_t num_channels;     // total channel slots in region
}MeshFreqConfig_t; */

//Calculate the LoRa center frequency for a given region, preset, and channel name.
//Matches RadioInterface.cpp:842-867 exactly.
//channel_name`: the effective channel name (preset name if default, e.g. "LongFast")
//slot_override`: if >= 0, use this slot instead of hash-based calculation (matches Meshtastic's config.lora.channel_num)
/* MeshFreqConfig_t meshCalcFrequency(const RegionDef_t *region,MeshModemPreset_t preset, const char *channel_name,int32_t slot_override);

typedef struct
{
  float    frequency_mhz;
  float    bandwidth_khz;
  uint8_t  spreading_factor;
  uint8_t  coding_rate;       // denominator (5 = 4/5, 8 = 4/8)
  uint8_t  sync_word;
  int8_t   tx_power_dbm;
  uint16_t preamble_length;
}MeshRadioConfig_t;

//Build a complete radio config from region + preset + channel name.
//`slot_override`: if >= 0, use this frequency slot instead of hash-based
MeshRadioConfig_t meshBuildRadioConfig(MeshRegion_t region,
                                                     MeshModemPreset_t preset,
                                                     const char *channel_name,
                                                     int8_t tx_power,
                                                     int32_t slot_override); */

#endif
