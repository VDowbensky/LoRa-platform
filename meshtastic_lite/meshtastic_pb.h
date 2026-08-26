#ifndef __MESHTASTIC_PB_H__
#define __MESHTASTIC_PB_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

// ─── Port Numbers (portnums.pb.h) ──────────────────────────────────────────────

typedef enum
{
  PORT_UNKNOWN          = 0,
  PORT_TEXT_MESSAGE      = 1,
  PORT_POSITION          = 3,
  PORT_NODEINFO          = 4,
  PORT_ROUTING           = 5,
  PORT_ADMIN             = 6,
  PORT_WAYPOINT          = 8,
  PORT_TELEMETRY         = 67,
  PORT_TRACEROUTE        = 70,
  PORT_NEIGHBORINFO      = 71,
}MeshPortNum_t;

// ─── Meshtastic Data Envelope ──────────────────────────────────────────────────

/**
 * Decoded meshtastic.Data protobuf.
 * Fields: portnum(1), payload(2), want_response(3), dest(4), source(5),
 *         request_id(6), reply_id(7), emoji(8), bitfield(9)
 */
typedef struct
{
  MeshPortNum_t portnum;
  uint8_t     payload[240];     // inner payload bytes
  uint16_t      payload_len;
  bool        want_response;
  uint32_t    dest;
  uint32_t    source;
  uint32_t    request_id;
  uint32_t    bitfield;         // field 9: bit 0 = ok_to_mqtt
}MeshData_t;

bool meshDecodeData(const uint8_t *buf, uint16_t len, MeshData *out);
bool meshValidateData(const uint8_t *plaintext, uint16_t len);
// ─── Position ──────────────────────────────────────────────────────────────────

typedef struct
{
  int32_t  latitude_i;      // degrees * 1e7  (sfixed32, field 1)
  int32_t  longitude_i;     // degrees * 1e7  (sfixed32, field 2)
  int32_t  altitude;        // meters          (int32, field 3)
  uint32_t time;            // Unix epoch      (fixed32, field 4)
  uint32_t altitude_hae;    // field 13 in some versions
  uint32_t precision_bits;  // field 12
  double latitude();
  double longitude();
}MeshPosition_t;

bool meshDecodePosition(const uint8_t *buf, uint16_t len, MeshPosition *out);

// ─── User / NodeInfo ───────────────────────────────────────────────────────────

typedef struct
{
  char     id[16];         // e.g., "!aabbccdd"   (string, field 1)
  char     long_name[40];  // human name            (string, field 2)
  char     short_name[5];  // 3-char abbreviation   (string, field 3)
  uint16_t hw_model;       // HardwareModel enum    (varint, field 5)
  uint8_t  public_key[32]; //                        (bytes, field 8)
  uint8_t  public_key_len;
}MeshUser_t;

bool meshDecodeUser(const uint8_t *buf, uint16_t len, MeshUser_t *out);

// ─── Telemetry (Device Metrics subset) ─────────────────────────────────────────

typedef struct
{
  uint32_t battery_level;      // 0-100       (varint, field 1)
  float    voltage;            //             (float, field 2)
  float    channel_utilization; //            (float, field 3)
  float    air_util_tx;        //             (float, field 4)
  uint32_t uptime_seconds;     //             (varint, field 5)
}MeshDeviceMetrics_t;

typedef struct
{
  uint32_t          time;      // Unix epoch  (fixed32, field 1)
  bool              has_device_metrics;
  MeshDeviceMetrics device_metrics;
}MeshDeviceMetrics_t;

bool meshDecodeDeviceMetrics(const uint8_t *buf, uint16_t len,MeshDeviceMetrics_t *out);
bool meshDecodeTelemetry(const uint8_t *buf, uint16_t len, MeshTelemetry_t *out);

/**
 * Encode a meshtastic.Data protobuf.
 * Returns encoded length, or 0 on failure.
 */
uint16_t meshEncodeData(uint8_t *buf, uint16_t capacity,
                                     MeshPortNum portnum,
                                     const uint8_t *payload, uint16_t payload_len,
                                     bool want_response = false,
                                     bool ok_to_mqtt = false);

/**
 * Encode a meshtastic.User protobuf.
 * Fields match meshDecodeUser: 1=id, 2=long_name, 3=short_name, 5=hw_model, 8=public_key.
 * Returns encoded length, or 0 on failure.
 */
uint16_t meshEncodeUser(uint8_t *buf, uint16_t capacity,
                                     const char *id, const char *long_name,
                                     const char *short_name, uint16_t hw_model,
                                     const uint8_t *public_key = nullptr,
                                     uint8_t public_key_len = 0);
                                     
                                     
#endif
