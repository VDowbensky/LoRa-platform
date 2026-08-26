#include "meshtastic.h"

// ─── Port Numbers (portnums.pb.h) ──────────────────────────────────────────────
MeshPortNum_t  MeshPortNum;

// ─── Meshtastic Data Envelope ──────────────────────────────────────────────────

/**
 * Decoded meshtastic.Data protobuf.
 * Fields: portnum(1), payload(2), want_response(3), dest(4), source(5),
 *         request_id(6), reply_id(7), emoji(8), bitfield(9)
 */
MeshData_t MeshData; 

/**
 * Decode a meshtastic.Data protobuf from raw bytes.
 * Returns true on success. Validates that portnum != 0 (UNKNOWN).
 *
 * This is the validation step equivalent to Router.cpp:499-503.
 */
bool meshDecodeData(const uint8_t *buf,size_t len,MeshData_t *out) 
{
  memset(out, 0, sizeof(MeshData_t));
  while (!pb_exhausted()) 
  {
    uint32_t field; 
    uint8_t wtype;
    if (!pb_readTag(&field,&wtype)) break;
    switch (field) 
    {
      case 1: 
      { // portnum (varint)
        uint64_t v;
        if (!pb_readVarint(&v)) return false;
        out->portnum = (MeshPortNum_t)(v & 0xFFFF);
        break;
      }
      case 2: 
      { // payload (bytes)
        if (!pb_readBytes(out->payload,sizeof(out->payload),&out->payload_len)) return false;
        break;
      }
      case 3: 
      { // want_response (varint/bool)
        uint64_t v;
        if (!pb_readVarint(&v)) return false;
        out->want_response = (v != 0);
        break;
      }
      case 4: 
      { // dest (fixed32)
        if (!pb_readFixed32(&out->dest)) return false;
        break;
      }
      case 5: 
      { // source (fixed32)
        if (!pb_readFixed32(&out->source)) return false;
        break;
      }
      case 6: 
      { // request_id (fixed32)
        if (!pb_readFixed32(&out->request_id)) return false;
        break;
      }
      case 9: 
      { // bitfield (varint) — bit 0 = ok_to_mqtt
        uint64_t v;
        if (!pb_readVarint(&v)) return false;
        out->bitfield = (uint32_t)v;
        break;
      }
      default:
      if (!pb_skipField(wtype)) return false;
      break;
    }
  }
  // Reject UNKNOWN portnum (bad PSK produces garbage)
  return (out->portnum != PORT_UNKNOWN);
}

/**
 * Protobuf validation callback for MeshChannelTable::tryDecrypt().
 * Returns true if the decrypted bytes parse as a valid Data message.
 */
bool meshValidateData(const uint8_t *plaintext,uint16_t len) 
{
  MeshData_t tmp;
  return meshDecodeData(plaintext,len,&tmp);
}

// ─── Position ──────────────────────────────────────────────────────────────────

MeshPosition_t MeshPosition;

bool meshDecodePosition(const uint8_t *buf,size_t len,MeshPosition_t *out) 
{
  memset(out, 0, sizeof(MeshPosition_t));
  while (!pb_exhausted()) 
  {
    uint32_t field; 
    uint8_t wtype;
    if (!pb_readTag(&field, &wtype)) break;
    switch (field) 
    {
      case 1: 
      { // latitude_i (sfixed32)
        uint32_t v;
        if (!pb_readFixed32(&v)) return false;
        out->latitude_i = (int32_t)v;
        break;
      }
      case 2: 
      { // longitude_i (sfixed32)
        uint32_t v;
        if (!pb_readFixed32(&v)) return false;
        out->longitude_i = (int32_t)v;
        break;
      }
      case 3: 
      { // altitude (int32 varint)
        uint64_t v;
        if (!pb_readVarint(&v)) return false;
        // zigzag decode for sint32
        out->altitude = (int32_t)((v >> 1) ^ -(int32_t)(v & 1));
        break;
      }
      case 4: 
      { // time (fixed32)
        if (!pb_readFixed32(&out->time)) return false;
        break;
      }
      case 12: 
      { // precision_bits (uint32 varint)
        uint64_t v;
        if (!pb_readVarint(&v)) return false;
        out->precision_bits = (uint32_t)v;
        break;
      }
      default:
      if (!pb_skipField(wtype)) return false;
      break;
    }
  }
  return true;
}

// ─── User / NodeInfo ───────────────────────────────────────────────────────────

MeshUser_t MeshUser;

bool meshDecodeUser(const uint8_t *buf,size_t len,MeshUser_t *out) 
{
  memset(out, 0, sizeof(MeshUser_t));
  while (!pb_exhausted()) 
  {
    uint32_t field; 
    uint8_t wtype;
    if (!pb_readTag(&field, &wtype)) break;
    size_t slen;
    switch (field) 
    {
      case 1: // id
      if (!pb_readString(out->id, sizeof(out->id), &slen)) return false;
      break;
      
      case 2: // long_name
      if (!pb_readString(out->long_name, sizeof(out->long_name), &slen)) return false;
      break;
      
      case 3: // short_name
      if (!pb_readString(out->short_name, sizeof(out->short_name), &slen)) return false;
      break;
      
      case 5: 
      { // hw_model
        uint64_t v;
        if (!pb_readVarint(&v)) return false;
        out->hw_model = (uint16_t)v;
        break;
      }
      
      case 8: 
      { // public_key
        if (!pb_readBytes(out->public_key, sizeof(out->public_key), &slen)) return false;
        out->public_key_len = (uint8_t)slen;
        break;
      }
      
      default:
      if (!pb_skipField(wtype)) return false;
      break;
    }
  }
  return true;
}

// ─── Telemetry (Device Metrics subset) ─────────────────────────────────────────

MeshDeviceMetrics_t MeshDeviceMetrics;
MeshTelemetry_t MeshTelemetry;

bool meshDecodeDeviceMetrics(const uint8_t *buf,uint16_t len,MeshDeviceMetrics_t *out)
{
  memset(out,0,sizeof(MeshDeviceMetrics_t));
  while (!pb_exhausted()) 
  {
    uint32_t field; uint8_t wtype;
    if (!pb_readTag(&field, &wtype)) break;
    switch (field) 
    {
      case 1: 
      { 
        uint64_t v; 
        if (!pb_readVarint(&v)) return false;
        out->battery_level = (uint32_t)v; 
        break; 
      }
      
      case 2: 
      if (!pb_readFloat(&out->voltage)) return false; 
      break;
      
      case 3: 
      if (!pb_readFloat(&out->channel_utilization)) return false; 
      break;
      
      case 4: 
      if (!pb_readFloat(&out->air_util_tx)) return false; 
      break;
      
      case 5: 
      { 
        uint64_t v; 
        if (!pb_readVarint(&v)) return false;
        out->uptime_seconds = (uint32_t)v; 
        break; 
      }
      default: 
      if (!pb_skipField(wtype)) return false; 
      break;
    }
  }
  return true;
}

bool meshDecodeTelemetry(const uint8_t *buf,size_t len,MeshTelemetry_t *out)
{
  memset(out,0,sizeof(MeshTelemetry_t));
  while (!pb_exhausted()) 
  {
    uint32_t field; 
    uint8_t wtype;
    
    if (!Ppb_readTag(&field, &wtype)) break;
    switch (field) 
    {
      case 1: // time
      if (!pb_readFixed32(&out->time)) return false;
      break;
        
      case 2: 
      { // device_metrics (submessage)
        PbCursor_t sub;
        if (!pb_readLengthDelimited(&sub)) return false;
        out->has_device_metrics = meshDecodeDeviceMetrics(sub.buf,sub.len,&out->device_metrics);
        break;
      }
      default:
      if (!pb_skipField(wtype)) return false;
      break;
    }
  }
  return true;
}

/**
 * Encode a meshtastic.Data protobuf.
 * Returns encoded length, or 0 on failure.
 */
uint16_t meshEncodeData(uint8_t *buf, uint16_t capacity,
                                     MeshPortNum portnum,
                                     const uint8_t *payload, uint16_t payload_len,
                                     bool want_response = false,
                                     bool ok_to_mqtt = false)
{
  if (!pb_writeVarintField(1, (uint64_t)portnum)) return 0;
  if (payload_len > 0)
  {
    if (!pb_writeBytes(2, payload, payload_len)) return 0;
  }
  if (want_response)
  {
    if (!pb_writeVarintField(3, 1)) return 0;
  }
  if (ok_to_mqtt)
  {
    if (!pb_writeVarintField(9, 1)) return 0;  // bitfield bit 0 = ok_to_mqtt
  }
  return pb_written();
}

/**
 * Encode a meshtastic.User protobuf.
 * Fields match meshDecodeUser: 1=id, 2=long_name, 3=short_name, 5=hw_model, 8=public_key.
 * Returns encoded length, or 0 on failure.
 */
uint16_t meshEncodeUser(uint8_t *buf, size_t capacity,
                                     const char *id, const char *long_name,
                                     const char *short_name, uint16_t hw_model,
                                     const uint8_t *public_key = nullptr,
                                     uint8_t public_key_len = 0) 
{
  if (id && id[0])
  {
    if (!pb_writeString(1, id)) return 0;
  }
  if (long_name && long_name[0]) 
  {
    if (!pb_writeString(2, long_name)) return 0;
  }
  if (short_name && short_name[0]) 
  {
    if (!pb_writeString(3, short_name)) return 0;
  }
  if (hw_model)
  {
    if (!pb_writeVarintField(5, hw_model)) return 0;
  }
  if (public_key && public_key_len > 0)
  {
    if (!pb_writeBytes(8, public_key, public_key_len)) return 0;
  }
  return pb_written();
}
