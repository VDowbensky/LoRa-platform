#include "pb_functions.h"

uint8_t pb_read_buf[256];
uint16_t pb_read_pos;
uint16_t pb_read_len;

uint8_t pb_write_buf[256];
uint16_t pb_write_pos;
uint16_t pb_write_capacity;

//Read functions
bool pb_exhausted(void)
{ 
  return pb_read_pos >= pb_read_len; 
}
    
uint16_t pb_remaining(void) 
{ 
  return (pb_read_pos < pb_read_len) ? (pb_read_len - pb_read_pos) : 0; 
}

bool pb_readByte(uint8_t *out) 
{
  if (pb_read_pos >= pb_read_len) return false;
  *out = buf[pb_read_pos++];
  return true;
}

bool pb_readVarint(uint64_t *out) 
{
  *out = 0;
  int shift = 0;
  uint8_t b;
  do 
  {
    if (!pb_readByte(&b)) return false;
    *out |= (uint64_t)(b & 0x7F) << shift;
    shift += 7;
    if (shift > 63) return false; // overflow
  } while (b & 0x80);
  return true;
}

bool pb_readFixed32(uint32_t *out) 
{
  if (pb_remaining() < 4) return false;
  memcpy(out,pb_read_buf + pb_read_pos,4);
  pb_read_pos += 4;
  return true;
}

bool pb_readFixed64(uint64_t *out) 
{
  if (pb_remaining() < 8) return false;
  memcpy(out,pb_read_buf + pb_read_pos,8);
  pb_read_pos += 8;
  return true;
}

bool pb_readFloat(float *out) 
{
  uint32_t bits;
  if (!pb_readFixed32(&bits)) return false;
  memcpy(out,&bits,4);
  return true;
}

//Read a protobuf field tag. Returns field number and wire type.
bool pb_readTag(uint32_t *field_num,uint8_t *wire_type) 
{
  uint64_t tag;
  if (!pb_readVarint(&tag)) return false;
  *wire_type = tag & 0x07;
  *field_num = tag >> 3;
  return true;
}

//Skip a field value based on wire type.
bool pb_skipField(uint8_t wire_type) 
{
  switch (wire_type) 
  {
    case 0: 
    { // varint
      uint64_t dummy;
      return pb_readVarint(&dummy);
    }
    case 1: 
    { // 64-bit
      if (pb_remaining() < 8) return false;
      pb_read_pos += 8; 
      return true;
    }
    case 2: 
    { // length-delimited
      uint64_t slen;
      if (!pb_readVarint(&slen)) return false;
      if (pb_remaining() < slen) return false;
      pb_read_pos += slen; 
      return true;
    }
    case 5: 
    { // 32-bit
      if (pb_remaining() < 4) return false;
      pb_read_pos += 4; return true;
    }
    default:
    return false; // unsupported wire type
  }
}

//Read a length-delimited field's length prefix, returning a sub-cursor.
bool pb_readLengthDelimited(PbCursor *sub) 
{
  uint64_t slen;
  if (!pb_readVarint(&slen)) return false;
  if (pb_remaining() < slen) return false;
  sub->buf = pb_read_buf + pb_read_pos;
  sub->pos = 0;
  sub->len = (uint16_t)slen;
  pb_read_pos += slen;
  return true;
}

//Read a length-delimited field into a buffer.
bool pb_readBytes(uint8_t *out,uint16_t max_len,uint16_t *out_len) 
{
  uint64_t slen;
  if (!pb_readVarint(&slen)) return false;
  if (pb_remaining() < slen) return false;
  uint16_t copy_len = (slen > max_len) ? max_len : (uint16_t)slen;
  memcpy(out,pb_read_buf + pb_read_pos,copy_len);
  *out_len = copy_len;
  pb_read_pos += slen;
  return true;
}

//Read a string field (length-delimited bytes interpreted as UTF-8).
bool pb_readString(char *out,uint16_t max_len,uint16_t *out_len) 
{
  uint16_t slen;
  if (!pb_readBytes((uint8_t *)out,max_len - 1,&slen)) return false;
  out[slen] = '\0';
  *out_len = slen;
  return true;
}

// ─── Minimal Protobuf Encoder (for TX) ─────────────────────────────────────────

bool PbWriter_full(void)
{ 
  return pb_write_pos >= pb_write_capacity; 
}
    
uint16_t pb_written(void)
{ 
  return pb_write_pos; 
}

bool pb_writeByte(uint8_t b) 
{
  if (pb_write_pos >= pb_write_capacity) return false;
  pb_write_buf[pb_write_pos++] = b;
  return true;
}

bool pb_writeVarint(uint64_t val) 
{
  do 
  {
    uint8_t b = val & 0x7F;
    val >>= 7;
    if (val) b |= 0x80;
    if (!pb_writeByte(b)) return false;
  } while (val);
  return true;
}

bool pb_writeTag(uint32_t field, uint8_t wire_type) 
{
  return pb_writeVarint((uint64_t)(field << 3 | wire_type));
}

bool pb_writeFixed32(uint32_t val) 
{
  if (pb_write_pos + 4 > pb_write_capacity) return false;
  memcpy(pb_write_buf + pb_write_pos, &val, 4);
  pb_write_pos += 4;
  return true;
}

bool pb_writeBytes(uint32_t field, const uint8_t *data, size_t len) 
{
  if (!pb_writeTag(field, 2)) return false;
  if (!pb_writeVarint(len)) return false;
  if (pb_write_pos + len > pb_write_capacity) return false;
  memcpy(pb_write_buf + pb_write_pos, data, len);
  pb_write_pos += len;
  return true;
}

bool pb_writeString(uint32_t field, const char *str) 
{
  return pb_writeBytes(field,(const uint8_t *)str,strlen(str));
}

bool pb_writeVarintField(uint32_t field, uint64_t val) 
{
  if (!pb_writeTag(field, 0)) return false;
  return pb_writeVarint(val);
}

bool pb_writeFixed32Field(uint32_t field, uint32_t val) 
{
  if (!pb_writeTag(field, 5)) return false;
  return pb_writeFixed32(val);
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
