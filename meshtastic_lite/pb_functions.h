#ifndef _PB_FUNC_H_
#define _PB_FUNC_H_

#include "meshtastic.h"

typedef struct PbCursor 
{
  const uint8_t *buf;
  uint16_t pos;
  uint16_t len;
}PbCursor_t;

//Read functions
bool pb_exhausted(void); 
uint16_t pb_remaining(void);
bool pb_readByte(uint8_t *out);
bool pb_readVarint(uint64_t *out);
bool pb_readFixed32(uint32_t *out);
bool pb_readFixed64(uint64_t *out);
bool pb_readFloat(float *out);
//Read a protobuf field tag. Returns field number and wire type.
bool pb_readTag(uint32_t *field_num,uint8_t *wire_type);
//Skip a field value based on wire type.
bool pb_skipField(uint8_t wire_type);
//Read a length-delimited field's length prefix, returning a sub-cursor.
bool pb_readLengthDelimited(PbCursor *sub);
//Read a length-delimited field into a buffer.
bool pb_readBytes(uint8_t *out,uint16_t max_len,uint16_t *out_len);
//Read a string field (length-delimited bytes interpreted as UTF-8).
bool pb_readString(char *out,uint16_t max_len,uint16_t *out_len);

// Write functions
bool PbWriter_full(void);  
uint16_t pb_written(void);
bool pb_writeByte(uint8_t b);
bool pb_writeVarint(uint64_t val);
bool pb_writeTag(uint32_t field, uint8_t wire_type);
bool pb_writeFixed32(uint32_t val);
bool pb_writeBytes(uint32_t field, const uint8_t *data, size_t len);
bool pb_writeString(uint32_t field, const char *str);
bool pb_writeVarintField(uint32_t field, uint64_t val);
bool pb_writeFixed32Field(uint32_t field, uint32_t val);

#endif
