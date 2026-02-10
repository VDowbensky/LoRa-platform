#ifndef _DATA_TYPES_H_
#define _DATA_TYPES_H_

enum 
{
  PKT_DATA = 1,
  PKT_BEACON = 2,
  PKT_ACK = 3 
};

/* ================= STRUCTS ================= */
typedef struct 
{
  uint16_t id;
  int8_t rssi;
  int8_t snr;
  uint32_t lastSeen;
} node_t;

typedef struct 
{
  uint16_t id;
  uint16_t dst;
  uint32_t ts;
  uint8_t retries;
  uint8_t len;
  uint8_t payload[64];  // reasonable and fixed size
} pending_t;

typedef struct __attribute__((packed)) 
{
  uint32_t freq;
  uint32_t baud;
  uint32_t beacon_ms;
  uint8_t sf;
  uint8_t bw;
  uint8_t cr;
  uint8_t ttl;
  uint8_t chan;
  int8_t power;
  uint16_t debug;  // 0-false, 1-true
  uint16_t crc;
} mesh_cfg_t;

typedef struct __attribute__((packed)) 
{
  uint8_t  ver;
  uint8_t  type;
  uint16_t src;
  uint16_t dst;
  uint16_t id;
  uint8_t  ttl;
  uint16_t len;
  uint8_t  chan;
} mesh_hdr_t;

typedef struct 
{
  uint16_t src;
  uint16_t id;
} seen_t;

#endif
