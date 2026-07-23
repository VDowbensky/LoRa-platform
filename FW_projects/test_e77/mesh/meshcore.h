#ifndef _MESHCORE_H_
#define _MESHCORE_H_

#include "bsp.h"
#include "radio_proc.h"

#define PAYLOAD_TYPE_REQ						0x00	//Request (destination/source hashes + MAC)
#define PAYLOAD_TYPE_RESPONSE				0x01	//Response to REQ or ANON_REQ
#define PAYLOAD_TYPE_TXT_MSG				0x02	//Plain text message
#define PAYLOAD_TYPE_ACK						0x03	//Acknowledgment
#define PAYLOAD_TYPE_ADVERT					0x04	//Node advertisement
#define PAYLOAD_TYPE_GRP_TXT				0x05	//Group text message (unverified)
#define PAYLOAD_TYPE_GRP_DATA				0x06	//Group datagram (unverified)
#define PAYLOAD_TYPE_ANON_REQ				0x07	//Anonymous request
#define PAYLOAD_TYPE_PATH						0x08	//Returned path
#define PAYLOAD_TYPE_TRACE					0x09	//Trace a path, collecting SNR for each hop
#define PAYLOAD_TYPE_MULTIPART			0x0A	//Packet is part of a sequence of packets
#define PAYLOAD_TYPE_CONTROL				0x0B	//Control packet data (unencrypted)
//0x0C	reserved	
//0x0D	reserved	
//0x0E	reserved	
#define PAYLOAD_TYPE_RAW_CUSTOM			0x0F	//Custom packet (raw bytes, custom encryption)

#define ROUTE_TYPE_TRANSPORT_FLOOD	0x00	//Flood Routing + Transport Codes
#define ROUTE_TYPE_FLOOD						0x01	//Flood Routing
#define ROUTE_TYPE_DIRECT						0x02	//Direct Routing
#define ROUTE_TYPE_TRANSPORT_DIRECT	0x03	//Direct Routing + Transport Codes

//Payload Versions
//Value	Version	Description
//0x00	1	1-byte src/dest hashes, 2-byte MAC
//0x01	2	Future version (e.g., 2-byte hashes, 4-byte MAC)
//0x02	3	Future version
//0x03	4	Future version

typedef struct
{
//header
	uint8_t route_type;
	uint8_t payload_type;
	uint8_t payload_version;
//transport_codes(optional)
	uint8_t transport_codes[4]; //Only present for ROUTE_TYPE_TRANSPORT_FLOOD and ROUTE_TYPE_TRANSPORT_DIRECT
//path_length
	uint8_t hop_count;
	uint8_t hash_size_code;
//path
	uint8_t path[64];
//payload
	uint8_t payload[184];
}meshcore_pkt_t;


void decode_meshcore_packet(uint8_t *buf,meshcore_pkt_t *pkt);
void print_meshcore_packet(meshcore_pkt_t *pkt);


#endif

