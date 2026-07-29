#include "meshcore.h"


meshcore_pkt_t meshcore_msg;
extern rxpacketstatus_t pktstatus;


void decode_meshcore_packet(uint8_t *buf,meshcore_pkt_t *pkt)
{
	pkt->route_type = buf[0] & 0x03;
	pkt->payload_type = (buf[0] >> 2) & 0x0f;
	pkt->payload_version = (buf[0] >> 6) & 0x03;
	if((pkt->route_type == ROUTE_TYPE_TRANSPORT_FLOOD) || (pkt->route_type == ROUTE_TYPE_TRANSPORT_DIRECT))
	{
		for(uint8_t i = 0; i < 4; i++) pkt->transport_codes[i] = buf[i+1];
	}
}
	

//typedef struct
//{
////header
//	uint8_t route_type;
//	uint8_t payload_type;
//	uint8_t payload_version;
////transport_codes(optional)
//	uint8_t transport_codes[4]; //Only present for ROUTE_TYPE_TRANSPORT_FLOOD and ROUTE_TYPE_TRANSPORT_DIRECT
////path_length
//	uint8_t hop_count;
//	uint8_t hash_size_code;
////path
//	uint8_t path[64];
////payload
//	uint8_t payload[184];
//}meshcore_pkt_t;

void print_meshcore_packet(meshcore_pkt_t *pkt)
{
	
}