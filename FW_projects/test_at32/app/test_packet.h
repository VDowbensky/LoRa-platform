#ifndef _TEST_PACKET_H_
#define _TEST_PACKET_H_

#include "bsp.h"
#include "radio_proc.h"

typedef struct
{
  uint32_t destination_id; //slave_id
  uint32_t sender_id; //master_id
  uint32_t packet_id; //packetnumber
	float rssi_received; //16
	float signal_rssi_received; //20
	float snr_received;
	float latitude; 
	float longitude;
	float vbatt; //36
  uint8_t reserved[224]; 
}testpacket_t;

void prepare_test_request(void);
void prepare_test_ack(void);
void process_test_packet(void);

extern testpacket_t test_txpacket, test_rxpacket;

#endif
