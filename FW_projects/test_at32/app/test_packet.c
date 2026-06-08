#include "test_packet.h"


testpacket_t test_txpacket, test_rxpacket;

void prepare_test_request(void)
{
	test_txpacket.sender_id = radioconfig.id;
	test_txpacket.destination_id = radioconfig.pair_id;
	test_txpacket.packet_id = txpacketnumber;
	test_txpacket.rssi_received = -128;
	test_txpacket.snr_received = -30;
	test_txpacket.vbatt = Vbatt;
	txlen = 32;
	memcpy((void*)radio_txbuffer,(void*)&test_txpacket,txlen);
	printf("TX: %d\r\n",txpacketnumber);
}

void prepare_test_ack(void)
{
	test_txpacket.sender_id = radioconfig.id;
	test_txpacket.destination_id = radioconfig.pair_id;
	test_txpacket.packet_id = txpacketnumber;
	test_txpacket.rssi_received = pktstatus.rssi_pkt;
	test_txpacket.signal_rssi_received = pktstatus.signal_rssi_pkt;
	test_txpacket.snr_received = pktstatus.snr_pkt;
	test_txpacket.vbatt = Vbatt;
	txlen = 32;
	memcpy((void*)radio_txbuffer,(void*)&test_txpacket,txlen);
	printf("TX: %d\r\n",txpacketnumber);
}

void process_test_packet(void)
{
	memcpy((void*)&test_rxpacket,(void*)&radio_rxbuffer,rxlen);
	
	//print results
	if(crc_error) 
	{
		//crc_error = false;
		printf("FERR: ");
	}
	else printf("RPCK: ");
  printf("RSSI_PKT=%.1f,RSSI_SIGN=%.1f,SNR=%.1f\r\n",pktstatus.rssi_pkt,pktstatus.signal_rssi_pkt,pktstatus.snr_pkt);
	printf("RSSI_PKT=%.1f,RSSI_SIGN=%.1f,SNR=%.1f\r\n",test_rxpacket.rssi_received,test_rxpacket.signal_rssi_received,test_rxpacket.snr_received);
  printf("Sender ID: 0x%08X\r\n",test_rxpacket.sender_id);
  printf("Packet ID: 0x%08X\r\n",test_rxpacket.packet_id);
	
}



