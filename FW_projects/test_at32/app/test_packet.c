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
	txlen = 36;
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
	txlen = 36;
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
  printf("%d,%.1f,%.1f,%.1f\r\n",test_rxpacket.packet_id,pktstatus.rssi_pkt,pktstatus.signal_rssi_pkt,pktstatus.snr_pkt);
	if(!crc_error)
	{
		printf("BACK: %.1f,%.1f,%.1f\r\n",test_rxpacket.rssi_received,test_rxpacket.signal_rssi_received,test_rxpacket.snr_received);
		printf("Vbatt:%.2f\r\n",test_rxpacket.vbatt);
	} 
	else printf("Bad data\r\n");

	#if OLED_ENABLED
	SSD1306_Clear(0);
	//sprintf(strbuffer,"BATT: %.3fV",Vbatt);
	//GUI_ShowString(0,0,strbuffer,8,1);
	if(crc_error) sprintf(strbuffer,"FERR:");
	else sprintf(strbuffer,"RPCK:");
	GUI_ShowString(0,0,strbuffer,8,1);
	sprintf(strbuffer,"REC:%.1f,%.1f,%.1f",pktstatus.rssi_pkt,pktstatus.signal_rssi_pkt,pktstatus.snr_pkt);
	GUI_ShowString(0,8,strbuffer,8,1);
	if(!crc_error)
	{
		sprintf(strbuffer,"BCK:%.1f,%.1f,%.1f",test_rxpacket.rssi_received,test_rxpacket.signal_rssi_received,test_rxpacket.snr_received);
		GUI_ShowString(0,16,strbuffer,8,1);
		sprintf(strbuffer,"Sender: %d",test_rxpacket.sender_id);
		GUI_ShowString(0,24,strbuffer,8,1);
		sprintf(strbuffer,"Packet: %d",test_rxpacket.packet_id);
		GUI_ShowString(0,32,strbuffer,8,1);
		sprintf(strbuffer,"Voltage: %.2f",test_rxpacket.vbatt);
		GUI_ShowString(0,40,strbuffer,8,1);
	}
	else
	{
		GUI_ShowString(0,16,"Bad data",8,1);
	}
	
//	sprintf(strbuffer,"DEST:0x%08X",rxmessage.destination_id);
//	GUI_ShowString(0,24,strbuffer,8,1);
//	sprintf(strbuffer,"PKT:0x%08X",rxmessage.packet_id);
//	GUI_ShowString(0,32,strbuffer,8,1);
//	sprintf(strbuffer,"S:%d,L:%d,H:0x%02X,R:0x%02X",hop_start,hop_limit,rxmessage.channel_hash,rxmessage.relay_node);
//	GUI_ShowString(0,40,strbuffer,8,1);
//	//sprintf(strbuffer,"Hash:0x%02X,Relay:0x%02X",rxmessage.channel_hash,rxmessage.relay_node);
//	sprintf(strbuffer,"WantAck:%d,MQTT:%d",want_ack,mqtt);
//	GUI_ShowString(0,48,strbuffer,8,1);
//	sprintf(strbuffer,"Payload: %d",rxlen - 16);
//	GUI_ShowString(0,56,strbuffer,8,1);
#endif
	crc_error = false;
}



