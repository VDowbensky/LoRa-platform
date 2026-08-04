#include "meshtastic.h"
#include "menu.h"

extern meshtastic_pkt_t rxmessage;
extern rxpacketstatus_t pktstatus;

void decode_meshtastic_packet(void)
{
  uint8_t i;
  //packet copied to rxbuffer[]. Lenght of packet (pkt_len) is known from LoRa header.
  rxmessage.destination_id = ((uint32_t)radio_rxbuffer[3] << 24) | ((uint32_t)radio_rxbuffer[2] << 16) | ((uint32_t)radio_rxbuffer[1] << 8) | radio_rxbuffer[0]; //slave_id
  rxmessage.sender_id = ((uint32_t)radio_rxbuffer[7] << 24) | ((uint32_t)radio_rxbuffer[6] << 16) | ((uint32_t)radio_rxbuffer[5] << 8) | radio_rxbuffer[4]; //master_id
  rxmessage.packet_id = ((uint32_t)radio_rxbuffer[11] << 24) | ((uint32_t)radio_rxbuffer[10] << 16) | ((uint32_t)radio_rxbuffer[9] << 8) | radio_rxbuffer[8]; //packetnumber
  rxmessage.flags = radio_rxbuffer[12];
  rxmessage.channel_hash = radio_rxbuffer[13];
  rxmessage.next_hop = radio_rxbuffer[14];
  rxmessage.relay_node = radio_rxbuffer[15];
  for(i = 0; i < rxlen-16; i++)
  {
    rxmessage.payload[i] = radio_rxbuffer[i+16]; //237 max.
  }
}

void print_meshtastic_packet(void)
{
  uint8_t i;
  //print rssi, rssi_pkt, snr_pkt first from radio_get_pkt_status()
	uint8_t hop_start = (rxmessage.flags & HOPSTART_MSK) >> HOPSTART_POS;
	uint8_t hop_limit = rxmessage.flags & HOPLIMIT_MSK;
	uint8_t want_ack = (rxmessage.flags & WANTACK_MSK) >> WANTACK_POS;
	uint8_t mqtt = (rxmessage.flags & VIAMQTT_MSK) >> VIAMQTT_POS;
	if(crc_error) 
	{
		//crc_error = false;
		printf("FERR: ");
	}
	else printf("RPCK: ");
  printf("RSSI_PKT=%.1f,RSSI_SIGN=%.1f,SNR=%.1f\r\n",pktstatus.rssi_pkt,pktstatus.signal_rssi_pkt,pktstatus.snr_pkt);
  printf("Destination ID: 0x%08X\r\n",rxmessage.destination_id);
  printf("Sender ID: 0x%08X\r\n",rxmessage.sender_id);
  printf("Packet ID: 0x%08X\r\n",rxmessage.packet_id);
  //flags
  printf("Hop start: %d\r\n",hop_start);
  printf("Hop limit: %d\r\n",hop_limit);
  printf("Channel hash: 0x%02X\r\n",rxmessage.channel_hash);
  printf("Relay node: %d\r\n",rxmessage.relay_node);
  printf("Want ack: ");
  if(want_ack) printf("Yes\r\n");
  else printf("No\r\n");
    printf("Via MQTT: ");
  if(mqtt) printf("Yes\r\n");
  else printf("No\r\n");
  printf("Payload: %d\r\n",rxlen - 16);
  for(i = 0; i < rxlen-16; i++) printf("0x%02X,",rxmessage.payload[i]);
  printf("\r\n");
#if OLED_ENABLED
	if(!menu_mode)
	{
	SSD1306_Clear(0);
	//sprintf(strbuffer,"BATT: %.3fV",Vbatt);
	//GUI_ShowString(0,0,strbuffer,8,1);
	if(crc_error) sprintf(strbuffer,"FERR:%.1f",pktstatus.snr_pkt);
	else sprintf(strbuffer,"RPCK:%.1f",pktstatus.snr_pkt);
	GUI_ShowString(0,0,strbuffer,8,1);
	sprintf(strbuffer,"RSSI:%.1f,%.1f",pktstatus.rssi_pkt,pktstatus.signal_rssi_pkt);
	GUI_ShowString(0,8,strbuffer,8,1);
	sprintf(strbuffer,"SEND:0x%08X",rxmessage.sender_id);
	GUI_ShowString(0,16,strbuffer,8,1);
	sprintf(strbuffer,"DEST:0x%08X",rxmessage.destination_id);
	GUI_ShowString(0,24,strbuffer,8,1);
	sprintf(strbuffer,"PKT:0x%08X",rxmessage.packet_id);
	GUI_ShowString(0,32,strbuffer,8,1);
	sprintf(strbuffer,"S:%d,L:%d,H:0x%02X,R:0x%02X",hop_start,hop_limit,rxmessage.channel_hash,rxmessage.relay_node);
	GUI_ShowString(0,40,strbuffer,8,1);
	//sprintf(strbuffer,"Hash:0x%02X,Relay:0x%02X",rxmessage.channel_hash,rxmessage.relay_node);
	sprintf(strbuffer,"WantAck:%d,MQTT:%d",want_ack,mqtt);
	GUI_ShowString(0,48,strbuffer,8,1);
	sprintf(strbuffer,"Payload: %d",rxlen - 16);
	GUI_ShowString(0,56,strbuffer,8,1);
	}
#endif
	crc_error = false;
}
