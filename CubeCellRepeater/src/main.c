void radio_proc(void) 
{
			if(packet_received)
			{
				packet_received = false;
				rxled_on();
				process_rx_packet();
				rxled_off();
				radio_rx();
			}
			if(packet_sent)
			{
				packet_sent = false;
				txled_off();
				radio_rx();
			}
}

void process_rx_packet(void)
  {
    packet_received = false;
    rxled_on();
    
    Packet_t pck;
    
    radio_getpktstatus(&pktstatus);
		//copy packet to buffer
		radio_getpacket(radio_rxbuffer);
    decode_meshtastic_packet(pck);
    print_meshtastic_packet();
  }
  
  void decode_meshtastic_packet(Packet_t pck)
  {
    pck.packetTime = millis();
    pck.size = radio.getPacketLength();
    if (pck.size == 0) 
    {
      printf("[ERR]Received packet length = 0!\r\n");
      return; // we are still in receive mode
    }
  }
    

    err = radio.readData(pck.buf, pck.size);
    PacketHeader* h = (PacketHeader *)pck.buf;
    snr = radio.getSNR();
    if (err == RADIOLIB_ERR_NONE) {
      const int32_t payloadLen = pck.size - sizeof(PacketHeader);
      if (payloadLen < 0) {
        MSG("[WARN]Not a Meshtastic packet, too short!\n\r");
        return; // will not repeat, continue receiving
      }
      const uint8_t hop_limit = h->flags & PACKET_FLAGS_HOP_MASK;
      MSG("[NEW](id=0x%08X) (HopLim %d) ", h->id, hop_limit);
      repeatPacket = msgID.add(h->id);
      // print new packets only not repeated due to HopLim 0
      if ((repeatPacket) && (hop_limit==0)) {
        MSG("\n\r");
        #ifndef SILENT      
        perhapsDecode(&pck);
        MSG("\n\r");
        #endif
      }
      if (hop_limit == 0) repeatPacket = false;
      // do not repeat if id is known or hop limit is zero
      if ( !repeatPacket ){
        MSG(" !!! no repeat !!!\n\r");
      } 
      else {
        h->flags -= 1; // decrease hop limit by 1
        txQueue.add(&pck);
      }
    } else if (err == RADIOLIB_ERR_CRC_MISMATCH) {
      MSG(" [ERROR]CRC error!\n\r");
    } else {
      MSG(" [ERROR]Receive failed, code: %i!\n\r", err);
    }
  }

  if ( (txQueue.Count > 0) || !(p==NULL) ) { 
    uint32_t now = millis();
    // we have packets in queue and currently no packet to transmit
    if (p == NULL) {
      if ( txQueue.pop() ) {
        p = &PacketToSend;
      }
    }
    // resume receiving in case we popped an empty packet
    if (p == NULL) {
      PacketSent = true; 
      return;
    } 
      
    if (p->size > 0) {  // size == 0 means "deleted", we don't send deleted packets
      uint32_t wait = getTxDelayMsecWeighted(snr);

      MSG("[INF]wait %i ms before TX\n\r", wait);
      activeReceiveStart = 0;
      while ( ((now + wait) > millis() ) || ( isActivelyReceiving() ) ) {
        // while waiting, we still are in receive mode
        if (dio1) {
          MSG("[INF]New packet, no TX\n\r");
          return; // new packet arrived, return to handle it
        }
        delay(5);
      }
      // drop packet if we could not send it in 1 minute
      if ( (p->packetTime + 60*1000) < millis() ) {
        p = NULL;
        PacketSent = true;
        MSG("[INF] TX aborted, could not send packet in 1 minute\n\r");
        return;
      }
      if (perhapsSend(p) ) {
        // packet successfully sent
        // try to decode the packet and print it
        #ifndef SILENT
        perhapsDecode(p);
        MSG("\n\r");
        #endif
        PacketSent = true;
        p = NULL; 
      } else {      
        // resume receiving if we could not send
        PacketSent = true;
      }
    } else {
      MSG("[ERR]Tried to send empty package! TxQueue count=%i\n\r", txQueue.Count);
      PacketSent = true;
    }
  }

  if (PacketSent) {
      PacketSent = false;
      startReceive();
  }

  if (txQueue.Count == 0) {
    #ifndef SILENT
    // wait for serial output to conplete
    delay(10);
    #endif
    signalizeLED_OFF();
    MCU_deepsleep(); // sleep until IRQ
  }
}