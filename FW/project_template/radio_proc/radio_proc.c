#include "radio_proc.h"
#include "flash.h"

volatile uint32_t txpacketnumber = 0;
volatile uint32_t rxpacketnumber = 0;
volatile uint32_t txpacketcount = 0;
volatile uint32_t rxpacketcount = 0;
bool master = false;
uint32_t inter_packet_delay = 100;
volatile uint32_t pkt_timecnt;
volatile bool tx_request = false;

uint8_t opmode = 0;
uint8_t prevopmode = 0;
uint8_t txmode;

uint32_t master_id;
uint32_t slave_id;
meshtastic_pkt_t txmessage;
meshtastic_pkt_t rxmessage;
void prepareTxPacket(void);
void printcrcerror(void);


//radio events handler
void radio_proc(void)
{
  //static uint8_t phase = 0;
  
  if(tx_request)
  {
    tx_request = false;
    prepareTxPacket();
    txled_on();
    radio_sendpacket(radio_txbuffer);
  }

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
	//if(crc_error)
	//{
	//	crc_error = false;
	//	printcrcerror();
	//}
	//else
	//{
		//retrieve packet params
		radio_getpktstatus(&pktstatus);
		//copy packet to buffer
		radio_getpacket(radio_rxbuffer);
		//process buffer
		decode_meshtastic_packet();
		print_meshtastic_packet();
	//}
}


void radio_startburst(void)
{
  master = true;
  txpacketnumber = 1;
  //start TX timer here
	pkt_timecnt = 0; //to systick
  tx_request = true;
}

void prepareTxPacket(void)
{
	uint8_t i;
	
	txmessage.destination_id = slave_id;
	txmessage.sender_id = radioconfig.id;
	txmessage.packet_id = txpacketnumber;
	txmessage.relay_node = 0;
	txmessage.next_hop = 1;
	txmessage.flags = (1 << HOPSTART_POS) | (3 << HOPLIMIT_POS); //for test
	for(i = 0; i < 16; i++) txmessage.payload[i] = i;
	txlen = 16 + i;
	//printf("Payload: ");
	memcpy((void*)radio_txbuffer,(void*)&txmessage,txlen);
	printf("TX: %d\r\n",txpacketnumber);
}

void printcrcerror(void)
{
  printf("FERR\r\n");
}

void LORA_IRQHandler(void)
{
	
	uint16_t irqstatus;

  //read SX126x status
  irqstatus = SX126X_GetIrqStatus();
  SX126X_ClearIrqStatus(SX126X_ALL_IRQMSK);

  if(irqstatus & SX126X_TXDONE_IRQMSK) packet_sent = true;
  if(irqstatus & SX126X_RXDONE_IRQMSK) packet_received = true;
  if(irqstatus & SX126X_CRCERR_IRQMSK) crc_error = true;
}







