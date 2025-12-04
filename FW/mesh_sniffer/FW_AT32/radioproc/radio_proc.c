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

//Frequency sweep
uint32_t startfreq;
uint32_t stopfreq;
//uint32_t prevfreq;
//uint32_t currfreq;
uint32_t sweepstep;
uint8_t sweepstream;
volatile uint32_t sweepcnt;
uint32_t sweeptime;
volatile bool sweeptx = false;
volatile bool sweeprx = false;
volatile bool sweepflag = false;
float rssitr;

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
	
  if(sweepflag) 
	{
		sweepflag = false;
		
		if(sweeptx)
		{
			currfreq+= sweepstep;
			if(currfreq >= stopfreq) currfreq = startfreq;
			//printf("freq:%d,stream:%d\r\n",currfreq,txmode);
			radio_setopmode(RADIO_OPMODE_FS);
			radio_set_freq(currfreq);
			radio_stream(txmode);
			//printf("freq:%d,stream:%d\r\n",currfreq,txmode);
		}
		if(sweeprx)
		{
			float rssi;
			radio_getrssi(&rssi);
			if(rssi > rssitr)
			{
				//rssi_peak = rssi;
				printf("Freq=%d,RSSI=%.1f dBm\r\n",currfreq,rssi); //temporary; send to buffer instead
				//display_rssi(currfreq/1000,rssi_peak);
				//beep(2000,100);
			}
			radio_setopmode(RADIO_OPMODE_FS);
			currfreq += sweepstep;
			if(currfreq >= stopfreq) currfreq = startfreq;
			radio_set_freq(currfreq);
			radio_rx();
		}
	}
}

void process_rx_packet(void)
{
	//if(crc_error)
	//{
		//crc_error = false;
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

int8_t radio_txsweep(uint32_t start,uint32_t stop,uint32_t step,uint32_t interval,uint8_t stream)
{
	if(stop < (start+step)) return RADIO_INVALID_PARAMETER;
	if(step < MIN_FREQ_STEP) return RADIO_INVALID_PARAMETER;	
	if((interval < MIN_TX_SWEEP_TIME) || (interval > MAX_TX_SWEEP_TIME)) return RADIO_INVALID_PARAMETER;
	if(stream > 2) return RADIO_INVALID_PARAMETER;
	if(stream == 0)
	{
		txmode = 0;
		sweeptx = false;
		currfreq = prevfreq;
		radio_set_freq(currfreq);
		txled_off();
		radio_rx();
		return RADIO_OK;
	}
	else
	{
		prevfreq = currfreq;
		startfreq = start;
		stopfreq = stop;
		sweepstep = step;
		txmode = stream;
		currfreq = startfreq;
		radio_set_freq(currfreq);
		txled_on();
		int8_t err = radio_stream(stream);
		if(err != 0) return err;
		sweeptime = interval;
		sweepcnt = sweeptime;
		sweeptx = true;
		return RADIO_OK;
	}
}

int8_t radio_rxscan(uint32_t start,uint32_t stop,uint32_t step,uint32_t interval,float tr)
{
	if(stop < (start+step)) return RADIO_INVALID_PARAMETER;
	if(step < MIN_FREQ_STEP) return RADIO_INVALID_PARAMETER;
	if((interval < MIN_RX_SWEEP_TIME) || (interval > MAX_RX_SWEEP_TIME)) return RADIO_INVALID_PARAMETER;
	//SSD1306_Clear(0);
	if(tr >= 0)
	{
//		//GUI_ShowString(0,0,"IDLE       ",16,1);
		sweeprx = false;
		radio_set_freq(prevfreq);
		currfreq = prevfreq;
	}
	else
	{
		//GUI_ShowString(0,0,"SCAN       ",16,1);
		prevfreq = currfreq;
		startfreq = start;
		stopfreq = stop;
		sweepstep = step;
		rssitr = tr;
		currfreq = startfreq;
		radio_set_freq(currfreq);
		sweeptime = interval;
		sweepcnt = sweeptime;
		sweeprx = true;
	}
	radio_rx();
	return RADIO_OK;
}


