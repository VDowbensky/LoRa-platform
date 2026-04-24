#ifndef _RADIO_PROC_H_
#define _RADIO_PROC_H_

#include "radio_func.h"
#include "mesh_packet.h"

#define MIN_FREQ_STEP									1
#define MIN_TX_SWEEP_TIME							1
#define MAX_TX_SWEEP_TIME							60000
#define MIN_RX_SWEEP_TIME							10
#define MAX_RX_SWEEP_TIME							60000

/* 
Preset	      Bandwidth (kHz)	      SF	      Data Rate (kbps)	      Link Budget	      Best For
LongFast	    250	                  11	      1.07	                  153dB	            Default
MediumSlow	  250	                  10	      1.95	                  150.5dB	          Better speed
MediumFast	  250	                  9	        3.52	                  148dB	            Fast with good range
ShortSlow	    250	                  8	        6.25	                  145.5dB	          Fast with moderate range
ShortFast	    250	                  7	        10.94	                  143dB	            Very fast, shorter range
ShortTurbo	  500	                  7	        21.88	                  140dB	            Maximum speed, minimum range 
*/

void radio_proc(void);
void process_rx_packet(void);
void radio_startburst(void);
int8_t radio_txsweep(uint32_t start,uint32_t stop,uint32_t step,uint32_t interval,uint8_t stream);
int8_t radio_rxscan(uint32_t start,uint32_t stop,uint32_t step,uint32_t interval,float tr);

extern volatile uint32_t txpacketnumber;
extern volatile uint32_t rxpacketnumber;
extern volatile uint32_t txpacketcount;
extern volatile uint32_t rxpacketcount;
extern bool master;
extern uint32_t inter_packet_delay;
extern volatile uint32_t pkt_timecnt;
extern volatile bool tx_request;
extern bool packet_received;
extern bool crc_error;
extern bool packet_sent;

extern volatile bool sweeptx;
extern volatile bool sweeprx;
extern volatile bool sweepflag;
extern volatile uint32_t sweepcnt;
extern uint32_t sweeptime;

extern uint32_t master_id;
extern uint32_t slave_id;

extern uint8_t opmode;
extern uint8_t prevopmode;

extern volatile bool sec_flag;

#endif
