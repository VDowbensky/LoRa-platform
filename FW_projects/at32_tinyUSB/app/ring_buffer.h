
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>

#define CDC_RX_SIZE 512
#define CDC_TX_SIZE 1024
//#define UART1_RX_SIZE 256
//#define UART2_RX_SIZE 512

typedef struct
{
	uint8_t *buf;
	uint16_t size;
	volatile uint16_t head;
	volatile uint16_t tail;
} ringbuf_t;

void ringbuf_init(ringbuf_t *rb,uint8_t *mem,uint16_t size);
void ringbuf_push(ringbuf_t *rb,uint8_t b);
int32_t ringbuf_pop(ringbuf_t *rb);
void ringbuf_flush(ringbuf_t *rb);

extern uint8_t cdc_rx_mem[];
extern ringbuf_t cdc_rx_ring;
extern uint8_t cdc_tx_mem[];
extern ringbuf_t cdc_tx_ring;

#endif
