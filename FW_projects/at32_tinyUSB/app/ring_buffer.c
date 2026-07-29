
#include "ring_buffer.h"

uint8_t cdc_rx_mem[CDC_RX_SIZE];
uint8_t cdc_tx_mem[CDC_TX_SIZE];
ringbuf_t cdc_rx_ring;
ringbuf_t cdc_tx_ring;

void ringbuf_init(ringbuf_t *rb,uint8_t *mem,uint16_t size)
{
	rb->buf = mem;
	rb->size = size;
	rb->head = 0;
	rb->tail = 0;
}

void ringbuf_push(ringbuf_t *rb,uint8_t b)
{
	uint16_t next = (rb->head + 1) % rb->size;

	if(next != rb->tail)
	{
		rb->buf[rb->head] = b;
		rb->head = next;
	}
}

int32_t ringbuf_pop(ringbuf_t *rb)
{
	if(rb->head == rb->tail) return -1;

	uint8_t v = rb->buf[rb->tail];
	rb->tail = (rb->tail + 1) % rb->size;
	return v;
}

void ringbuf_flush(ringbuf_t *rb)
{
	rb->head = 0;
	rb->tail = 0;
}
