#include "retarget.h"

/* Receive buffer */

static volatile int     rxReadIndex  = 0;       /**< Index in buffer to be read */
static volatile int     rxWriteIndex = 0;       /**< Index in buffer to be written to */
volatile int     rxCount      = 0;       /**< Keeps track of how much data which are stored in the buffer */
//uint8_t rxBuffer[RXBUFSIZE];    /**< Buffer to store data */
//uint8_t txBuffer[TXBUFSIZE];    /**< Buffer to store data */
static bool initialized = false;    /**< Initialize UART/LEUART */

extern uint32_t Receive_length;
volatile int txCount = 0;

//uint16_t usb_tx_write_index = 0;
//uint16_t usb_tx_read_index = 0;

//extern volatile uint32_t packet_sent,packet_receive;

/**************************************************************************//**
 * @brief Intializes UART/LEUART
 *****************************************************************************/
void RETARGET_Init(void)
{
#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
  setvbuf(stdout, NULL, _IONBF, 0);   /*Set unbuffered mode for stdout (newlib)*/
#endif
  initialized = true;
}



#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
    RETARGET_WriteChar((uint8_t)ch);
    return ch;
}


/**************************************************************************//**
 * @brief Receive a byte from USART/LEUART and put into global buffer
 * @return -1 on failure, or positive character integer on sucesss
 *****************************************************************************/

int RETARGET_ReadChar(void)
{
	return ringbuf_pop(&cdc_rx_ring);
}


/**************************************************************************//**
 * @brief Transmit single byte to USART/LEUART
 * @param c Character to transmit
 * @return Transmitted character
 *****************************************************************************/

int RETARGET_WriteChar(char c)
{
	ringbuf_push(&cdc_tx_ring,c);
	return c;
}

//int putc(int c, FILE * stream)
int stdout_putchar(int c, FILE * stream)
{
	RETARGET_WriteChar(c);
	return c; //return the character written to denote a successfull write
}

//int getc(FILE * stream)
int stdin_getchar(FILE * stream)
{
	char c = RETARGET_ReadChar();
	return c;
}

