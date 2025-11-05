#include "retarget.h"

/* Receive buffer */

static volatile int     rxReadIndex  = 0;       /**< Index in buffer to be read */
static volatile int     rxWriteIndex = 0;       /**< Index in buffer to be written to */
volatile int     rxCount      = 0;       /**< Keeps track of how much data which are stored in the buffer */
volatile uint8_t rxBuffer[RXBUFSIZE];    /**< Buffer to store data */
volatile uint8_t txBuffer[TXBUFSIZE];    /**< Buffer to store data */
static bool initialized = false;    /**< Initialize UART/LEUART */

extern uint32_t Receive_length;
volatile int txCount = 0;

//extern volatile uint32_t packet_sent,packet_receive;
extern usbd_core_type usb_core_dev;

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
  int c = -1;

  if (initialized == false) RETARGET_Init();
  if (rxCount > 0)
  {
    c = rxBuffer[rxReadIndex];
    rxReadIndex++;
    if (rxReadIndex == RXBUFSIZE) rxReadIndex = 0;
    rxCount--;
  }
	else rxReadIndex = 0;
  return c;
}


/**************************************************************************//**
 * @brief Transmit single byte to USART/LEUART
 * @param c Character to transmit
 * @return Transmitted character
 *****************************************************************************/

//int _write(int file, char *ptr, int len) { 
//    CDC_Send_DATA((uint8_t*) ptr, len); return len; 
//}


int RETARGET_WriteChar(char c)
{
  if (initialized == false) RETARGET_Init();
	txBuffer[txCount] = c;
	txCount++;
	if(txCount == RXBUFSIZE)
	{
		txCount = 0;
	}
	return c;
	//return -1;
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

