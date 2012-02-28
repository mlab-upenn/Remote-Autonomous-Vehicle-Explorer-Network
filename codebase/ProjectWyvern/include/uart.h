// --------------------------------------------------
// WyvernMaEvArM microcontroller board
// UART
// version: 1.0.1
// date: March 28, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// --------------------------------------------------

// STATIC VARIABLES
#define UART_RX_BUFFER_SIZE 512	// 2,4,8,16,32,64,128 or 256 bytes
#define UART_TX_BUFFER_SIZE 512	// 2,4,8,16,32,64,128 or 256 bytes
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
	#error RX buffer size is not a power of 2
#endif
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif
// Baudrate - Set Baud Rate to 38,400 given U2X1 = 0
// 38,400 -> 0x0c		250,000 -> 0x01;
unsigned int baud = 0x01;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES
static unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void init_uart(void);
unsigned char ReceiveByte(void);
void TransmitByte(unsigned char data);
void TransmitString(const char *s );
void TransmitInt(int i );
unsigned char DataInReceiveBuffer(void);
int uart_available(void);
void uart_flush(void);

// ===========================================================

/*************************************************************************
Function: init_uart()
Purpose:  Sets up UART communication
		  8-bit transmission, no parity, 1-stop bit, U2X1 = 0
Input:
Returns:
**************************************************************************/
void init_uart(void)
{
	//UCSR1A |= 0x02;  // Uncomment to set U2X1 = 1
	// Set baudrate
	UBRR1L = baud;
	// Enable Rx and Tx
	UCSR1B |=(1<<RXEN1)|(1<<TXEN1);
	// Set to 8 bit tranmission, 1 stop bit
	UCSR1C |=(1<<UMSEL11)|(1<<UCSZ10)|(1<<UCSZ11);
	// Enable USART Receive Complete Interrupt
	UCSR1B |=(1<<RXCIE1);
	// Flush receive buffer
	UART_RxTail = 0;
	UART_RxHead = 0;
	UART_TxTail = 0;
	UART_TxHead = 0;
}

/*************************************************************************
Function: ISR(USART1_RX_vect)
Purpose:  RXC (Receive Complete) Interrupt
		  Interrupt Vector 26
Input:
Returns:
**************************************************************************/
ISR(USART1_RX_vect)
{
	unsigned char data;
	unsigned char tmphead;
	// Read the received data
	data = UDR1;
	/// Calculate buffer index
	tmphead = ( UART_RxHead + 1 ) & UART_RX_BUFFER_MASK;
	// Store new index
	UART_RxHead = tmphead;
	if ( tmphead == UART_RxTail )
	{
		//ERROR! Receive buffer overflow
		uart_flush();
	}
	// Store received data in buffer
	UART_RxBuf[tmphead] = data;
}

/*************************************************************************
Function: ISR(USART1_UDRE_vect)
Purpose:  UDRE (USART Data Register Empty) Interrupt
		  Interrupt Vector 27
Input:
Returns:
**************************************************************************/
ISR(USART1_UDRE_vect)
{
	unsigned char tmptail;
	// Check if all data is transmitted
	if ( UART_TxHead != UART_TxTail )
	{
		// Calculate buffer index
		tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;
		// Store new index
		UART_TxTail = tmptail;
		// Start transmition
		UDR1 = UART_TxBuf[tmptail];
	}
	else
	{
		// Disable UDRE interrupt
		UCSR1B &= ~(1<<UDRIE);
	}
}

/*************************************************************************
Function: ReceiveByte()
Purpose:  Returns the next byte in the Rx buffer
		  Waits for incoming data if nothing in buffer
Input:
Returns:  Next char in Rx buffer
**************************************************************************/
unsigned char ReceiveByte(void)
{
	unsigned char tmptail;
	// Wait for incoming data
	while ( UART_RxHead == UART_RxTail );
	// Calculate buffer index
	tmptail = ( UART_RxTail + 1 ) & UART_RX_BUFFER_MASK;
	// Store new index
	UART_RxTail = tmptail;
	 // Return data
	return UART_RxBuf[tmptail];
}

/*************************************************************************
Function: TransmitByte()
Purpose:  Places data into Tx buffer and enable UDRE interrupt to transmit
		  Waits for free space if no room in buffer
Input:    Data to transmit
Returns:
**************************************************************************/
void TransmitByte(unsigned char data)
{
	unsigned char tmphead;
	tmphead=0;
	// Calculate buffer index
	tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;
	// Wait for free space in buffer
	while ( tmphead == UART_TxTail );
	// Store data in buffer
	UART_TxBuf[tmphead] = data;
	// Store new index
	UART_TxHead = tmphead;
	// Enable UDRE interrupt
	UCSR1B |= (1<<UDRIE1);
}

/*************************************************************************
Function: TransmitString()
Purpose:  Transmit string to UART
Input:    String to be transmitted
Returns:
**************************************************************************/
void TransmitString(const char *s )
{
    while (*s)
      TransmitByte(*s++);
}

/*************************************************************************
Function: TransmitInt()
Purpose:  Transmit integer to UART
Input:    Integer to be transmitted
Returns:
**************************************************************************/
void TransmitInt(int i )
{
	char s[8];
	itoa(i,s,10);
	TransmitString(s);
}

/*************************************************************************
Function: DataInReceiveBuffer()
Purpose:  Determine if there is data in the Rx buffer
Input:
Returns:  Return 0 (FALSE) if Rx buffer is empty
**************************************************************************/
unsigned char DataInReceiveBuffer(void)
{
	return ( UART_RxHead != UART_RxTail );
}

/*************************************************************************
Function: uart_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
int uart_available(void)
{
        return (UART_RX_BUFFER_MASK + UART_RxHead - UART_RxTail) % UART_RX_BUFFER_MASK;
}

/*************************************************************************
Function: uart_flush()
Purpose:  Flush bytes waiting in the receive buffer.  (Actually ignores them)
Input:
Returns:
**************************************************************************/
void uart_flush(void)
{
        UART_RxHead = UART_RxTail;
}

