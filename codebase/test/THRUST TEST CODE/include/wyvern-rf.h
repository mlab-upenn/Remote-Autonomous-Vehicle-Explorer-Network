/*
			v1.0

Wyvern RF header for MaEvArM
Based on code by Joe Romano and J. Fiene

Provides the following functions:

 to set the board, define either MAEVARM_RF_GREEN or MAEVARM_RF_RED before including
 ( #define MAEVARM_RF_GREEN
 ( - or -
 ( #define MAEVARM_RF_RED
 
 to set the packet size, define PACKET_SIZE before including - must be less than 32
 ( #define PACKET_SIZE 17
 
	void RFsetup(char * recvAddr)
		// initialize the wireless node and start receiving
		- recvAddr is a pointer to a 5-element char array
 
	void RFtransmit( char* txDat, char* destAddr)
		// suspend receive mode and transmit a char array to a specific wireless node
		// will only send one packet, and does not check for receipt
		- txDat is a pointer to an n-element char array
		- destAddr is a pointer to a 5-element char array

	int RFtransmitUntil( char* txDat, char* destAddr, int txTries)
		// suspend receive mode and transmit a char array to a specific wireless node
		// if no acknowledgement is returned from the other node, it will retransmit up to txTries times
		// returns 0 if unsuccessful, or the number of tries it took before success
		- txDat is a pointer to an n-element char array
		- destAddr is a pointer to a 5-element char array
		- txTries is the maximum number of retransmit attempts
		
	int RFRXdataReady()
		// check to see if a message has been received
		- returns 1 if data, 0 if empty

	void RFreceive( char* retDat)
		// read the latest data from receive buffer
		- retDat - pointer to an n-element char array storage container 
*/

// make sure we don't include this file twice
#ifndef _RF24L01_HELPER
#define _RF24L01_HELPER

//basic WINAVR includes
#include <avr/io.h>
#include <util/delay.h>

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0        
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

// Pins that the 24L01 connects to the MEAMAVR board on
#define P_RF_CS			PORTC7
#define PORT_CS			PORTC
#define	DDR_P_RF_CS		DDC7
#define DDR_RF_CS		DDRC

#define P_RF_CE			PORTB0
#define	PORT_CE			PORTB
#define DDR_P_RF_CE		DDB0
#define	DDR_RF_CE		DDRB

// Pins that are used for SPI on the MEAMAVR board
#define PORT_SPI    PORTB		// the port that corresponds to the SPI pins of our 32UF AVR chip
#define DDR_SPI     DDRB		// the data register that corresponds to the SPI of our 32UF AVR chip
#define DD_MISO     DDB3		// the data register that corresponds with the MISO SPI pin of the 32UF AVR chip
#define DD_MOSI     DDB2		// the data register that corresponds with the MOSI SPI pin of the 32UF AVR chip
#define DD_SCK      DDB1		// the data register that corresponds with the SCK SPI pin of the 32UF AVR chip
#define DD_SS       DDB0		// the data register that corresponds with the SS SPI pin of the 32UF AVR chip

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
char PACKET_SIZE;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Config Options - set all options to default values
#define CONFIG_VAL (1<<EN_CRC) | (1<<MASK_TX_DS) | (1<<MASK_MAX_RT)

// set up the spi port in master mode with the polarity options (etc.) that the 24L01 requires. Also set DDR of the CE pin of rf module.
void SPIsetup(){
	//Make sure Power Save didn't turn SPI off
 	PRR0 &= ~(1 << PRSPI);

	// Set MOSI and SCK and SS output, all others input. SS MUST be config. to output 
	// Also configure the CE pin of the rf module as an output
	DDR_SPI |= (1 << DD_MOSI) | (1<<DD_SCK) | (1<<DD_SS);

	// Enable SPI, Master, set clock rate sys_clk/128
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0);

	//PAUSE to let the wireless chip go through initialization 
	_delay_ms(200);
}

// function to write a data register on the 24L01 RF chip. 
// writeReg - the register we want to write data to
// writeDat - the storage container for the data we plan on writing
// numBytes - the number of bytes to write. !! The writeDat storage container must be at least this size in bytes (char's)
void RFwriteReg(char writeReg, char* writeDat, int numBytes){

	// turn the CS channel low to begin transmission
	PORT_CS &= ~(1<<P_RF_CS);

	// send 1 byte out the spi port to request write of writeReg
	// in order to do this we add 0x20 to our write register as per the format of the 23L01 datasheet
	SPDR = W_REGISTER + writeReg;

	// wait for transmission to finish
	while(!(SPSR & (1<<SPIF) ) );

	// write the number of bytes we want from writeDat
	int i;
	for(i = 0; i < numBytes; i++)
	{
		// send 1 byte out the spi port. this is the data we want to write to the RF chip
		SPDR = writeDat[i];

		// wait for transmission to finish
		while(!(SPSR & (1<<SPIF) ) );
	}

	// turn the CS channel high to end transmission
	PORT_CS |= (1<<P_RF_CS);
}





// function to fill the transfer (TX) buffer with data we wish to transmit wirelessly
// txDat - an appropriately sized storage container containing the data we plan on pushing into the TX register
// numBytes - the number of bytes to push in back. !! The txDat storage container must be at least this size in bytes (char's)
void RFfillTransferBuffer( char* txDat, int numBytes ){
	// turn the CS channel low to begin transmission
	PORT_CS &= ~(1<<P_RF_CS);

	// send 1 byte out the spi port that indicates we want to fill the TX buffer
	SPDR = W_TX_PAYLOAD;

	// wait for transmission to finish
	while(!(SPSR & (1<<SPIF) ) );

	// write the number of bytes we want to write out to the TX buffer
	int i;
	for(i = 0; i < numBytes; i++)
	{
		// send 1 byte out the spi port. this is the data we want to write to the RF chip
		SPDR = txDat[i];

		// wait for transmission to finish
		while(!(SPSR & (1<<SPIF) ) );
	}

	// turn the CS channel high to end transmission
	PORT_CS |= (1<<P_RF_CS);
}


// function to read a data register from the 24L01 RF chip. 
// readReg - the register we want to read data from
// retDat - an appropriately sized storage container for the data we plan on getting back
// numBytes - the number of bytes to read back. !! The retDat storage container must be at least this size in bytes (char's)
void RFreadReg(char readReg, char* retDat, int numBytes){

	// turn the CS channel low to begin transmission
	PORT_CS &= ~(1<<P_RF_CS);

	// send 1 byte out the spi port to request write of writeReg
	SPDR = R_REGISTER + readReg;

	// wait for transmission to finish
	while(!(SPSR & (1<<SPIF) ) );

	// read the number of bytes we intend to receive into retDat
	int i;
	for(i = 0; i < numBytes; i++){
		// send 1 byte out the spi port. this is a dummy send just to read the incoming MISO data from our read request above
		SPDR = 0xFF;

		// wait for transmission to finish
		while(!(SPSR & (1<<SPIF) ) );

		// read off the contents of our return data that is in SPDR (the value we requested to read). This came from MISO
		retDat[i] = SPDR;
	}

	// turn the CS channel high to end transmission
	PORT_CS |= (1<<P_RF_CS);
}


// put the chip into receiving mode
void RFsetRxAddr(char * recvAddr, int numBytes){
	RFwriteReg(RX_ADDR_P1,recvAddr,numBytes);
}


// flush the tx buffer
void RFflushTXBuffer(){

	// turn the CS channel low to begin transmission
	PORT_CS &= ~(1<<P_RF_CS);

	// send 1 byte out the spi port to request write of writeReg
	SPDR = FLUSH_TX;

	// wait for transmission to finish
	while(!(SPSR & (1<<SPIF) ) );

	// turn the CS channel high to end transmission
	PORT_CS |= (1<<P_RF_CS);
}

// flush the rx buffer
void RFflushRXBuffer(){

	// turn the CS channel low to begin transmission
	PORT_CS &= ~(1<<P_RF_CS);

	// send 1 byte out the spi port to request write of writeReg
	SPDR = FLUSH_RX;

	// wait for transmission to finish
	while(!(SPSR & (1<<SPIF) ) );

	// turn the CS channel high to end transmission
	PORT_CS |= (1<<P_RF_CS);
}

// put the chip into receiving mode
void RFstartReceiving(){

	char writeDat[1] = { CONFIG_VAL | (1<<PWR_UP) | (1<<PRIM_RX) };

	// turn the PWR_UP and PRIM_RX bits high
	RFwriteReg( CONFIG, writeDat, 1);

	// wait a millisecond 
	_delay_ms(2);

	// turn pin CE high
	PORT_CE |= (1<<P_RF_CE);
}

// take the chip out of receiving mode
void RFstopReceiving(){

	// turn pin CE low
	PORT_CE &= ~(1<<P_RF_CE);
}

// transmit txDat wirelessly and repeat until we receive verification that the packet was received. Returns chip to receive state when done
char RFtransmitUntil( char* txDat, char* destAddr, char txTries){

	char txAttempt = 0;
	char success = 0;
	char tempWrite[1] = { 0x00 };
	
	// take out of receiving mode
	RFstopReceiving();

	
	// repeat transmission until the TX bit goes high (verifies receipt) or we've tried txTries times
	while( (!success) &&  (txAttempt < txTries)  ){

		RFflushTXBuffer();
		
		// clear the TX transmission bit
		tempWrite[0] = (1<<TX_DS);
		RFwriteReg(STATUS, tempWrite, 1);


		// increment the attempt counter
		txAttempt++; 		
		
		// set up the destination transmit address
		RFwriteReg(TX_ADDR,destAddr,5);

		// set up the destination recive address for auto-acknowledgement
		RFwriteReg(RX_ADDR_P0,destAddr,5);

		// setup our write data to configure the chip into TX mode
		char writeDat[1] = { CONFIG_VAL | (1<<PWR_UP) };

		// turn the PWR_UP high and PRIM_RX bit low for transmitting
		RFwriteReg( CONFIG, writeDat, 1);

		//write data to TX register for outputting
		RFfillTransferBuffer(txDat,PACKET_SIZE);

		// turn pin CE high
		PORT_CE |= (1<<P_RF_CE);

		// wait 2 millisecond to make sure transfer fires
		_delay_ms(2);

		// end transmission by pulling CE low
		PORT_CE &= ~(1<<P_RF_CE);
		
		// delay so we have time to receive an ACK response
		_delay_ms(4);
		
		// temp variable to store our STATUS register state
		char tempStatus[1] = {0x00};
		RFreadReg(STATUS,tempStatus,1);

		// check for acknowledgement from the receiving node
		//success = tempStatus[0];
		success = ( tempStatus[0] & (1<<TX_DS));

		// clear any MAX_RT bits transmission bit
		tempWrite[0] = (1<<MAX_RT);
		RFwriteReg(STATUS, tempWrite, 1);	
	}
		
	// if the TX bit is high clear the TX transmission bit
	if(success){
		tempWrite[0] =  (1<<TX_DS);
		RFwriteReg(STATUS, tempWrite, 1);
	}

	// turn receiving mode back on 
	RFstartReceiving();

	if(success){
		return 1;
	}
	else{
		return 0;
	}
}


// transmit txDat wirelessly and leave the chip in receive mode when done. Function only fires 1 packet and does not check if data was received.
void RFtransmit( char* txDat, char* destAddr){
	RFtransmitUntil(txDat,destAddr,1);
}


int RFRXdataReady(){
	// dummy write data
	char retDat[1] = {0xFF};

	RFreadReg(STATUS,retDat,1);

	if( retDat[0] & (1<<RX_DR) ){
		return 1;
	}

	else{
		return 0;
	}

}


int RFRXbufferEmpty(){
	// dummy write data
	char retDat[1] = {0xFF};

	RFreadReg(STATUS,retDat,1);

	if( retDat[0] & 0x0E ){
		return 1;
	}

	else{
		return 0;
	}
}

// read data out of the receive FIFO and turn off the data-ready flag RX_DR if the buffer is empty
// this function stores the previous value of the CONFIG register, which indicated whether we were transmitting or receiving prior to this function call, and restores this state after reading
// NOTE: it is not possible to receive or transmit new packets while readings, all packets will be lost during this time
void RFreadRXFIFO( char* retDat){
	// take out of receiving mode
	RFstopReceiving();

	// dummy write data
	retDat[0] = 0xFF;

	// turn the CS channel low to begin transmission
	PORT_CS &= ~(1<<P_RF_CS);

	// send 1 byte out the spi port that indicates we want to read the RX FIFO
	SPDR = R_RX_PAYLOAD;

	// wait for transmission to finish
	while(!(SPSR & (1<<SPIF) ) );

	// read the number of bytes we want to read out to the RX buffer
	int i;
	for(i = 0; i < PACKET_SIZE; i++){
		// read 1 byte out the spi port
		SPDR = retDat[i];

		// wait for transmission to finish
		while(!(SPSR & (1<<SPIF) ) );

		// store the returned data
		retDat[i] = SPDR;
	}


	// turn the CS channel high to end transmission
	PORT_CS |= (1<<P_RF_CS);

	_delay_ms(2);

	// if there is no new data ready to be read then turn the RX_DR data ready pin low
	if(RFRXbufferEmpty()){
		char writeDat[1] = { (1<<RX_DR) };
		RFwriteReg(STATUS,writeDat,1);
	}

	// put back in receiving mode
	RFstartReceiving();
}


int RFRXbufferFull(){
	// dummy write data
	char retDat[1] = {0xFF};

	RFreadReg(FIFO_STATUS,retDat,1);

	if(retDat[0] & (1<<RX_FULL)){
		return 1;
	}else{
		return 0;
	}
}


// clear out the FIFO and return the last received packet
void RFreceive(char * buffer){

	clear(PCICR,PCIE0); // disable pin-change interrupts

	while(RFRXdataReady()){
		RFreadRXFIFO(buffer);
	}

	set(PCICR,PCIE0); // enable pin-change interrupts
}


// function to setup the rf chip registers for communication and boot it up in receiving mode
void RFsetup(char * recvAddr, char packet_size){
	PACKET_SIZE = packet_size;
	_delay_ms(100);
	
	//setup the SPI port for use with the 24L01 chip
	SPIsetup();

	//setup the RF CE and CS pins as outputs
	
	DDR_RF_CS|=(1<<DDR_P_RF_CS);

	DDR_RF_CE |=(1<<DDR_P_RF_CE);

	//set the config register up
	char writeDat[1] = {CONFIG_VAL};
	RFwriteReg(CONFIG,writeDat,1);

	//enable PACKET_SIZE byte collection on pipe 1
	writeDat[0]  = PACKET_SIZE;
	RFwriteReg(RX_PW_P1,writeDat,1);

	//turn on auto acknowledgement on first two pipes
	writeDat[0] = 0x03;
	RFwriteReg(EN_AA,writeDat,1);

	//turn off auto retransmit
	writeDat[0] = 0x00;
	RFwriteReg(SETUP_RETR,writeDat,1);		

	//enable receive on pipe 0 and 1  (this happens by default)
	//writeDat[0] = (1<<ERX_P0) | (1<<ERX_P1);
	//RFwriteReg(EN_RXADDR,writeDat,1);	
	
	//setup address size as 5 bytes (probably deafult setup is already OK)
	//writeDat[0] = 0x03;
	//RFwriteReg(SETUP_AW,writeDat,1);	

	// write receive address to pipe1
	RFsetRxAddr(recvAddr, 5);

	// turn the receiver on
	RFstartReceiving();
}

#endif
