// --------------------------------------------------
// Wyvern Quadrotor
// Custom Header File
// version: 1.0.4
// date: April 11, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// --------------------------------------------------

/* HEADER FILE INFORMATION
	-Include files
	-Register operations
	-Basic LED operations
	-Register name definitions
	-Functions:
		init_maevarm()
		disableJTAG()
*/

// ===========================================================

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// operations to set, clear, toggle, and check individual register bits
// clear_flag sets the register to 1 (uses =, not &= or |= )
#define set(reg,bit)	  reg |= (1<<(bit))
#define clear(reg,bit)	  reg &= ~(1<<(bit))
#define toggle(reg,bit)	  reg ^= (1<<(bit))
#define check(reg,bit)	  (reg & (1<<(bit)))
#define clear_flag(reg,bit)	reg=(1<<(bit))

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Basic LED Functions
// Red and Green UC LEDs
#define LED_ucgreen_on()		clear(PORTE,2)
#define LED_ucgreen_off()		set(PORTE,2)
#define LED_ucgreen_toggle()	toggle(PORTE,2)
#define LED_ucred_on()		clear(PORTE,6)
#define LED_ucred_off()		set(PORTE,6)
#define LED_ucred_toggle()	toggle(PORTE,6)

// Red, Green, Yellow, and Blue LEDs
#define LED_red_on()		clear(PORTD,4)
#define LED_red_off()		set(PORTD,4)
#define LED_red_toggle()	toggle(PORTD,4)
#define LED_green_on()		clear(PORTD,5)
#define LED_green_off()		set(PORTD,5)
#define LED_green_toggle()	toggle(PORTD,5)
#define LED_yellow_on()		clear(PORTD,6)
#define LED_yellow_off()		set(PORTD,6)
#define LED_yellow_toggle()	toggle(PORTD,6)
#define LED_blue_on()		clear(PORTD,7)
#define LED_blue_off()		set(PORTD,7)
#define LED_blue_toggle()	toggle(PORTD,7)

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Set up Atmega32U4 specific register names
// (In case non-32U4 code is used)
#define UBRRH UBRR1H
#define UBRRL UBRR1L
#define UDR UDR1
#define USBS USBS1
#define UCSRA UCSR1A
#define UDRE UDRE1
#define RXC RXC1
#define UCR UCSR1B
#define UCSRB UCSR1B
#define RXEN RXEN1
#define TXEN TXEN1
#define RXCIE RXCIE1
#define UDRIE UDRIE1
#define UCSRC UCSR1C
#define UCSZ0 UCSZ10
#define UCSZ1 UCSZ11
#define UCSRC_SELECT 0

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// packet struct
typedef struct{
	char command;
	int pitch;
	int roll;
	int yaw;
	int throttle;
}packet_com_t;

typedef struct{
	uint16_t yaw;
	uint16_t pitch;
	uint16_t roll;
	uint16_t omegazero;
	uint16_t omegaone;
	uint16_t omegatwo;
}packet_razordata_t;

typedef struct{
	int yaw;
	int pitch;
	int roll;
	int altitude;
	int battery;
	int p;
	int i;
	int d;
	int pidband;
	char stat0;
	char stat1;
	char stat2;
	char stat3;
	char stat4;
}packet_inf_t;

/*************************************************************************
Function: clockSet()
Purpose:  Sets clock to 8MHz
Input:    None
Returns:  None
**************************************************************************/
void clockSet(){
	CLKPR = (1<<CLKPCE);
	CLKPR = 0;
}


#define F_CPU 8000000UL

/*************************************************************************
Function: disableJTAG()
Purpose:  Allows access to F4-F7 as normal port pins
		  to disable the JTAG system, set the JTD bit in MCUCR twice within 4 clock cycles
		  |= is too slow, so we're going to write the entire register
		  fortunately, all the other bits in MCUCR should be 0, so this is fine
Input:    None
Returns:  None
**************************************************************************/
void disableJTAG(){
	MCUCR = (1 << JTD);
	MCUCR = (1 << JTD);
}

/*************************************************************************
Function: init_uc()
Purpose:  Sets up the microcontroller board
			-Sets system clock to 8MHz
			-Sets LED Ports (LEDs OFF)
Input:    None
Returns:  None
**************************************************************************/
void init_uc(){
	// Set system clock to 8MHzs
	clockSet();

	// Set up Red and Green UC LED Ports
	set(DDRE,2);
	set(DDRE,6);
	set(PORTE,2);
	set(PORTE,6);

	// Set up Red, Green, Yellow, and Blue LEDs
	set(DDRD,4);
	set(DDRD,5);
	set(DDRD,6);
	set(DDRD,7);
	set(PORTD,4);
	set(PORTD,5);
	set(PORTD,6);
	set(PORTD,7);

	// Enable Pins F4-F7
	disableJTAG();
}
