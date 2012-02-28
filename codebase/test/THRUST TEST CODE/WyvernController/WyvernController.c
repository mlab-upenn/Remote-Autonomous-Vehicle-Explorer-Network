// -------------------------------
// Wyvern controller board
// Main Program
// version: 1.0.1
// date: April 06, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// -------------------------------

// Includes
#include "wyvern.h"
#include "uart.h"
#include "pwm.h"
#include "wyvern-rf.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// CONSTANT VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES
char local[5] = {0x63, 0x6F, 0x6E, 0x74, 0x72}; // Wyvern Controller
char wyv00[5] = {0x77, 0x79, 0x76, 0x30, 0x30}; // Wyvern Quadrotor Wyv00

packet_inf_t incoming;
packet_com_t outgoing;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void init_wyvern(void);

// ===========================================================

/*************************************************************************
Interrupt Subroutines (ISRs)
**************************************************************************/
ISR(PCINT0_vect){
	LED_ucgreen_toggle();
	if(!check(PINB,4)){
		// a wireless packet was received
		RFreceive((char*) &incoming);
		LED_ucred_toggle();
	}
}

/*************************************************************************
Function: main()
**************************************************************************/
int main(void){
	
	init_wyvern();
	LED_green_on();
	outgoing.command = 1;

	for( ; ; ){
		_delay_ms(500);
		LED_ucred_on();
		RFtransmitUntil((char*) &outgoing,wyv00,10);
		LED_ucred_off();
	}
	return 0;
}

/*************************************************************************
Function: init_wyvern()
Purpose:  Runs all initialization functions
		  Enables interrupts
Input:    None
Returns:  None
**************************************************************************/
void init_wyvern(void)
{
	// Setup Wyvern Systems
	init_uc();		//UC
	init_uart();	// Serial  Communication
	init_pwm();		// Motor PWM Control
	
	// Setup Wyvern RF
	RFsetup(local,max(sizeof(packet_com_t),sizeof(packet_inf_t)));
	set(PCICR,PCIE0); // enable pin-change interrupts
	PCMSK0 =0x00;
	set(PCMSK0, PCINT4); // demask PCINT4
	
	// Enable Global Interrupts
	sei();

	// Force RF Interrupt (Pin Change Interrupt Channel 4) to run once
	clear(PORTB,4);
	if(RFRXdataReady()){
     	RFreceive((char*) &incoming);
	}
}
