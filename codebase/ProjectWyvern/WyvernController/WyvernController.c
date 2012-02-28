// --------------------------------------------------
// Wyvern controller board
// Main Program
// version: 1.0.1
// date: April 06, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// --------------------------------------------------

// Includes
#include "wyvern.h"
#include "uart.h"
#include "pwm.h"
#include "wyvern-rf.h"
#include "adc.h"
//#include "commands.h"
#include "controller.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// CONSTANT VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES
char local[5] = {0x63, 0x6F, 0x6E, 0x74, 0x72}; // Wyvern Controller
char wyv00[5] = {0x77, 0x79, 0x76, 0x30, 0x30}; // Wyvern Quadrotor Wyv00

packet_inf_t incoming;
packet_com_t outgoing;

char menuScreen = 'a';
int overflowcounter =0;
char printData = 0;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void init_wyvern(void);
void controllerTransmit(void);
void getJoystick(void);

// ===========================================================

/*************************************************************************
Interrupt Subroutines (ISRs)
**************************************************************************/
ISR(PCINT0_vect){
	if(!check(PINB,4)){
		LED_yellow_toggle();
		// a wireless packet was received
		RFreceive((char*) &incoming);
	}
}

/*
ISR(TIMER0_OVF_vect){
	// Timer1 Clock = 1MHz, 8-bit
	set(TIFR0,TOV0);	// Clear flag
	overflowcounter++;
	if(overflowcounter > 2){
		overflowcounter = 0;
	}
}
*/

/*************************************************************************
Function: main()
**************************************************************************/
int main(void){
	init_wyvern();
	LED_green_on();
	LED_ucgreen_on();
	char input;
	menuScreen = 'a';
	for( ; ; ){
		printMenu();
		input = '0';
		outgoing.command = input;
		if(DataInReceiveBuffer()){
			input = ReceiveByte();
		}
		getJoystick();
		controllerCommand(input);
		controllerTransmit();
		_delay_ms(5);
	if(incoming.battery < 100){
			LED_red_toggle();
			LED_ucred_toggle();
			LED_blue_toggle();
			LED_yellow_toggle();
			LED_green_toggle();
		}
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
void init_wyvern(void){
	// Setup Wyvern Systems
	init_uc();		//UC
	init_uart();	// Serial  Communication
	//init_pwm();		// Motor PWM Control
	init_adc();			// ADC
	// Setup Wyvern RF
	RFsetup(local,max(sizeof(packet_com_t),sizeof(packet_inf_t)));
	set(PCICR,PCIE0); // enable pin-change interrupts
	PCMSK0 =0x00;
	set(PCMSK0, PCINT4); // demask PCINT4
	
	//clear(TCCR0B,CS02);
	//set(TCCR0B,CS01);
	//clear(TCCR0B,CS00);	// Timer0 Clock = System Clock/1024
	//set(TIMSK0,TOIE0);	// Enable Timer0 Overflow Interrupt

	// Enable Global Interrupts
	sei();

	// Force RF Interrupt (Pin Change Interrupt Channel 4) to run once
	clear(PORTB,4);
	if(RFRXdataReady()){
     	RFreceive((char*) &incoming);
	}

	incoming.battery = 150;
}

/*************************************************************************
Function: controllerTransmit()
Purpose:  Transmits the Controller Packet
Input:    None
Returns:  None
**************************************************************************/
void controllerTransmit(void){
	// LED_ucred_toggle();
	RFtransmitUntil((char*) &outgoing,wyv00,1);
}

/*************************************************************************
Function: getJoystick()
Purpose:  Gets the X and Y (Pitch and Roll) Joystick data
Input:    None
Returns:  None
**************************************************************************/
void getJoystick(void){
	int adcval;
	// get right X (Roll) F1
	adcval = 512-get_adc(1);
	if(adcval > -4 && adcval <4){
		outgoing.roll = 0;
	}else{
		outgoing.roll = adcval;
	}
	
	// get right Y (Pitch) F0
	adcval = get_adc(0) - 512;
	if(adcval > -4 && adcval <4){
		outgoing.pitch = 0;
	}else{
		outgoing.pitch = adcval;
	}

	// get left X (Yaw) F5
	adcval = 512-get_adc(5);
	if(adcval > -4 && adcval <4){
		outgoing.yaw = 0;
	}else{
		outgoing.yaw = adcval;
	}
	// get left Y (Throttle) F4
	adcval = get_adc(4);
	if(adcval > 512){
		outgoing.throttle = adcval-512;
	}else{
		outgoing.throttle = 0;
	}
}
