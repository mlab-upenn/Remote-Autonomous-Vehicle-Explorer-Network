// -------------------------------
// Wyvern Quadrotor
// On-Board Software
// version: 1.0.2
// date: April 10, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// -------------------------------

// DEFINES

// INCLUDES
#include "wyvern.h"
#include "uart.h"
#include "pwm.h"
//#include "wyvern-rf.h"
//#include "adc.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// CONSTANT VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES
// RF Variables
char local[5] = {0x77, 0x79, 0x76, 0x30, 0x30}; // Wyvern Quadrotor Wyv00
char contr[5] = {0x63, 0x6F, 0x6E, 0x74, 0x72}; // Wyvern Controller
packet_com_t incoming;
packet_inf_t outgoing;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void init_wyvern(void);

// ===========================================================


/*************************************************************************
Function: main()
**************************************************************************/
int main(void)
{
	init_wyvern();
	int duty = 350;
	char input;
	char dutystring[8];
	LED_ucgreen_on();
	for(;;){
		
		TransmitString("Duty = ");
		itoa(duty,dutystring,10);
		TransmitString(dutystring);
		TransmitString("\n\r");
		input = ReceiveByte();                                                                                       
		set(PORTE,6);
		if(input == 'u'){
			if(duty<=2498)
				duty+=2;
		}
		else if(input == 'd'){
			if(duty>=2)
				duty-=2;
		}
		else if(input == '='){
			if(duty<=2400)
				duty+=100;
		}
		else if(input == '-'){
			if(duty>=100)
				duty-=100;
		}
		else if(input == '1'){
			duty=1480;
		}
		else if(input == '2'){
			duty=1300;
		}
		else if(input == '3'){
			duty=1200;
		}
		else if(input == 's'){
			duty=406;
		}
		else if(input == ' '){
			duty = 350;
			clear(PORTE,6);
		}
		set_duty(1,duty);
		toggle(PORTE,2);
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

	// Enable Global Interrupts
	sei();
}


