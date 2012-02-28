// --------------------------------------------------
// Wyvern Quadrotor
// On-Board Software
// version: 1.0.2
// date: April 10, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// --------------------------------------------------

// DEFINES

// INCLUDES
#include "wyvern.h"
#include "uart.h"
#include "pwm.h"
#include "wyvern-rf.h"
#include "adc.h"
#include "pid.h"
#include "commands.h"
//#include "usb_serial.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// CONSTANT VARIABLES


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES
// RF Variables
char local[5] = {0x77, 0x79, 0x76, 0x30, 0x30}; // Wyvern Quadrotor Wyv00
char contr[5] = {0x63, 0x6F, 0x6E, 0x74, 0x72}; // Wyvern Controller
packet_com_t initial;
packet_com_t incoming;
packet_inf_t outgoing;
packet_razordata_t razordata;
int overflowcounter=0;
int overflowcounterbattery = 0;
extern int thrustFloor;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void init_wyvern(void);
void ReceiveRazorData(void);
void batteryVoltage(void);

// ===========================================================

/*************************************************************************
Interrupt Subroutines (ISRs)
**************************************************************************/

ISR(PCINT0_vect)
{
	// change was high-to-low
	if(!check(PINB,4)){ 
		// a wireless packet was received
		RFreceive((char*) &incoming);
		// Run received command
		executeCommand(incoming.command);
	}
}

ISR(TIMER0_OVF_vect){
	// Used to send telemetry data back to Wyvern Controller
	// Timer1 Clock = 1MHz, 8-bit
	set(TIFR0,TOV0);	// Clear flag
	overflowcounter++;
	overflowcounterbattery++;
	if(overflowcounter >32){
		// Send Telemetry Data
		RFtransmitUntil((char*) &outgoing,contr,1);
		overflowcounter = 0;
	}
	if(overflowcounterbattery>200){
		batteryVoltage();
		overflowcounterbattery = 0;
	}
}

/*************************************************************************
Function: main()
**************************************************************************/
int main(void)
{
	init_wyvern();
	LED_ucgreen_on();
	set(DDRF,7);
	PID_setPosition(&initial);
	
	for(;;){
		ReceiveRazorData();
		PID_setPosition(&incoming);
		PID_setThrust(incoming.throttle);
		PID_updatePWM(&razordata);
	}
	return 0;
}

/*************************************************************************
Function: init_wyvern()
Purpose:  Runs all initialization functions
		  Enables interrupts
Input:
Returns:
**************************************************************************/
void init_wyvern(void)
{
	// Setup Wyvern Systems
	init_uc();		//UC
	init_uart();	// Serial  Communication
	init_pwm();		// Motor PWM Control
	init_adc();		// ADC Initialization

	// Setup Wyvern USB Interface (Uncomment for debugging)
	/*
	usb_init();
	if(!(usb_configured()){
		// Something is not right...
	}
	*/
	
	// Setup Wyvern RF
	RFsetup(local,max(sizeof(packet_com_t),sizeof(packet_inf_t)));
	set(PCICR,PCIE0); // enable pin-change interrupts
	PCMSK0 =0x00;
	set(PCMSK0, PCINT4); // demask PCINT4
	set(TCCR0B,CS02);
	set(TCCR0B,CS00);	// Timer0 Clock = System Clock/1024
	set(TIMSK0,TOIE0);	// Enable Timer0 Overflow Interrupt
	
	// Enable Global Interrupts
	sei();

	// Force RF Interrupt (Pin Change Interrupt Channel 4) to run once
	clear(PORTB,4);
	if(RFRXdataReady()){
     	RFreceive((char*) &incoming);
	}

	// Set initial orientation
	initial.yaw = 0;
	initial.pitch = 0;
	initial.roll = 0;
	initial.throttle = 0;
}

/*************************************************************************
Function: ReceiveRazorData()
Purpose:  Receives the data output from the razor
Input:
Returns:
**************************************************************************/
void ReceiveRazorData(void)
{
	clear(PCICR,PCIE0); // disable pin-change interrupts
	//int corrected;
	
	uart_flush();
	int startcounter = 0;
	char datain;
	char data[12];
	//int datacounter = 0;	
	do{
		datain = ReceiveByte();
		if(datain == 0xFF){
			startcounter++;
		}
		else{
			startcounter =0;
		}
	}while(startcounter<3);
	data[0]=ReceiveByte();
	data[1]=ReceiveByte();
	data[2]=ReceiveByte();
	data[3]=ReceiveByte();
	data[4]=ReceiveByte();
	data[5]=ReceiveByte();
	data[6]=ReceiveByte();
	data[7]=ReceiveByte();
	data[8]=ReceiveByte();
	data[9]=ReceiveByte();
	data[10]=ReceiveByte();
	data[11]=ReceiveByte();
	memcpy(&razordata,data,12);
	if((int)razordata.pitch<0){
		ReceiveRazorData();
	}
	outgoing.yaw = (int)(razordata.yaw-20000);
	outgoing.pitch = (int)(razordata.pitch-20000);
	outgoing.roll = (int)(razordata.roll-20000);
	outgoing.p = P_pitch;
	outgoing.i = I_pitch;
	outgoing.d = D_pitch;
	outgoing.pidband = PID_BAND;
	/*
	TransmitString("Pitch = ");
	TransmitInt(razordata.pitch);
	TransmitString("   Corrected = ");
	corrected = (int)(razordata.pitch - 20000);
	TransmitInt(corrected);
	TransmitString("\n\r");
	*/

	set(PCICR,PCIE0); // enable pin-change interrupts
}

/*************************************************************************
Function: batteryVoltage()
Purpose:  Uses ADC on F4 to measure battery voltage
Input:
Returns:
**************************************************************************/
void batteryVoltage(void){
	// voltage divider: 19.8k -> 147.1k
	uint16_t battVoltage;
	battVoltage = get_adc(4);
	// return Volts*10
	battVoltage = ((double)4119*(double)battVoltage/10000);
	outgoing.battery = (int) battVoltage;
}
