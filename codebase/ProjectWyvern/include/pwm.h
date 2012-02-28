// --------------------------------------------------
// WyvernMaEvArM microcontroller board
// PWM
// version: 1.0.1
// date: March 28, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// --------------------------------------------------

// CONSTANT VARIABLES
unsigned int PWM_PERIOD = 20000; //50 Hz PWM given 8 MHz system clock

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void init_pwm(void);
void set_duty(unsigned int motornum, unsigned int duty);

// ===========================================================

/*************************************************************************
Function: init_PWM()
Purpose:  Sets up four 16-bit PWM outputs
		-Timer 1
			-Channel A (Motor 1) (B5)
			-Channel B (Motor 2) (B6)
			-Channel C (Motor 3) (B7)
		-Timer 3
			-Channel A (Motor 4) (C6)
		-Initializes OCRxx to 0 (0 duty cycle) on all motors
Input:    None
Returns:  None
**************************************************************************/
void init_pwm(void){
	//////// SET TIME PRESCALER ////////
	// Timer 1 - use system clock (system_clock/1)
	clear(TCCR1B,CS10);
	set(TCCR1B,CS11);
	clear(TCCR1B,CS12);

	// Timer 3 - use system clock (system_clock/1)
	clear(TCCR3B,CS30);
	set(TCCR3B,CS31);
	clear(TCCR3B,CS32);

	//////// SET PWM MODE ////////
	// Timer 1 - UP to ICR1 PWM Mode (Mode 14 - 16bit 65535)
	set(TCCR1B,WGM13);
	set(TCCR1B,WGM12);
	set(TCCR1A,WGM11);
	clear(TCCR1A,WGM10);
	ICR1 = PWM_PERIOD;

	// Timer 1 Channel A - clear at OCR1A, set at rollover
	set(TCCR1A,COM1A1);
	clear(TCCR1A,COM1A0);
	OCR1A = 0;

	// Timer 1 Channel B - clear at OCR1B, set at rollover
	set(TCCR1A,COM1B1);
	clear(TCCR1A,COM1B0);
	OCR1B = 0;

	// Timer 1 Channel C - clear at OCR1C, set at rollover
	set(TCCR1A,COM1C1);
	clear(TCCR1A,COM1C0);
	OCR1C = 0;

	// Timer 3 - UP to ICR3 PWM Mode (Mode 14 - 16bit 65535)
	set(TCCR3B,WGM33);
	set(TCCR3B,WGM32);
	set(TCCR3A,WGM31);
	clear(TCCR3A,WGM30);
	ICR3 = PWM_PERIOD;

	// Timer 3 Channel A - clear at OCR3A, set at rollover
	set(TCCR3A,COM3A1);
	clear(TCCR3A,COM3A0);
	OCR3A = 0;

	// Enable Timer 1 (B5,B6,B7) and Timer 3 (C6) Output
	set(DDRB,5);
	set(DDRB,6);
	set(DDRB,7);
	set(DDRC,6);
}

/*************************************************************************
Function: set_duty()
Purpose:  Sets the Duty Cycle of the four PWM controlled motors
			-Motor 1 (Timer 1 - Channel A)
			-Motor 2 (Timer 1 - Channel B)
			-Motor 3 (Timer 1 - Channel C)
			-Motor 4 (Timer 3 - Channel A)
			-Duty Cycle ranges between 0 and PWM_PERIOD
		  Auto corrects for invalid input
			-This correction will be handled earlier during motor speed calculations
Input:    motor number, duty value
Returns:  None
**************************************************************************/
void set_duty(unsigned int motornum,unsigned int duty){
	// Prevent invalid Duty Cycle
	if(duty>2000){
		duty = 2000;
	}else if(duty<1000){
		duty = 1000;
	}

	switch(motornum){
		case 1:
			OCR1A = duty;
			break;
		case 2:
			OCR1B = duty;
			break;
		case 3:
			OCR1C = duty;
			break;
		case 4:
			OCR3A = duty;
			break;
		default:
			break;
	}
}
