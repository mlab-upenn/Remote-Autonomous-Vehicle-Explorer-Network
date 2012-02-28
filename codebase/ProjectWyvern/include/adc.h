// --------------------------------------------------
// Wyvern Quadrotor
// ADC
// version: 1.0.0
// date: April 11, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// --------------------------------------------------

// CONSTANT VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void init_adc(void);
uint16_t get_adc(unsigned int ADC_Channel);

// ===========================================================

/*************************************************************************
Function: init_adc()
Purpose:  Sets up the ACD
Input:    None
Returns:  None
**************************************************************************/
void init_adc(void){
	// ADC Multiplexer Selection Register
	// Use Vcc, right justified
	ADMUX |= (0<<REFS1)|(1<<REFS0)|(0<<ADLAR);

	
	// Digital Input Disable Register 0
	// Used to disable digital input on ADC Channels
	// Channels 0,1,4,5,6,7 (1=disabled)
	// DIDR0 = [ADC7D,ADC6D,ADC5D,ADC4D, - , - , ADC1D, ADC0D]
	DIDR0 |=0xF3;

	// ADC Control and Status Register A
	// Turn on ADC,prescale ADC clock by 64 (8MHz/64 = 125kHz)
	ADCSRA |=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);
}

/*************************************************************************
Function: 	get_adc()
Purpose:  	Read a value from one of the 6 ADC values
Input:		ADC Channel to obtain value on (0,1,4,5,6,7)
Returns:	ADC Result
**************************************************************************/
uint16_t get_adc(unsigned int ADC_Channel){
	// Channel set with	ADMUX => (MUX4,MUX3,MUX2,MUX1,MUX0)

	// Clear ADMUX Channel Select
	ADMUX &=0xE0;

	switch(ADC_Channel){
		case 0: 	// Channel 0 (00000)
			ADMUX |=0x00;
			break;
		case 1:		// Channel 1 (00001)
			ADMUX |=0x01;
			break;
		case 4:		// Channel 4 (00100)
			ADMUX |=0x04;
			break;
		case 5:		// Channel 5 (00101)
			ADMUX |=0x05;
			break;
		case 6:		// Channel 6 (00110)
			ADMUX |=0x06;
			break;
		case 7:		// Channel 7 (00111)
			ADMUX |=0x07;
			break;
		default:
			break;
	}

	// Start converstion
	set(ADCSRA,ADSC);

	// Wait until complete
	while(!(check(ADCSRA,ADIF))){
	}
	
	// Clear Flag
	set(ADCSRA,ADIF);
	// Obtain value
	return ADC;
}

// CURRENTLY IN DEVELOPEMENT
/*************************************************************************
Function: 	get_ucTemp()
Purpose:  	Get the (rough) UC temperature from the on-board sensor
Input:		
Returns:	ADC Temperature Sensor Result
**************************************************************************/
uint16_t get_ucTemp(){
	// Save previous ADMUX settings
	int oldADMUX = ADMUX;
	
	// Clear ADMUX Channel Select
	ADMUX &= 0xE0;

	// Set voltage refernce to internal 2.56V
	ADMUX |= 0xC0;

	// On-chip Temperature Sensor (11111)
	ADMUX |= 0x1F;
	
	// Start converstion
	set(ADCSRA,ADSC);

	// Wait until complete
	while(!(check(ADCSRA,ADIF))){
	}
	
	// Clear Flag
	set(ADCSRA,ADIF);

	// Read a second time
	// Start converstion
	set(ADCSRA,ADSC);

	// Wait until complete
	while(!(check(ADCSRA,ADIF))){
	}
	
	// Clear Flag
	set(ADCSRA,ADIF);

	// Return old ADMUX settings
	ADMUX = oldADMUX;

	// Obtain value
	return ADC;
}
