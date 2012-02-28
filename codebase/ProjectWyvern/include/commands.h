// --------------------------------------------------
// Wyvern Quadrotor
// Commands
// version: 1.0.1
// date: April 13, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// --------------------------------------------------

// CONSTANT VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void executeCommand(char command);

// ===========================================================

/*************************************************************************
Function: executeCommand()
Purpose:  Runs commands based on user input
Input:    Command character
Returns:  None
**************************************************************************/
void executeCommand(char command){
			
	switch(command){
		case 'r':
			LED_ucred_toggle();
			break;
		case 'g':
			LED_ucgreen_toggle();
			break;
		case 's':	// Wyvern Startup
			PID_command('s');
			break;
		case ' ':	// Emergency Motor Stop (decrease power to duty factor of 0 immediately)
			PID_command(' ');
			break;
		case '=':	// increases thrust
			PID_command('=');
			break;
		case '-':	// decreases thrust
			PID_command('-');
			break;
		case '1':	// Start Motor 1
			PID_command('1');
			break;
		case '2':	// Start Motor 2
			PID_command('2');
			break;
		case '3':	// Start Motor 3
			PID_command('3');
			break;
		case '4':	// Start Motor 4
			PID_command('4');
			break;
		case 'p':
			PID_command('p');
			break;
		case 'o':
			PID_command('o');
			break;
		case 'l':
			PID_command('l');
			break;
		case 'k':
			PID_command('k');
			break;
		case 'i':
			PID_command('i');
			break;
		case 'u':
			PID_command('u');
			break;
		default:
			break;
	}

}

