// --------------------------------------------------
// Wyvern Quadrotor
// Controller
// version: 1.0.1
// date: April 13, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// --------------------------------------------------

// CONSTANT VARIABLES

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// GLOBAL VARIABLES
extern packet_com_t outgoing;
extern packet_inf_t incoming;
extern char menuScreen;
extern char printData;
//char buf[10]={0,0,0,0,0,0,0,0,0,0};
char str[20];
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// FUNCTION HEADERS
void printMenu();
void controllerCommand(char inputcommand);
extern void controllerTransmit(void);
void printEmpty(int num);
extern void getJoystick(void);

// ===========================================================

/*************************************************************************
Function: printMenu()
Purpose:  Prints the Wyvern User Interface
Input:    Menu Screen to print
Returns:  None
**************************************************************************/
void printMenu(){
			
	switch(menuScreen){
		case 'a': // Main Menu
			TransmitString("~Wyvern Quadrotor Controller~\n\r");
			TransmitString("s = START Quadrotor (Motor Spinup)\n\r");
			TransmitString("SPACEBAR = EMERGENCY STOP\n\r");
			TransmitString("r = Toggle On-board Red LED\n\r");
			TransmitString("g = Toggle On-board Green LED\n\r");
			TransmitString("d = Print Wyvern Telemetry Data\n\r");
			TransmitString("m = Motor Menu\n\r");
			TransmitString("p = PID Menu\n\r");
			TransmitString("j = Joystick Menu\n\r");
			printEmpty(6);
			break;
		case 'b': // Motor Menu
			TransmitString("~Motor Menu~\n\r");
			TransmitString("s = START Quadrotor (Motor Spinup)\n\r");
			TransmitString("SPACEBAR = EMERGENCY STOP\n\r");
			TransmitString("= = Increase Thrust\n\r");
			TransmitString("- = Decreaase Thrust\n\r");
			TransmitString("1,2,3,4 = Start Motor X\n\r");
			TransmitString("m = Main Menu\n\r");
			TransmitString("p = PID Menu\n\r");
			printEmpty(7);
			break;
		case 'c': // Motor Menu
			TransmitString("~PID Menu~\n\r");
			TransmitString("s = START Quadrotor (Motor Spinup)\n\r");
			TransmitString("SPACEBAR = EMERGENCY STOP\n\r");
			TransmitString("= = Increase Thrust\n\r");
			TransmitString("- = Decreaase Thrust\n\r");
			TransmitString("p = Increase P gain\n\r");
			TransmitString("o = Decrease P gain\n\r");
			TransmitString("i = Increase PID BAND\n\r");
			TransmitString("u = Decrease PID BAND\n\r");
			TransmitString("l = Increase D gain\n\r");
			TransmitString("k = Decrease D gain\n\r");
			TransmitString("d = Print out P gain and D gain\n\r");
			TransmitString("m = Main Menu\n\r");
			printEmpty(2);
			break;
		case 'd': // Joystick Menu
			TransmitString("~JoyStick Menu~\n\r");
			TransmitString("s = START Quadrotor (Motor Spinup)\n\r");
			TransmitString("SPACEBAR = EMERGENCY STOP\n\r");
			TransmitString("d = Print out Joystick Data\n\r");
			TransmitString("m = Main Menu\n\r");
			printEmpty(10);
			break;
		default:
			break;
	}

}

/*************************************************************************
Function: controllerCommand()
Purpose:  Commands to run on the Wyvern Controller
Input:    Command characters
Returns:  None
**************************************************************************/
void controllerCommand(char inputcommand){
	switch(menuScreen){
		case 'a': // Menu Screen A (Main Menu)
			switch(inputcommand){
				case 's': // Start Quadrotor
					//TransmitString("Starting Quadrotor\n\r");
					outgoing.command = inputcommand;
					break;
				case ' ': // Emergency Stop
					outgoing.command = inputcommand;
					break;
				case 'r': // Toggle On-board Red LED
					//TransmitString("Toggling Red LED");
					outgoing.command = inputcommand;
					break;
				case 'g': // Toggle On-board Green LED
					//TransmitString("Toggling Green LED");
					outgoing.command = inputcommand;
					break;
				case 'd': // Print out telemetry data
					// PRINT OUT VALUES CONTINUOUSLY
					while(!(DataInReceiveBuffer())){
							TransmitString("Yaw = ");
							TransmitInt(incoming.yaw);
							TransmitString("   Pitch = ");
							TransmitInt(incoming.pitch);
							TransmitString("   Roll = ");
							TransmitInt(incoming.roll);
							TransmitString("   Battery = ");
							TransmitInt(incoming.battery);
							TransmitString("\n\r");
					}
					break;
				case 'm': // Go to Motor Menu (MenuScreen = 'b');
					menuScreen = 'b';
					break;
				case 'p': // Go to PID Menu (MenuScreen = 'c');
					menuScreen = 'c';
					break;
				case 'j': // Go to Joystick Menu (MenuScreen = 'd');
					menuScreen = 'd';
					break;
				default:
					break;
			}
			break;
		case 'b': // Menu Screen B (Motor Menu)
			switch(inputcommand){
				case 's': // Start Quadrotor
					outgoing.command = inputcommand;
					break;
				case ' ': // Emergency Stop
					outgoing.command = inputcommand;
					break;
				case '=': // Increase Thrust
					outgoing.command = inputcommand;
					break;
				case '-': // Decrease Thrust
					outgoing.command = inputcommand;
					break;
				case '1': // Start Motor 1
					outgoing.command = inputcommand;
					break;
				case '2': // Start Motor 2
					outgoing.command = inputcommand;
					break;
				case '3': // Start Motor 3
					outgoing.command = inputcommand;
					break;
				case '4': // Start Motor 4
					outgoing.command = inputcommand;
					break;
				case 'm': // Return to Menu Screen 'a' (Main Menu)
					menuScreen = 'a';
					break;
				case 'p': // Go to Menu Screen 'c' (PID Menu)
					menuScreen = 'c';
					break;
				default:
					break;
			}
			break;
		case 'c': // Menu Screen C (PID Menu)
			switch(inputcommand){
				case 's' : // Start Quadrotor
					outgoing.command = inputcommand;
					break;
				case ' ': // Emergency Stop
					outgoing.command = inputcommand;
					break;
				case '=': // Increase Thrust
					outgoing.command = inputcommand;
					break;
				case '-': // Decrease Thrust
					outgoing.command = inputcommand;
					break;
				case 'p':
					outgoing.command = inputcommand;
					break;
				case 'o':
					outgoing.command = inputcommand;
					break;
				case 'i':
					outgoing.command = inputcommand;
					break;
				case 'u':
					outgoing.command = inputcommand;
					break;
				case 'l':
					outgoing.command = inputcommand;
					break;
				case 'k':
					outgoing.command = inputcommand;
					break;
				case 'd': // Print out Gain Data
					// PRINT OUT VALUES CONTINUOUSLY
					while(!(DataInReceiveBuffer())){
							TransmitString("P_gyr = ");
							TransmitInt(incoming.p);
							TransmitString("   D_gyr = ");
							TransmitInt(incoming.d);
							TransmitString("   PID BAND = ");
							TransmitInt(incoming.pidband);
							TransmitString("\n\r");
					}
					break;
				case 'm': // Return to Menu Screen 'a' (Main Menu)
					menuScreen = 'a';
					break;
				default:
					break;
			}
			break;
		case 'd': // Menu Screen D (Joystick Menu)
			switch(inputcommand){
				case 's': // Start Quadrotor
					outgoing.command = inputcommand;
					break;
				case ' ': // Emergency Stop
					outgoing.command = inputcommand;
					break;
				case 'd': // Print out Joystick Data
					// PRINT OUT VALUES CONTINUOUSLY
					while(!(DataInReceiveBuffer())){
							getJoystick();
							TransmitString("Pitch = ");
							TransmitInt(outgoing.pitch);
							TransmitString("   Roll = ");
							TransmitInt(outgoing.roll);
							TransmitString("   Yaw = ");
							TransmitInt(outgoing.yaw);
							TransmitString("   Throttle = ");
							TransmitInt(outgoing.throttle);
							TransmitString("\n\r");
					}
					break;
				case 'm': // Return to Menu Screen 'a' (Main Menu)
					menuScreen = 'a';
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

/*************************************************************************
Function: printEmpty()
Purpose:  Prints out a empty line to fill up the menu screen
Input:    Number of empty lines to print
Returns:  None
**************************************************************************/
void printEmpty(int num){
	for(int i=0;i<num;i++){
		TransmitString("\n\r");
	}
}
