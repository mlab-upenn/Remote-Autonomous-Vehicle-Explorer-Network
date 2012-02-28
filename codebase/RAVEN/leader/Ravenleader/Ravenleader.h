/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.6
 February 9, 2011
 
 Authors:
       William Etter (UPenn EE 'll)
       Paul Martin (UPenn EE 'll)
 
 This program is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by 
 the Free Software Foundation, either version 3 of the License, or 
 (at your option) any later version. 
 
 This program is distributed in the hope that it will be useful, 
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 GNU General Public License for more details. 
 
 You should have received a copy of the GNU General Public License 
 along with this program. If not, see <http://www.gnu.org/licenses/>.
********************************************************************/

/* Software version */
#define VER 2.1    // Current software version (only numeric values)


/*******************************************************************/
// ArduPilotMega  Hardware and Software Settings
/*******************************************************************/
/* General definitions */
#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0

/*************************************************************************
//              Hardware definitions
*************************************************************************/
// Status LED pin connections
#define statusLEDvcc 53       // Status common anode (+)           (PB0)
#define statusLEDred 10       // Red cathode (-)                   (PB4)
#define statusLEDgreen 9      // Green cathode (-)                 (PH6)
#define statusLEDblue 11      // Blue cathode (-)                  (PB5)
#define headlightLED 32       // Headlight LED Switch (Transistor) (PC0)

// IR Sensor Ground and Signal pin connections
/////////////////////////////////////////////////////////////////////////////// Update for Sonar Sensors!!!!!!
#define IRFgnd 37             // Front IR Sensor Ground            (PC0)
#define IRFsig 0              // Front IR Sensor Signal (ADC0)     (PF0)
#define IRBgnd 36             // Back IR Sensor Ground             (PC1)
#define IRBsig 1              // Back IR Sensor Signal (ADC1)      (PF1)
#define IRLgnd 35             // Left IR Sensor Ground             (PC2)
#define IRLsig 2              // Left IR Sensor Signal (ADC2)      (PF2)
#define IRRgnd 34             // Right IR Sensor Ground            (PC3)
#define IRRsig 3              // Right IR Sensor Signal (ADC0)     (PF3)
#define IRUgnd 33             // UP IR Sensor Ground               (PC4)
#define IRUsig 4              // UP IR Sensor Signal (ADC0)        (PF4)

// Battery Definitions
#define BatterySig 15          // Battery Sensor Signal (ADC15)    (PK7)
#define INPUT_VOLTAGE 5.03     // Voltage supplied to ArduPilot Mega
#define VOLT_DIV 3.135         // Battery Sensor Voltage Divider R1 = 4.63K  R2 = 9.88K 1/((R1/(R1+R2)))
#define LOW_VOLTAGE 11.4       // Low battery voltage

/*************************************************************************
//              Software Definitions
*************************************************************************/
// TIMERS
unsigned long microTimer = 0;         // PID Microsecond Timer
unsigned long microTimer_old = 0;     // PID Microsecond Timer Timer Old value (used for getting actual run time)
long fastTimer = 0;                  // Fast Timer (IMU, PID Loop)
long mediumTimer = 0;                // Medium Timer (IR and Ping Sensors, etc)
long slowTimer = 0;                  // Slow Timer (Battery sensor, etc)
float Dt = 0.02;                     // Integration time for PID (differential time)
float Time_old = 0.0;
float Time = 0.0;
char slowloopcounter = 0;            // Slow loop program counter
char mediumloopcounter = 0;          // Medium loop program counter
char fastloopcounter = 0;

char xbeebuffermin = 16;             // Minimum number needed in buffer to read data

// FLAGS
boolean LowBatteryFlag = false;      // Low Battery Flag

boolean followerpresent = false;     // Is a follower present?

///////////////////// SENSORS ////////////////////////////////////////
// Sonar Sensor
int sonarAlt = 0;                   // Sonar Sensor Altitude

// IR Camera
int IRsensorAddress = 0xB0;         // I2C Address of Pixart IR Camera Sensor
int IRslaveAddress;                   // I2C Slave Address
int point1x = 0;                    // IR Point 1 X position
int point1y = 0;                    // IR Point 1 Y position
int point1s = 0;                    // IR Point 1 size
int point2x = 0;                    // IR Point 2 X position
int point2y = 0;                    // IR Point 2 Y position
int point2s = 0;                    // IR Point 2 size
int point3x = 0;                    // IR Point 3 X position
int point3y = 0;                    // IR Point 3 Y position
int point3s = 0;                    // IR Point 3 size
int point4x = 0;                    // IR Point 4 X position
int point4y = 0;                    // IR Point 4 Y position
int point4s = 0;                    // IR Point 4 size
byte ir_data[16];                   // IR Data Array
int ir_s;                           // IR Data Mask
int IRcameraServoCh = 5;            // IR Camera Servo Channel
int IRcameraServo = 0;              // IR Camera Servo Control
int IRcameraServoCenter = 1510;     // IR Camera Servo Center

//Pressure Sensor
long pressureAlt = 0;               // Pressure Sensor Altitude
long ground_pressure = 0;           // Ground Pressure
double ground_temperature = 0;      // Ground Temperature

// IMU Data
boolean newIMUdata = false;         // Check if received new IMU data
float yaw = 0;                      // Yaw in °
float pitch = 0;                    // Pitch in °
float roll = 0;                     // Roll in °
float yawrate = 0;                  // Yaw Rate in °/s
float pitchrate = 0;                // Pitch Rate in °/s
float rollrate = 0;                 // Roll Rate in °/s
float gyroz = 0;                    // Z Gyro in °/s
float gyroy = 0;                    // Y Gyro in °/s
float gyrox = 0;                    // X Gyro in °/s
float yaw_old = 0;                  // Old Yaw in °
float pitch_old = 0;                // Old Pitch in °
float roll_old = 0;                 // Old Roll in °
float yawrate_old = 0;                // Old Z Gyro in °/s
float pitchrate_old = 0;                // Old Y Gyro in °/s
float rollrate_old = 0;                // Old X Gyro in °/s

float ratefilter_alpha = 0.5;       // Gyro Filter Alpha Value
boolean packetfound = false;        // IMU Data Packet Found

// IMU Data Offsets
float yaw_offset = 0;
float pitch_offset = 0.0;            // Set when Aux Channel 1 is set high (and motors disarmed)
float roll_offset = 0.0;             // Set when Aux Channel 1 is set high (and motors disarmed)
float pitch_offset_temp = 0.0;       // Temporary value
float roll_offset_temp = 0.0;        // Temporary value
float yawrate_offset = 0;            // Yaw Rate Offset
float pitchrate_offset = 0;          // Pitch Rate Offset
float rollrate_offset = 0;           // Roll Rate Offset
float gyroz_offset = 0;              // Z Gyro Offset
float gyroy_offset = 0;              // Y Gyro Offset
float gyrox_offset = 0;              // X Gyro Offset

// Velocity Data
float velx = 0;
float vely = 0;
float velz = 0;

// Battery
uint8_t battery = 0;                 // 10*(Battery Voltage) (i.e. 114 = 11.4V)

///////////////////// FLIGHT ////////////////////////////////////////
// Arming & Disarming delays
#define ARM_DELAY 400                // Milliseconds of how long you need to keep rudder to max right for arming motors
#define DISARM_DELAY 100             // Milliseconds of how long you need to keep rudder to max left for disarming motors

// Arming/Disarming
int Arming_counter=0;            // Counter used for arming motors
int Disarming_counter=0;         // Counter used for disarming motors

// Motor Speed Definitions
#define MOTOR_STOP 7600              // Setting at which motors are stopped
#define MOTOR_ARM 7500               // Setting at which ESCs arm
#define MOTOR_START 8600             // Setting at which motors start to spin (idle)
#define MAX_MOTOR 15100              // Maximum motor speed

// Motor Channel Defitions
int FLMotorCh = 3;                   // Front Left Motor Channel
int FRMotorCh = 1;                   // Front Right Motor Channel
int RLMotorCh = 2;                   // Rear Left Motor Channel
int RRMotorCh = 0;                   // Rear Right Motor Channel

// Motor Variables
int FrontLeftMotor;                  // Front Left Motor Speed
int RearRightMotor;                  // Rear Right Motor Speed
int RearLeftMotor;                   // Rear Left Motor Speed
int FrontRightMotor;                 // Front Right Motor Speed
byte motorArmed = 0;                 // Motor Armed Boolean (0 = Unarmed, 1 = Armed)
int minThrottle = 0;                 // Minimum Motor Throttle for current state (armed/disarmed)

// RC Values
int ROLL_MID = 1500;                 // Radio Roll channel mid value
int PITCH_MID = 1500;                // Radio Pitch channel mid value
int YAW_MID = 1500;                  // Radio Yaw channel mid value
int THROTTLE_MID = 1505;             // Radio Throttle channel mid value
int AUX_MID = 1500;                  // Radio Aux channel mid value
int MIN_THROTTLE = 1040;             // Minimum Throttle channel value
int throttle_gain = 5;               // Throttle Gain (1000-2000 to x-y, get it in an acceptable range)

// Yaw, Pitch, Roll, and Throttle Channel Definitions
#define roll_channel 0               // Roll Channel definition
#define pitch_channel 1              // Pitch Channel definition
#define throttle_channel 2           // Throttle Channel definition
#define yaw_channel 3                // Yaw Channel definition  

// RC Input Values
int ch_roll = 0;                     // RC Roll Input
int ch_pitch = 0;                    // RC Pitch Input
int ch_throttle = 0;                 // RC Throttle Input
int ch_throttle_adj = 0;             // Adjust RC Throttle Input
int ch_yaw = 0;                      // RC Yaw Input
int ch_aux = 0;                      // RC Aux Channel 1 Input
int ch_aux2 = 0;                     // RC Aux Channel 2 Input

// RC Input Offset Values
#ifdef IsLeader
  float ch_roll_offset = -50;          // RC Roll Input Offset
  float ch_pitch_offset = -22;         // RC Pitch Input Offset
  float ch_throttle_offset = 0;        // RC Throttle Input Offset
  int ch_yaw_offset = -30;             // RC Yaw Input Offset
  float ch_aux_offset = 0;             // RC Aux Channel 1 Input Offset
  float ch_aux2_offset = 0;            // RC Aux Channel 2 Input Offset
#else
  float ch_roll_offset = 0;          // RC Roll Input Offset
  float ch_pitch_offset = 0;         // RC Pitch Input Offset
  float ch_throttle_offset = 0;        // RC Throttle Input Offset
  int ch_yaw_offset = 0;             // RC Yaw Input Offset
  float ch_aux_offset = 0;             // RC Aux Channel 1 Input Offset
  float ch_aux2_offset = 0;            // RC Aux Channel 2 Input Offset
#endif

// RC Input Slope Values (Input Filtering)
float ch_roll_slope = 1.0;           // RC Roll Slope
float ch_pitch_slope = 1.0;          // RC Pitch Slope
float ch_throttle_slope = 1.0;       // RC Throttle Slope
float ch_yaw_slope = 1.0;            // RC Yaw Slope
float ch_aux_slope = 1.0;            // RC Aux Channel 1 Slope
float ch_aux2_slope = 1.0;           // RC Aux Channel 2 Slope

// User Attitude control variables
float command_rx_roll = 0;           // RC Roll Control variable 
float command_rx_pitch = 0;          // RC Pitch Control variable
float command_rx_yaw = 0;            // RC Yaw Control variable
float roll_command_divider = 15.0;   // RC Roll Command Divider (decrease sensitivity)
float pitch_command_divider = 15.0;  // RC Pitch Command Divider (decrease sensitivity)
float yaw_command_divider = 180.0;   // RC Yaw Command Divider (decrease sensitivity)

// PID Stability Variables
float stable_yaw = 0;                // PID Yaw Stability Variable
float stable_pitch = 0;              // PID Pitch Stability Variable
float stable_roll = 0;               // PID Roll Stability Variable

// PID Attitude control variables
int control_roll = 0;                // PID Roll Control
int control_pitch = 0;               // PID Pitch Control
int control_yaw = 0;                 // PID Yaw Control
#define MAX_CONTROL_OUTPUT 350       // Maxiumum PID Output Control

// Attitude PID controls
float yaw_P = 0;                     // Yaw Proportional Term
float yaw_I = 0;                     // Yaw Integral Term
float yaw_D = 0;                     // Yaw Derivative Term
float pitch_P = 0;                   // Pitch Proportional Term
float pitch_I = 0;                   // Pitch Integral Term
float pitch_D = 0;                   // Pitch Derivative Term
float roll_P = 0;                    // Roll Proportional Term
float roll_I = 0;                    // Roll Integral Term
float roll_D = 0;                    // Roll Derivative Term

// PID Constants for Yaw, Pitch, and Roll

// Attempt 6

float KP_Yaw = 9.0;                  // Yaw P Constant  9.0
float KI_Yaw = 0.0;                  // Yaw I Constant
float KD_Yaw = 2.6;                 // Yaw D Constant   2.6

float KP_Pitch = 7.5;               // Pitch P Constant    7.5    
float KI_Pitch = 0.0;                // Pitch I Constant
float KD_Pitch = 2.15;               // Pitch D Constant

float KP_Roll = 7.5;                // Roll P Constant   7.5
float KI_Roll = 0.0;                 // Roll I Constant
float KD_Roll = 2.15;                // Roll D Constant

/*
float KP_Yaw = 9.0;                  // Yaw P Constant
float KI_Yaw = 0.0;                  // Yaw I Constant
float KD_Yaw = 2.6;                 // Yaw D Constant

float KP_Pitch = 7.5;               // Pitch P Constant        
float KI_Pitch = 0.0;                // Pitch I Constant
float KD_Pitch = 2.73;               // Pitch D Constant

float KP_Roll = 7.5;                // Roll P Constant
float KI_Roll = 0.0;                 // Roll I Constant
float KD_Roll = 2.73;                // Roll D Constant
*/

// Attempt 5
/*
float KP_Yaw = 5.0;                  // Yaw P Constant  4.2old
float KI_Yaw = 0.0;                  // Yaw I Constant
float KD_Yaw = 2.13;                 // Yaw D Constant

float KP_Pitch = 7.0;               // Pitch P Constant  6.05old        
float KI_Pitch = 0.0;                // Pitch I Constant
float KD_Pitch = 2.73;               // Pitch D Constant  2.68old

float KP_Roll = 7.0;                // Roll P Constant  6.05old
float KI_Roll = 0.0;                 // Roll I Constant
float KD_Roll = 2.73;                // Roll D Constant 6.05old
*/


// PID Gain Constants
float PID_Yaw_Control_Gain = 5.0;    // PID Yaw Gain
float PID_Pitch_Control_Gain = 5.0;  // PID Pitch Gain
float PID_Roll_Control_Gain = 5.0;   // PID Roll Gain


// Test
float chaux2_pid = 0.0;
char toggle = 0;
float newtime = 0;
float oldtime = 0;
float oldgyrox = 0;
float yaw_D_old = 0;
float pitch_D_old = 0;
float roll_D_old = 0;
float alpha = 0.7;
