/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.1
 April 2, 2011
 
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

/* ************************************************************ */
/* **************** DEFINITION OF VARIABLES ******************* */
/* ************************************************************ */
//
float PI = 3.14159265358979323846;

// General purpose LEDs
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

// Communication
// UART0 = USB
// UART1 = P9/P10
// UART2 = P13/P14
// UART3 = P27/P28
MODSERIAL pc(USBTX,USBRX);                     // USB Serial for debugging
MODSERIAL xbee_serial(p13,p14);                // XBee Serial - UART2
MODSERIAL IMUSerial(p28,p27);                  // IMU Serial - UART3

// XBee Serial Circular Buffer
//const int xbee_buffer_size = 1024;             // Circular Buffer Size
//char xbee_rx_buffer[xbee_buffer_size];        // Circular Buffer Array
//volatile int xbee_rx_head;                   // Circular Buffer Head
//volatile int xbee_rx_tail;                   // Circular Buffer Tail
//int xbee_bytes_available;                      // Number of Bytes Available in Circular Buffer


// IMU Serial Circular Buffer
//const int imu_buffer_size = 1024;             // Circular Buffer Size
//char imu_rx_buffer[imu_buffer_size];        // Circular Buffer Array
//volatile int imu_rx_head;                   // Circular Buffer Head
//volatile int imu_rx_tail;                   // Circular Buffer Tail
//int IMUBytesAvailable;                      // Number of Bytes Available in Circular Buffer

// This Unit's Attitude Information
struct Attitude{
    float Yaw;                    // Yaw of this unit
    float Pitch;                  // Pitch of this unit
    float Roll;                   // Roll of this unit
    float YawRate;                // Yaw Rate of this unit
    float PitchRate;              // Pitch Rate of this unit
    float RollRate;               // Roll Rate of this unit
    float Yaw_zero;               // Zeroed Yaw of this unit
    float Pitch_zero;             // Zeroed Pitch of this unit
    float Roll_zero;              // Zeroed Roll of this unit
};

Attitude MyAttitude;              // Attitude object

// Leader's Data
struct Leader{
    float Yaw;
    float Pitch;
    float Roll;
    float height;
    float distance;
    char command;
};

Leader LeaderData;                 // Leader Data Object

char low;
char high;
char checklow;
char checkhigh;
short checksum;
short yawnum;
short pitchnum;
short rollnum;
short yawratenum;
short pitchratenum;
short rollratenum;

int yawhi = 0;
int yawlow = 0;
// Yaw Factor
float yawFactor = 0.0109863;
int noIRcounter = 0;

// IR Camera Variables
char cam_address = 0xB0;            // I2C Camera Address
I2C cam_i2c(p9,p10);                // IR Camera I2C Pins
float IR_WIDTH = 0.031;             // Distance between IR beacons (in meters)
float cam_alpha = 0.9;              // IR Camera smoothing filter multiplier
float beacon_width = 19.5;          // Beacon width in centimeters


//IR Camera Data
struct IRcamera{
    int p1x;
    int p1y;
    int p1s;
    int p2x;
    int p2y;
    int p2s;
    int p3x;
    int p3y;
    int p3s;
    int p4x;
    int p4y;
    int p4s;
    bool valid[4];
    float dist;
    float centroidY;
    float centroidZ;
};

IRcamera IRdata;                    // IR Camera Data Object

// IR Camera Servo
double IRservoAngle = 0.00;
double IRservoMax = 30.00;
double IRservoMin = -30.00;
PwmOut IRservo(p26);                // PWM pin 26


// Thermal Sensor Variables
char thermal_address = 0xD0;        // I2C Thermal Sensor Address

// Thermal Sensor Data
struct Thermal{
    char ambient;
    char pixel1;
    char pixel2;
    char pixel3;
    char pixel4;
    char pixel5;
    char pixel6;
    char pixel7;
    char pixel8;
    char pixel9;
    // 8x32 array?
};

Thermal ThermalData;                // Thermal Sensor Data Object


// Sonar Variables
//float SONAR_METERS = 0.031;       // Sonar Meter Conversion
float sonar_alpha = 0.9;            // Sonar smoothing filter multiplier
PinName FrontSonarPin = p18;
PinName BackSonarPin = p16;
PinName LeftSonarPin = p19;
PinName RightSonarPin = p15;
PinName UpSonarPin = p20;
PinName DownSonarPin = p17;
PinName SonarPulsePin = p8;

//Sonar Data
struct Sonar{
    float front;
    float back;
    float left;
    float right;
    float up;
    float down;
};

Sonar SonarData;              // Sonar Data Object

//Tracking variables
struct Tracking{
    float dist_x;                // X Tracking Distance (in centimeters)
    float dist_y;                // Y Tracking Distance (in centimeters)
    float dist_z;                // Z Tracking Distance (in centimeters)
};

Tracking TrackingData;              // Tracking Data Object

// RC PWM Signal
PwmOut RCch1(p21);          // Roll Channel
PwmOut RCch2(p22);          // Pitch Channel
PwmOut RCch3(p23);          // Throttle Channel
PwmOut RCch4(p24);          // Yaw Channel
PwmOut RCch5(p25);          // Aux Channel

// RC PWM pulsewidths in microseconds (1000-2000)
struct RC{
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    int ch5;
};

RC RCoutput;              // RC Data Object

bool followleader = false;
float leaderyawoffset = 0;

/* ************************************************************ */
/* ******************* GENERIC mbed FUNCTIONS ***************** */
/* ************************************************************ */
void initUSB(){
    pc.baud(115200);
}

void sweepLEDs(){
    led1 = 1;
    wait(0.05);
    led2 = 1;
    led1 = 0;
    wait(0.05);
    led3 = 1;
    led2 = 0;
    wait(0.05);
    led4 = 1;
    led3 = 0;
    wait(0.1);
    led4 = 0;
}

void pulseLEDs(int count){
    for(int i=0; i<count; i++){
        led1 = 1;
        led2 = 1;
        led3 = 1;
        led4 = 1;
        wait(0.1);
        led1 = 0;
        led2 = 0;
        led3 = 0;
        led4 = 0;
        wait(0.1);
     }
}