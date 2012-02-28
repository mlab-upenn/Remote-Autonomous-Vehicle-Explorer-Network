/* ********************************************************************** */
/*                    R.A.V.E.N. Leader Quadroter code                    */
/*                                                                        */
/* Authors : William Etter (UPenn EE '11)                                 */
/*           Paul Martin (UPenn EE 'll)                                   */
/*                                                                        */
/* Date : April 5, 2011                                                   */
/* Version : 2.2                                                          */
/* Hardware : ArduPilot Mega, CHRobotics AHRS IMU, XBee Pro RF            */
/* Disclaimer: Some code segments from ArduCopter Project                 */
/* ********************************************************************** */

/* ****************************************************************************** */
/* ********************   Configuration Definitions  **************************** */
/* ****************************************************************************** */
//#define IsLeader        // Quadrotor is a Leader

//#define HasBaro          // Barometer connected
//#define HasGPS           // GPS connected


/* ****************************************************************************** */
/* ******************************   TO DO  ************************************** */
/* ****************************************************************************** */
//  - Altitude PID
//  - Thermal Sensor
//  - Sonar Sensor
//  - Fixed IR Sensor Implementation
//  - IR Obstacle avoidance
//  - Remove un-needed variables
//  - Implement Wait for Battery Sensor Connection


/* ****************************************************************************** */
/* ****************************** Includes ************************************** */
/* ****************************************************************************** */
#include "WProgram.h"
#include "Ravenleader.h"       // Variable definitions for the RAVEN System
#include <Wire.h>              // I2C Communication library
#include <APM_RC.h>            // ArduPilot Mega RC Library
#include <APM_BMP085.h>        // ArduPilot Mega BMP085 Library
//#include <SoftwareSerial.h>  // If another serial connection is required


/* ************************************************************ */
/* **************** MAIN PROGRAM - SETUP ********************** */
/* ************************************************************ */
void setup(){
  Serial.begin(115200);        // Initialize Serial Communication (FTDI on Serial 0)
  
  LEDs_init();                 // Initialize LEDs
  statLEDredOn();              // Red status LED On
  #ifdef IsLeader
    XBee_init();               // Initialize XBee Communication
  #endif
  
  
  #ifdef HasBaro
    APM_BMP085.Init();         // Initialize BMP085 Pressure Sensors
    delay(1000);               
    set_GroundTempPress();     // Read ground temperature and pressure
  #endif
  
  delay(3000);
  statLEDblueOn();             // Blue status LED On
  boolean IMUinitialization = IMU_init();    // Initialize IMU
  delay(1000);
  IMUinitialization = IMU_calibrate();       // Calibrate IMU
 
  for(int i = 0; i <= 12; i++) {             // Delay to calibrate IMU
    statLEDblueOn();
    delay(200);
    statLEDoff();
    delay(200);
  }
  
  
  APM_RC.Init();                 // Initialize RC
  delay(500);
  
  // Arm Motor Controllers (Ready to Spin Up)
  APM_RC.OutputCh(3,MOTOR_ARM);
  delay(250);
  APM_RC.OutputCh(1,MOTOR_ARM);
  delay(250);
  APM_RC.OutputCh(0,MOTOR_ARM);
  delay(250);
  APM_RC.OutputCh(2,MOTOR_ARM);
  
  // Delay between motor ARM and motor STOP speed
  for(int i = 0; i <= 2; i++) {  
    statLEDorangeOn();
    delay(200);
    statLEDoff();
    delay(200);
  }
  
  // Set RC Outputs to Minimum Throttle (Motors Stopped) 
  APM_RC.OutputCh(0,MOTOR_STOP);
  APM_RC.OutputCh(1,MOTOR_STOP);
  APM_RC.OutputCh(2,MOTOR_STOP);
  APM_RC.OutputCh(3,MOTOR_STOP);
  
  motorArmed = 0;                // Motors "disarmed" (not spinning)

  // Initialize timers
  fastTimer = millis();
  mediumTimer = millis();
  slowTimer = millis();
  
  IMU_data_request();
  
  statLEDgreenOn();              // Ready to Go
   
}                                // End of setup()


/* ************************************************************ */
/* **************       MAIN PROGRAM LOOP  ******************** */
/* ************************************************************ */
void loop(void){

  /* **************      Slow Loop  (1 Hz)    ******************** */
  if((millis()-slowTimer)>=1000){
    slowTimer = millis();        // Update timer
    
    // Slow Loop Command Hierarchy
    switch(slowloopcounter){
      case 0:                    // Program 0 - Read Battery Voltage
        read_battery();
        slowloopcounter++;       // Increment slowloopcounter
        break;
        
      case 1:                    // Program 1 - Calibrate IMU Roll and Pitch and Channel Mids
        if(ch_aux>1200){
          // Set Pitch and Roll offsets (level surface) 
          pitch_offset_temp = -pitch;
          roll_offset_temp = -roll;
          YAW_MID = ch_yaw;                             // Obtain Center Yaw
        }else{
          pitch_offset += pitch_offset_temp;
          pitch_offset_temp = 0;
          roll_offset += roll_offset_temp;
          roll_offset_temp = 0;
        } 
        slowloopcounter++;       // Increment slowloopcounter
        break;
      
      case 2:                    // Program 2 - Print PID Term
        //Serial.println(KD_Yaw);
        slowloopcounter=0;       // Increment slowloopcounter - SET TO 0 for right now 
        break;
      
      case 3:                    // Program 3 - Read Altitude from Pressure Sensor
        #ifdef HasBaro
          read_pressure();
        #endif 
        slowloopcounter++;       // Increment slowloopcounter
        break;
        
      default:
        slowloopcounter = 0;     // Reset slowloopcounter
        break;
    }                            // End of Switch()
  }                              // End of Slow Loop
  
  
  /* **************    Medium Loop (5 Hz)    ******************** */
   if((millis()-mediumTimer)>=200){
     mediumTimer = millis();     // Update timer  
     
     // Medium Loop Command Hierarchy
     switch(mediumloopcounter){
       case 0:                    // Program 0 - Send data to basestation
         #ifdef IsLeader
           if(followerpresent == true){
             XBee_sendBase();
           }
         #endif
         mediumloopcounter++;     // Increment mediumloopcounter
         break;
       case 1:                    // Program 1 - 
         mediumloopcounter++;     // Increment mediumloopcounter
         break;
       default:
         mediumloopcounter = 0;   // Reset mediumloopcounter
         break;
     }                            // End of Switch()
     
   }                              // End of Medium Loop
  
    /* **************     Fast Loop (20 Hz) ******************** */
   if((millis()-fastTimer)>=50){
      // Update timers
      fastTimer = millis();
      //Serial.print(yaw);
      //Serial.print(" / ");
      //Serial.print(pitch);
      //Serial.print(" / ");
      //Serial.println(roll);
      
      #ifdef IsLeader
        if(followerpresent == false){
          XBee_sendBase();
        }
        if(followerpresent == true){
          XBee_sendFollower();
        }
      #endif
   }
    
    /* **************    Full Speed Loop    ******************** */
    IMU_data();              // Read IMU Data
    get_RC();                     // Get RC Commands
    armDisarm_RC();               // Arm/Disarm Motors
    
    #ifdef IsLeader
      if(Serial3.available()>xbeebuffermin){    // Leader - Check for Base Station Command
        XBeeParseData();
      }
    #endif

    if(newIMUdata){
      PID();      // PID Loop
    }
    send_RC();                    // Send RC Commands
}                                 // End of loop()
                                  // End of Ravenleader.pde
