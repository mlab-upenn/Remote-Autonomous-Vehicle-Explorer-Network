/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.0
 December 15, 2010
 
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

// TO DO
//  - Error lights


/*************************************************************************
Function: LEDs_init()
Purpose:  Initializes LEDs
**************************************************************************/
void LEDs_init(void){
  // Initialize status LED as OFF
  pinMode(statusLEDvcc, OUTPUT);
  digitalWrite(statusLEDvcc, LOW);
  pinMode(statusLEDred, OUTPUT);
  digitalWrite(statusLEDred, HIGH);
  pinMode(statusLEDgreen,OUTPUT);
  digitalWrite(statusLEDgreen, HIGH);
  pinMode(statusLEDblue, OUTPUT);
  digitalWrite(statusLEDblue, HIGH);
  
  // Initialize Headlight as OFF
  pinMode(headlightLED, OUTPUT);
  digitalWrite(headlightLED, LOW);
}

/*************************************************************************
Function: statLEDoff()
Purpose:  Turns off the Status LED
**************************************************************************/
void statLEDoff(void){
  // Set Status LED common anode to LOW
  digitalWrite(statusLEDvcc, LOW);
}

/*************************************************************************
Function: statLEDredOn()
Purpose:  Turns on the Status LED as RED
**************************************************************************/
void statLEDredOn(void){
  // Set Status LED common anode to HIGH
  digitalWrite(statusLEDvcc, HIGH);
  // Turn off other cathodes
  digitalWrite(statusLEDgreen, HIGH);
  digitalWrite(statusLEDblue, HIGH);
  // Set Status LED red cathode to full LOW
  digitalWrite(statusLEDred, LOW);
}

/*************************************************************************
Function: statLEDgreenOn()
Purpose:  Turns on the Status LED as GREEN
**************************************************************************/
void statLEDgreenOn(void){
  // Set Status LED common anode to HIGH
  digitalWrite(statusLEDvcc, HIGH);
  // Turn off other cathodes
  digitalWrite(statusLEDred, HIGH);
  digitalWrite(statusLEDblue, HIGH);
  // Set Status LED green cathode to full LOW
  digitalWrite(statusLEDgreen, LOW);
}

/*************************************************************************
Function: statLEDblueOn()
Purpose:  Turns on the Status LED as BLUE
**************************************************************************/
void statLEDblueOn(void){
  // Set Status LED common anode to HIGH
  digitalWrite(statusLEDvcc, HIGH);
  // Turn off other cathodes
  digitalWrite(statusLEDred, HIGH);
  digitalWrite(statusLEDgreen, HIGH);
  // Set Status LED blue cathode to full LOW
  digitalWrite(statusLEDblue, LOW);
}

/*************************************************************************
Function: statLEDorangeOn()
Purpose:  Turns on the Status LED as ORANGE
**************************************************************************/
void statLEDorangeOn(void){
  // Set Status LED common anode to HIGH
  digitalWrite(statusLEDvcc, HIGH);
  // Turn off BLUE LED
  digitalWrite(statusLEDblue, HIGH);
  // Turn on RED and GREEN LEDs to full LOW
  analogWrite(statusLEDred, 100);
  digitalWrite(statusLEDgreen, LOW);
}


/*************************************************************************
Function: statLEDyellowOn()
Purpose:  Turns on the Status LED as YELLOW
**************************************************************************/
void statLEDyellowOn(void){
  // Set Status LED common anode to HIGH
  digitalWrite(statusLEDvcc, HIGH);
  // Turn off BLUE LED
  digitalWrite(statusLEDblue, HIGH);
  // Turn on RED and GREEN LEDs to LOW
  analogWrite(statusLEDred, 185);
  digitalWrite(statusLEDgreen, LOW);
}

/*************************************************************************
Function: statLEDvioletOn()
Purpose:  Turns on the Status LED as VIOLET
**************************************************************************/
void statLEDvioletOn(void){
  // Set Status LED common anode to HIGH
  digitalWrite(statusLEDvcc, HIGH);
  // Turn on RED, BLUE, and GREEN LEDs to LOW
  analogWrite(statusLEDred, 185);
  digitalWrite(statusLEDblue, LOW);
  digitalWrite(statusLEDgreen, LOW);
}

/*************************************************************************
Function: statLEDwhiteOn()
Purpose:  Turns on the Status LED as WHITE
**************************************************************************/
void statLEDwhiteOn(void){
  // Set Status LED common anode to HIGH
  digitalWrite(statusLEDvcc, HIGH);
  // Turn on RED, BLUE, and GREEN LEDs to LOW
  analogWrite(statusLEDred, 220);
  analogWrite(statusLEDblue, 254);
  //digitalWrite(statusLEDblue, HIGH);
  digitalWrite(statusLEDgreen, LOW);
}

/*************************************************************************
Function: statLEDcyanOn()
Purpose:  Turns on the Status LED as CYAN
**************************************************************************/
void statLEDcyanOn(void){
  // Set Status LED common anode to HIGH
  digitalWrite(statusLEDvcc, HIGH);
  // Turn off RED LED
  digitalWrite(statusLEDred,HIGH);
  // Turn on BLUE and GREEN LEDs to LOW
  analogWrite(statusLEDblue, 255);
  digitalWrite(statusLEDgreen, LOW);
}


