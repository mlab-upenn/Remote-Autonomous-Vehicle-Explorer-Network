/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.0
 February 12, 2011
 
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

/*************************************************************************
Function Headers
**************************************************************************/
void initSonar(void);
void readSonar(void);

/*************************************************************************
Function Varibles
**************************************************************************/
float frontsonar = 0.0;
float backsonar = 0.0;
float leftsonar = 0.0;
float rightsonar = 0.0;
float upsonar = 0.0;
float downsonar = 0.0;
AnalogIn FrontSonar(FrontSonarPin);
AnalogIn BackSonar(BackSonarPin);
AnalogIn LeftSonar(LeftSonarPin);
AnalogIn RightSonar(RightSonarPin);
AnalogIn UpSonar(UpSonarPin);
AnalogIn DownSonar(DownSonarPin);
DigitalOut SonarPulse(SonarPulsePin);

/*************************************************************************
Function: initSonar()
Purpose:  Initializes the LV-MaxSonar-EZ0 Sensors
**************************************************************************/
void initSonar(void){
    // Initialize Analog Pins
    SonarPulse = 0;
}

/*************************************************************************
Function: readSonar()
Purpose:  Read the LV-MaxSonar-EZ0 Sensors
**************************************************************************/
void readSonar(void){
    // Pulse Rx Pin
    SonarPulse = 1;
    //wait(0.000025);
    wait(0.002);
    SonarPulse = 0;
    
    SonarData.right = (3.3*RightSonar.read())/0.0098;
    SonarData.back = (3.3*BackSonar.read())/0.0098;    
    
    //SonarData.front = (5*(FrontSonar.read()))/0.0098;
    //SonarData.back = (5*(BackSonar.read()))/0.0098;
    //SonarData.left = (5*(LeftSonar.read()))/0.0098;
    //SonarData.right = (5*(RightSonar.read()))/0.0098;
    //SonarData.up = (5*(UpSonar.read()))/0.0098;
    //SonarData.down = (5*(DownSonar.read()))/0.0098;
}
