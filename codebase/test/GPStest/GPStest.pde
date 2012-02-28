/********************************************************************
 * R.A.V.E.N. Quadrotor
 * www.AirHacks.com
 * Copyright (c) 2011.  All rights reserved.
 * 
 * Version: 1.0
 * April 7, 2011
 * 
 * Authors:
 * Andrew Botelho (University of Pennsylvania BSE Electrical Engineering '14)
 * 
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 ********************************************************************/


/* ****************************************************************************** */
/*     Set-Up Definitions 
/* ****************************************************************************** */

#define GPS_Baud 115200 //Defines baud rate (115200)
#define GPS_SerialAvailable  Serial1.available
#define GPS_SerialWrite      Serial1.write
#define GPS_SerialRead       Serial1.read
#define GPS_SerialPeek       Serial1.peek
#define GPS_SerialFlush      Serial1.flush
#define GPS_SerialBegin      Serial1.begin
#define GPS_SerialEnd        Serial1.end


/* ****************************************************************************** */
/*     Global Variables
/* ****************************************************************************** */
boolean foundPacket = false;
byte timeArray[6];
int timeZone = -4;
int time_hours = 0;
int time_minutes = 0;
int time_seconds = 0;
char degree =0xF8;
char minutes = 0x27;
char seconds = 0x22;
byte latArray[8];
int latDeg = 0;
int latMin = 0;
float latSec = 0;
byte longArray[9];
int longDeg = 0;
int longMin = 0;
float longSec = 0;
char headingNS = 0x00;
char headingEW = 0x00;
int lockInD = 0;
byte satArray[2];
int satsUsed = 0;
byte hdopArray[3];
float hdop = 0.00; 
byte mslArray[6];
int mslArrayLength;
byte





/* ****************************************************************************** */
/*     GPS Set-Up Fucntion
/* ****************************************************************************** */
void setup (void){
  GPS_SerialBegin(9600); //Sets Baud Rate to 9600 (GPS Default) (Board to GPS)
  GPS_SerialWrite ("$PMTK251,38400*27*1F\r\n"); //Sets Baud Rate to 115200 (GPS ro Boad)
  delay(2000);
  GPS_SerialFlush();
  GPS_SerialEnd();
  GPS_SerialBegin(38400); //Sets Baud Rate to 115200 (Board to GPS)
  GPS_SerialWrite ("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); //Sets GPS to output GGA mode once every position fix
  Serial.begin(GPS_Baud); //Sets Baud Rate to 115200 (Board to Computer)
}

//
void loop (void){
  if(GPS_SerialAvailable()>45){
    //Checks for Start Delimiter
    if(GPS_SerialRead()=='$'){
      if(GPS_SerialRead()=='G'){
        if(GPS_SerialRead()=='P'){
          if(GPS_SerialRead()=='G'){
            if(GPS_SerialRead()=='G'){ 
              if(GPS_SerialRead()=='A'){
                foundPacket = true; 
                //Start Parse
                GPS_SerialRead();
                Serial.print("GPS Data is:\r\n\r\nTime:\r\n");
                //Read Time, Divide into hours, minutes, and seconds
                for (int i=0; i<=5; i++){
                  timeArray[i]= (uint8_t)(GPS_SerialRead()-0x30);
                }
                for (int i=0; i<=3; i++){
                  GPS_SerialRead();
                }
                time_hours = ((int)(timeArray[0]*10+timeArray[1])+timeZone)%24; //Sets hours, adjusts for time zone
                Serial.print(time_hours);
                Serial.print(" hours, ");
                time_minutes = (int)timeArray[2]*10+timeArray[3];
                Serial.print(time_minutes);
                Serial.print(" minutes, ");
                time_seconds = (int)timeArray[4]*10+timeArray[5];
                Serial.print(time_seconds);
                Serial.print(" seconds.\r\n\r\n");
                GPS_SerialRead();          
                //Read Lattitude, Divide into Degrees, Minutes, Seconds
                Serial.print("Latitude:\r\n");
                while(GPS_SerialPeek() != ','){
                  for (int i=0; i<=7; i++){
                    if (GPS_SerialPeek() != '.'){
                      latArray[i]= ((uint8_t)(GPS_SerialRead()-0x30));
                    }else GPS_SerialRead();
                  }
                }
                latDeg = ((int)(latArray[0]*10+latArray[1])); //Sets degrees
                Serial.print(latDeg);
                Serial.print(degree);
                latMin = ((int)(latArray[2]*10+latArray[3])); //Sets minutes
                Serial.print(latMin);
                Serial.print(minutes);
                latSec = ((float)latArray[4]+(float)latArray[5]/10+(float)latArray[6]/100+(float)latArray[7]/1000)*60;//Check Conversion Factor
                Serial.print(latSec);
                Serial.print(seconds);
                Serial.print("\r\n\r\n");
                //Read N/S
                if (GPS_SerialPeek() != ','){
                  headingNS = GPS_SerialRead();
                }else GPS_SerialRead();
                //Read Longitude, Divide into Degrees, Minutes, Seconds
                Serial.print("Longitude:\r\n");
                while(GPS_SerialPeek() != ','){
                  for (int i=0; i<=8; i++){
                    if (GPS_SerialPeek() != '.'){
                      longArray[i]= ((uint8_t)(GPS_SerialRead()-0x30));
                    }else GPS_SerialRead();
                  }
                }
                longDeg = ((int)(longArray[0]*100+longArray[1]*10+longArray[2])); //Sets degrees
                Serial.print(longDeg);
                Serial.print(degree);
                longMin = ((int)(longArray[3]*10+longArray[4])); //Sets minutes
                Serial.print(longMin);
                Serial.print(minutes);
                longSec = ((float)longArray[5]+(float)longArray[6]/10+(float)longArray[7]/100+(float)longArray[8]/1000)*60;//Check Conversion Factor
                Serial.print(longSec);
                Serial.print(seconds);
                Serial.print("\r\n\r\n");
                //Read E/W
                if (GPS_SerialPeek() != ','){
                  headingEW = GPS_SerialRead();
                }else GPS_SerialRead();
                //Print Heading
                Serial.print("Heading:\r\n");
                Serial.print(headingNS);
                Serial.print(headingEW);
                Serial.print("\r\n\r\n");
                //Lock-On Indicator
                headingEW = GPS_SerialRead();
                //Lock-On Indicator
                while (GPS_SerialPeek() == ','){
                  GPS_SerialRead();
                }
                lockInD = (int)GPS_SerialRead();
                Serial.print("Lock On Status:\r\n");
                switch (lockInD){
                 case 0: 
                   Serial.print("Lock not available"); 
                   break;
                 case 1: 
                   Serial.print("GPS Lock");
                   break;
                 case 2: 
                   Serial.print("Differential GPS Lock");
                   break;
                 default: 
                   Serial.print("Lock not available"); 
                }
                Serial.print("\r\n\r\n");
                //Sattelites Used
                while(GPS_SerialPeek() != ','){
                  for (int i=0; i<=1; i++){
                    satArray[i]= ((uint8_t)(GPS_SerialRead()-0x30));
                  }
                }
                satsUsed = satArray[0]*10+satArray[1];
                Serial.print("Number of Sattelites Used:\r\n");
                Serial.print(satsUsed);
                Serial.print("\r\n\r\n");
                //Horizontal Dilution of Precision
                while(GPS_SerialPeek() != ','){
                  for (int i=0; i<=2; i++){
                    if (GPS_SerialPeek() != '.'){
                      hdopArray[i]= ((float)(GPS_SerialRead()-0x30));
                    }else GPS_SerialRead();
                  }
                }
                hdop = hdopArray[0]+(hdopArray[1]/10)+(hdopArray[2]/100);
                Serial.print("Horizontal Diultion of Precision:\r\n");
                Serial.print(hdop);
                Serial.print("\r\n\r\n");
                //MSL Altitude
                Serial.print("MSL Altitude:\r\n");
                while(GPS_SerialPeek() != ','){
                  for (int i=0; i<=5; i++){
                    if (GPS_SerialPeek() != '.'){
                      longArray[i]= ((uint8_t)(GPS_SerialRead()-0x30));
                    }else GPS_SerialRead();
                  }
                }
                Serial.print("\r\n\r\n\r\n\r\n");
              }
            }
          }
        }
      }
    }

  }
}






