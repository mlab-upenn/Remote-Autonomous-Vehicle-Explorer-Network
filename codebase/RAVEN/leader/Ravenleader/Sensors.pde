/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.1
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

/*************************************************************************
Function: read_battery()
Purpose:  Reads battery voltage (10*(Battery Voltage) i.e. 114 = 11.4V
          If battery lower than LOW_VOLTAGE, sets LowBatteryFlag
**************************************************************************/
void read_battery(void)
{
  int batteryvoltage = analogRead(BatterySig);
  batteryvoltage = (((batteryvoltage*INPUT_VOLTAGE)/1024)*10)*VOLT_DIV;
  battery = (uint8_t)batteryvoltage;
  if(battery < (10*LOW_VOLTAGE)){
    LowBatteryFlag = true;
  }
  else LowBatteryFlag = false;
    
}

/*************************************************************************
Function: set_GroundTempPress()
Purpose:  Reads Pressure Sensor and sets ground temperature and pressure
**************************************************************************/
void set_GroundTempPress(void){
  APM_BMP085.Read();
  delay(100);
  APM_BMP085.Read();
  delay(400);
  APM_BMP085.Read();
  delay(400);
  ground_temperature = APM_BMP085.Temp;
  ground_pressure = APM_BMP085.Press;
}

/*************************************************************************
Function: read_pressure()
Purpose:  Reads Pressure Sensor and updates relative altitude
**************************************************************************/
void read_pressure(void){
  // read Pressure Sensor
  APM_BMP085.Read();
  
  //convert Pascals to centimeters relative to starting altitude
  long temp;
  temp = (APM_BMP085.Press-ground_pressure)*100;
  pressureAlt = (temp*83.152)/1000;
}



/*
void loop()
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
 
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
 
  delay(100);
}

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
*/
