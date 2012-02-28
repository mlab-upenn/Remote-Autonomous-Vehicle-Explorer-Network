/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.0
 February 9, 2010
 
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
Function: initIRcam()
Purpose:  Initializes the Pixart InfraRed Camera
          and sets the servo to center (1500us)
**************************************************************************/
void initIRcam(void)
{
  // Initialize IR Camera Sensor
  IRslaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
  writeTwoI2C(0x30,0x01);
  delay(10);
  writeTwoI2C(0x30,0x08);
  delay(10);
  writeTwoI2C(0x06,0x90);
  delay(10);
  writeTwoI2C(0x08,0xC0);
  delay(10);
  writeTwoI2C(0x1A,0x40);
  delay(10);
  writeTwoI2C(0x33,0x33);
  delay(110);
  
  // Center IR Camera Servo
  APM_RC.OutputCh(IRcameraServoCh, IRcameraServoCenter);
}

/*************************************************************************
Function: stabilizeIRcam()
Purpose:  Stabilizes the Pixart InfraRed Camera based on the quadrotor PITCH
**************************************************************************/
void stabilizeIRcam(void)
{
  IRcameraServo = constrain(((int)pitch*11+IRcameraServoCenter),1010,1950);
  APM_RC.OutputCh(IRcameraServoCh, IRcameraServo);
}

/*************************************************************************
Function: readIRcam()
Purpose:  Obtains data from the Pixart InfraRed Camera
**************************************************************************/
void readIRcam(void)
{
  Wire.beginTransmission(IRslaveAddress);
  Wire.send(0x36);
  Wire.endTransmission();
  int i = 0;
  Wire.requestFrom(IRslaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
  for(i=0;i<16;i++){
    ir_data[i]=0;                             // Clear IR Data Array
  }
  i = 0;
  while(Wire.available() && i < 16){
    ir_data[i] = Wire.receive();
    i++;
  }
  
  // IR Point 1
  point1x = ir_data[1];
  point1y = ir_data[2];
  ir_s   = ir_data[3];
  point1x += (ir_s & 0x30) <<4;
  point1y += (ir_s & 0xC0) <<2;
  point1s = (ir_s & 0x0F);
  
  // IR Point 2
  point2x = ir_data[4];
  point2y = ir_data[5];
  ir_s   = ir_data[6];
  point2x += (ir_s & 0x30) <<4;
  point2y += (ir_s & 0xC0) <<2;
  point2s = (ir_s & 0x0F);
  
  // IR Point 3
  point3x = ir_data[7];
  point3y = ir_data[8];
  ir_s   = ir_data[9];
  point3x += (ir_s & 0x30) <<4;
  point3y += (ir_s & 0xC0) <<2;
  point3s = (ir_s & 0x0F);
  
  // IR Point 4
  point4x = ir_data[10];
  point4y = ir_data[11];
  ir_s   = ir_data[12];
  point4x += (ir_s & 0x30) <<4;
  point4y += (ir_s & 0xC0) <<2;
  point4s = (ir_s & 0x0F); 
}

/*************************************************************************
Function: writeTwoI2C
Purpose:  Writes two bytes on the I2C bus to the Pixart IR Camera Sensor
Inputs: byte data1, byte data2 = data bytes to write to I2C bus
**************************************************************************/
void writeTwoI2C(byte data1, byte data2)
{
    Wire.beginTransmission(IRslaveAddress);
    Wire.send(data1); 
    Wire.send(data2);
    Wire.endTransmission();
}
