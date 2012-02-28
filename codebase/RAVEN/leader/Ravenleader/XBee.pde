/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.2
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

// TO DO
//  - Implement Follower Send function


/*************************************************************************
//        XBee SERIAL SETUP
*************************************************************************/
// Baud Rate = 115,200
#define XBeeBaud  115200

#define XBeeAvailable  Serial3.available
#define XBeeWrite  Serial3.write
#define XBeeRead  Serial3.read
#define XBeeFlush  Serial3.flush
#define XBeeBegin  Serial3.begin
#define XBeePrint  Serial3.print
#define XBeePrintln  Serial3.println

#define XBeetype 0x01
#define XBeeStartDelim 0x7E

byte baseid0 = 0x00;
byte baseid1 = 0x13;
byte baseid2 = 0xA2;
byte baseid3 = 0x00;
byte baseid4 = 0x40;
byte baseid5 = 0x68;
byte baseid6 = 0xB4;
byte baseid7 = 0xBF;

byte followid0 = 0x00;
byte followid1 = 0x13;
byte followid2 = 0xA2;
byte followid3 = 0x00;
byte followid4 = 0x40;
byte followid5 = 0x47;
byte followid6 = 0x2C;
byte followid7 = 0x6E;

byte receivedid0 = 0x00;
byte receivedid1 = 0x00;
byte receivedid2 = 0x00;
byte receivedid3 = 0x00;
byte receivedid4 = 0x00;
byte receivedid5 = 0x00;
byte receivedid6 = 0x00;
byte receivedid7 = 0x00;

byte frameid = 0x00;
byte receivedLength = 0x00;
byte temp;

/*************************************************************************
//        TRANSMIT PACKET STRUCTURE
*************************************************************************/
// Function 's' 'n' 'p' PT N d1 ... dN     CHK
// Byte      1   2   3  4  5 6  ... N+5  N+6 N+7
// RX Packet Description
// 1 - 3 Each received packet must begin with the three-byte (character) sequence
// "snp" to signal the beginning of a new packet.
// 4 PT specifies the packet type.
// 5 N specifies the number of data bytes to expect.
// 6 - (N+5) d1 through dN contain the N data bytes in the packet.
// (N+6) - (N+7) CHK is a two-byte checksum.

/*************************************************************************
//        API TRANSMIT PACKET STRUCTURE
*************************************************************************/
// NAME                Byte Num    Byte
// Start Delimiter        0        0x7E
// Length              MSB 1       0xXX     Number of bytes between the length and the checksum
//                     LSB 2       0xXX
// Frame Type             3        0x10
// Frame ID               4        0xXX     Identifies the UART data frame for the host to correlate
//                                          with a subsequent ACK (acknowledgement). If set to 0,
//                                          no response is sent.
// 64-bit Address         5        0xXX
// of Destination      MSB 6       0xXX    0x000000000000FFFF = Broadcast
//                        7        0xXX
//                        8        0xXX
//                        9        0xXX
//                       10        0xXX
//                     LSB 11      0xXX
// Reserved              12        0xFF
//                       13        0xFE
// Broadcast Radius      14
// Transmit Options      15        0x0X     bit 0: Diasble ACK    bit 1: Don't atempt route Discovery
// RF Data               16        0xXX
//                       17        0xXX
//                       ..        ....
//                       ..        ....
//Checksum               XX        0xXX     Checksum = 0xFF - (the 8 bit sum of bytes from offset 3 to this byte)
/*************************************************************************/


/*************************************************************************
//        API RECEIVE PACKET STRUCTURE
*************************************************************************/
// NAME                Byte Num    Byte
// Start Delimiter        0        0x7E
// Length              MSB 1       0xXX     Number of bytes between the length and the checksum
//                     LSB 2       0xXX
// Frame Type             3        0x90
// Frame ID               4        0xXX     Identifies the UART data frame for the host to correlate
//                                          with a subsequent ACK (acknowledgement). If set to 0,
//                                          no response is sent.
// 64-bit Source          5        0xXX
// Address of sender   MSB 6       0xXX
//                        7        0xXX
//                        8        0xXX
//                        9        0xXX
//                       10        0xXX
//                     LSB 11      0xXX
// Reserved              12        0xFF
//                       13        0xFE
// Receive Options       14        0x0X     0x01 - Packet Acknowledged  0x02 - Packet was a broadcast packet
// Received Data         15        0xXX
//                       16        0xXX
//                       ..        ....
//                       ..        ....
//Checksum               XX        0xXX     Checksum = 0xFF - (the 8 bit sum of bytes from offset 3 to this byte)
/*************************************************************************/

/*************************************************************************
Function: XBee_init()
Purpose:  Initialize XBee Serial Channel
**************************************************************************/
void XBee_init(void){
  // Open Serial Channel, 115,200 Baud Rate
  XBeeBegin(XBeeBaud);
  
  // Flush buffer
  XBeeFlush();
}

/*************************************************************************
Function: XBeeParseData()
Purpose:  Parse data in XBee Serial Buffer and act according to data type
**************************************************************************/
void XBeeParseData(void){
  // Read from buffer and sync with packet
  byte unitID;
  byte frametype;
  boolean packetfound = false;
  
  while(XBeeAvailable()>17 && packetfound == false){    
    temp = XBeeRead();
    if(temp == XBeeStartDelim){     
      temp = XBeeRead();
      temp = XBeeRead();
      frametype = XBeeRead();
      if(frametype == 0x90){
        packetfound = true;
      }
    }
  }
  
  if(packetfound == true){
    receivedid0 = XBeeRead();           // Sender ID
    receivedid1 = XBeeRead();
    receivedid2 = XBeeRead();
    receivedid3 = XBeeRead();
    receivedid4 = XBeeRead();
    receivedid5 = XBeeRead();
    receivedid6 = XBeeRead();
    receivedid7 = XBeeRead();
    temp = XBeeRead();                  // Reserved Byte
    temp = XBeeRead();                  // Reserved Byte
    temp = XBeeRead();                  // Receive Options
    unitID = XBeeRead();                // Unit ID
    if(unitID == 0x00){
      temp = XBeeRead();
      BaseCommands(temp);               // Base Station Command Packet
    }
    temp = XBeeRead();                  // Read last byte (checksum)
  }
}

/*************************************************************************
Function: XBee_sendFollower()
Purpose:  
**************************************************************************/
void XBee_sendFollower(void){
  // Send follower the required data
    char sum = 0;
  // Send Packet to XBee
  
  XBeeWrite(0x7E);                       // Start Delimeter
  XBeeWrite((byte)0);                       //Length
  XBeeWrite(0x12);                       //14 + payload (18 total)
  XBeeWrite(0x10);                       // Frame Type
  sum +=0x10;
  XBeeWrite((byte)0);                    // Frame ID
  sum += 0x00;
  
  XBeeWrite(followid0);                    // Destination Address
  sum+=followid0;
  XBeeWrite(followid1);
  sum+=followid1;
  XBeeWrite(followid2);
  sum+=followid2;
  XBeeWrite(followid3);
  sum+=followid3;
  XBeeWrite(followid4);
  sum+=followid4;
  XBeeWrite(followid5);
  sum+=followid5;
  XBeeWrite(followid6);
  sum+=followid6;
  XBeeWrite(followid7);
  sum+=followid7;
  // Reserved Bytes
  XBeeWrite(0xFF);
  sum+=0xFF;
  XBeeWrite(0xFE);
  sum+=0xFE;
  // Broadcase Radius
  XBeeWrite((byte)0);
  sum+=0x00;
  // Transmit Options
  XBeeWrite((byte)0);
  sum+=0x00;
  
  // UNIT ID
  XBeeWrite(0x01);
  sum+=0x01;
  
  // YAW DATA
  int yaw_int = (int)(yaw/YAW_FACTOR);
  char yaw_high = (char)(yaw_int>>8);
  char yaw_low = (char)(yaw_int);
  XBeeWrite(yaw_high);
  sum +=yaw_high;
  XBeeWrite(yaw_low);
  sum +=yaw_low;
  // COMMAND BYTE
  XBeeWrite((byte)0);
  sum += 0x00;
  
  //Checksum
  sum = 0xFF-sum;
  XBeeWrite(sum); 
}

/*************************************************************************
Function: XBee_sendBase()
Purpose:  
**************************************************************************/
// Send data to Base Station
void XBee_sendBase(void){
  char sum = 0;
  // Send Packet to XBee
  
  XBeeWrite(0x7E);                       // Start Delimeter
  XBeeWrite((byte)0);                    //Length
  XBeeWrite(0x13);
  XBeeWrite(0x10);                       // Frame Type
  sum +=0x10;
  XBeeWrite((byte)0);                    // Frame ID
  sum += 0x00;
  
  XBeeWrite(baseid0);                    // Destination Address
  sum+=baseid0;
  XBeeWrite(baseid1);
  sum+=baseid1;
  XBeeWrite(baseid2);
  sum+=baseid2;
  XBeeWrite(baseid3);
  sum+=baseid3;
  XBeeWrite(baseid4);
  sum+=baseid4;
  XBeeWrite(baseid5);
  sum+=baseid5;
  XBeeWrite(baseid6);
  sum+=baseid6;
  XBeeWrite(baseid7);
  sum+=baseid7;
  // Reserved Bytes
  XBeeWrite(0xFF);
  sum+=0xFF;
  XBeeWrite(0xFE);
  sum+=0xFE;
  // Broadcase Radius
  XBeeWrite((byte)0);
  sum+=0x00;
  // Transmit Options
  XBeeWrite(0x01);
  sum+=0x01;
  
  // RF DATA
  uint8_t data = 0;
  int dataint = 0;
  float dataadjust = 0;
  // Identifier = 1 (Leader)
  XBeeWrite(0x01); 
  sum+=0x01;
  // Yaw
  dataadjust = yaw;
  if(dataadjust<0){
    dataadjust+=360.0;
  }
  dataadjust = (dataadjust/360.0)*255.0;
  dataint = (int)dataadjust;
  data = (uint8_t)dataint;
  //Serial.println((uint16_t)data);
  XBeeWrite(data);
  sum +=data;
  // Pitch
  dataadjust = pitch;
  if(dataadjust<0){
    dataadjust+=360.0;
  }
  dataadjust = (dataadjust/360.0)*255.0;
  dataint = (int)dataadjust;
  data = (uint8_t)dataint;
  XBeeWrite(data);
  sum +=data;
  // Roll
  dataadjust = roll;
  if(dataadjust<0){
    dataadjust+=360.0;
  }
  dataadjust = (dataadjust/360.0)*255.0;
  dataint = (int)dataadjust;
  data = (uint8_t)dataint;
  XBeeWrite(data);
  sum +=data;
  
  // Battery
  XBeeWrite(battery);
  sum +=battery;
  
  //Checksum
  sum = 0xFF-sum;
  XBeeWrite(sum); 
}
