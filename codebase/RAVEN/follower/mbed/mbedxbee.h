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

// -To Base Station Packet Structure-
// Data Bytes
// Byte 1: Unit ID
// Byte 2: Yaw
// Byte 3: Pitch
// Byte 4: Roll
// Byte 5: Battery
// Byte 6: IR1x
// Byte 7: IR1y
// Byte 8: IR2x
// Byte 9: IR2y
// Byte 10: Displacement X - Hi
// Byte 11: Disclacement X - Lo
// Byte 12: Displacement Y - Hi
// Byte 13: Disclacement Y - Lo
// Byte 14: Displacement Z - Hi
// Byte 15: Disclacement Z - Lo
// Byte 16: Sonar - Front
// Byte 17: Sonar - Back
// Byte 18: Sonar - Left
// Byte 19: Sonar - Right
// Byte 20: Sonar - Up
// Byte 21: Sonar - Down
// Byte 22: Spare
// Byte 23: Spare
// Byte 24: Spare

/*************************************************************************
Function Headers
**************************************************************************/
void initXbee(void);
void xbeeRxISR();
char xbeeGetc();
void xbeeRead(void);
void xbeeSendBase();
void spinUp(void);
void spinDown(void);
void follow(void);

/*************************************************************************
Base Station (BS) Serial Address
**************************************************************************/
#define BS_SH1 0x00
#define BS_SH2 0x13
#define BS_SH3 0xA2
#define BS_SH4 0x00
#define BS_SL1 0x40
#define BS_SL2 0x68
#define BS_SL3 0xB4
#define BS_SL4 0xBF

/*************************************************************************
Lead Unit (LU) Serial Address
**************************************************************************/
#define LU_SH1 0x00
#define LU_SH2 0x13
#define LU_SH3 0xA2
#define LU_SH4 0x00
#define LU_SL1 0x40
#define LU_SL2 0x66
#define LU_SL3 0xF0
#define LU_SL4 0x54

/*************************************************************************
Function: initXbee()
Purpose:  Initializes the serial communication for the XBee Transciever
**************************************************************************/
void initXbee(void){
    xbee_serial.baud(115200);
    // Initialize circular buffer
    //xbee_rx_head = 0;
    //xbee_rx_tail = 0;
    //xbee_bytes_available = 0;
    //xbee_serial.attach(&xbeeRxISR, Serial::RxIrq);
    
}





/*************************************************************************
Function: xbeeRead()
Purpose:  reads a packet from the XBee
**************************************************************************/
void xbeeRead(){
    char thisByte;
    char unitID;
    char receivedCommand;
    int16_t receivedyaw = 0;
    int yawconverted = 0;
    char packetFound = 0;
    if(xbee_serial.rxBufferGetCount() == 0){
        return;
    }
    while(xbee_serial.rxBufferGetCount() > 14){
        thisByte = xbee_serial.getc();
        if(thisByte == 0x7E){
            thisByte =  xbee_serial.getc(); //length high
            thisByte =  xbee_serial.getc(); //length low
            thisByte =  xbee_serial.getc(); //packet type
            if(thisByte == 0x90){
                // packet is rx type
                packetFound = 1;
                break;
            }
        }
    }
    if(packetFound == 1){
        // we found a packet, parse 11 frame bytes and then the payload
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        thisByte =  xbee_serial.getc();
        
        // Payload:
        unitID =  xbee_serial.getc();
        //pc.printf("unitID = %x\r\n",unitID);   
        
        // Switch case for sending unit
        switch(unitID){
            case 0x00:  // Base Station
                receivedCommand =  xbee_serial.getc();
                // Switch case for received command
                switch(receivedCommand){
                    case 0xA1:  // Spin up
                        spinUp();
                        //pc.printf("1");
                        break;
                    case 0xB2:  // Auto-Land
                        //pc.printf("2");
                        break;
                    case 0xC3:  // Emergency Stop
                        spinDown();
                        //pc.printf("3");
                        break;
                    case 0xD4:  // Follow
                        follow();
                        //pc.printf("4");
                        break;
                    case 0xE5:
                        //pc.printf("5");
                        break;
                    case 0xF6:
                        //pc.printf("6");
                        break;
                    case 0x1A:
                        //pc.printf("7");
                        break;
                    case 0x2B:
                        //pc.printf("8");
                        break;
                    case 0x3C:
                        //pc.printf("9");
                        break;
                    default:
                        //pc.printf("unrecognized command");
                        break;
                }
                break;
            case 0x01:  // Leader
                // Read in Leader Information
                // Yaw (2 bytes)
                yawhi =  xbee_serial.getc();
                receivedyaw |=(yawhi<<8);
                yawlow =  xbee_serial.getc();
                receivedyaw |=yawlow;
                yawconverted = (int)receivedyaw;
                LeaderData.Yaw = (float)(yawconverted*yawFactor);
                //pc.printf("Yaw = %f \r\n",LeaderData.Yaw);
                //pc.printf("Yaw = %d \rn",(((int16_t)(yawhi<<8)|yawlow)));
                //LeaderData.Yaw= ((float)((int16_t)(yawhi<<8|yawlow)))*yawFactor;
                LeaderData.command =  xbee_serial.getc();
                //pc.printf("Yaw = %d \r\n", LeaderData.Yaw);                
                break;
            default:
                break;
        }
        
        // Checksum - unhandled
        thisByte =  xbee_serial.getc();       
     }
}
/*************************************************************************
Function: xbeeSendBase()
Purpose:  Sends a packet to the XBee - 16 bytes of information
          Transmits to the Base Station
**************************************************************************/
void xbeeSendBase(){
    uint16_t tempval;
    int inttemp;
    char chartemp;
    char checksum = 0;
    char frameID = 0;  // No ACK
    xbee_serial.putc(0x7E);
    xbee_serial.putc(0x00);
    xbee_serial.putc(0x26); //length = 14 + payload = 38
    xbee_serial.putc(0x10); //tx request
    checksum += 0x10;
    xbee_serial.putc((char)frameID);
    checksum += (char)frameID;
    
    // destination serial address
    xbee_serial.putc(BS_SH1);
    checksum += BS_SH1;
    xbee_serial.putc(BS_SH2);
    checksum += BS_SH2;
    xbee_serial.putc(BS_SH3);
    checksum += BS_SH3;
    xbee_serial.putc(BS_SH4);
    checksum += BS_SH4;
    xbee_serial.putc(BS_SL1);
    checksum += BS_SL1;
    xbee_serial.putc(BS_SL2);
    checksum += BS_SL2;
    xbee_serial.putc(BS_SL3);
    checksum += BS_SL3;
    xbee_serial.putc(BS_SL4);
    checksum += BS_SL4;
    
    // reserved bytes and options
    xbee_serial.putc(0xFF);
    checksum += 0xFF;
    xbee_serial.putc(0xFE);
    checksum += 0xFE;
    xbee_serial.putc(0x00);
    checksum += 0x00;
    xbee_serial.putc(0x00);
    checksum += 0x00;
    
    
    // Payload Data (24 Bytes):
    // Byte 1: Unit ID
    xbee_serial.putc(0x02);
    checksum += 0x02;
    // Byte 2: Yaw
     xbee_serial.putc(0xBB);
    checksum += 0xBB;
    // Byte 3: Pitch
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 4: Roll
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 5: Battery
    xbee_serial.putc(0x10);
    checksum += 0x10;
    // Byte 6: IR1x
    inttemp = (IRdata.p1x*255)/1024;
    chartemp = (char)inttemp;
    xbee_serial.putc(chartemp);
    checksum += chartemp;
    // Byte 7: IR1y
    inttemp = (IRdata.p1y*255)/1024;
    chartemp = (char)inttemp;
    xbee_serial.putc(chartemp);
    checksum += chartemp;
    // Byte 8: IR2x
    inttemp = (IRdata.p2x*255)/1024;
    chartemp = (char)inttemp;
    xbee_serial.putc(chartemp);
    checksum += chartemp;
    // Byte 9: IR2y
    inttemp = (IRdata.p2y*255)/1024;
    chartemp = (char)inttemp;
    xbee_serial.putc(chartemp);
    checksum += chartemp;
    // Byte 10: Displacement X - Hi
    tempval = (uint16_t)(TrackingData.dist_x + 1000);
    xbee_serial.putc((char)(tempval>>8));
    checksum += (char)(tempval>>8);
    
    // Byte 11: Dispclacement X - Lo
    xbee_serial.putc((char)(tempval));
    checksum += (char)(tempval);
    
    // Byte 12: Displacement Y - Hi
    tempval = (uint16_t)(TrackingData.dist_y + 1000);
    xbee_serial.putc((char)(tempval>>8));
    checksum += (char)(tempval>>8);
    
    // Byte 13: Disclacement Y - Lo
    xbee_serial.putc((char)(tempval));
    checksum += (char)(tempval);
    
    // Byte 14: Displacement Z - Hi
    tempval = (uint16_t)(TrackingData.dist_z + 1000);
    xbee_serial.putc((char)(tempval>>8));
    checksum += (char)(tempval>>8);
    
    // Byte 15: Disclacement Z - Lo
    xbee_serial.putc((char)(tempval));
    checksum += (char)(tempval);
    
    // Byte 16: Sonar - Front
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 17: Sonar - Back
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 18: Sonar - Left
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 19: Sonar - Right
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 20: Sonar - Up
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 21: Sonar - Down
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 22: Spare
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 23: Spare
    xbee_serial.putc(0x00);
    checksum += 0x00;
    // Byte 24: Spare
    xbee_serial.putc(0x00);
    checksum += 0x00;
    
    // write checksum and finish
    xbee_serial.putc(0xFF - (checksum & 0xFF));
}

/*************************************************************************
Function: spinUp()
Purpose:  Starts the Props Spinning
**************************************************************************/
void spinUp(void){
    RCoutput.ch4 = 1950;
    RCoutput.ch3 = 1000;
    sendRC();
    wait(0.5);
    
    RCoutput.ch4 = 1500;
    sendRC();
}

void spinDown(void){
    RCoutput.ch4 = 1050;
    RCoutput.ch3 = 1000;
    sendRC();
    wait(0.5);
    
    RCoutput.ch4 = 1500;
    sendRC();
}

void follow(void){
    followleader = true;
    wait(1);
    xbee_serial.rxBufferFlush();
    wait(.01);
    leaderyawoffset=0;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 1
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 2
    leaderyawoffset/=2;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 3
    leaderyawoffset/=2;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 4
    leaderyawoffset/=2;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 5
    leaderyawoffset/=2;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 6
    leaderyawoffset/=2;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 7
    leaderyawoffset/=2;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 8
    leaderyawoffset/=2;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 9
    leaderyawoffset/=2;
    xbeeRead();
    leaderyawoffset+=(LeaderData.Yaw-MyAttitude.Yaw);        // 10
    leaderyawoffset/=2;
}   