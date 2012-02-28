/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.0
 February 11, 2011
 
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

/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.0
 February 11, 2010
 
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
//        CHR-6dm AHRS Value Conversion Factors
*************************************************************************/
#define YAW_FACTOR 0.0109863F         //  &#65533;/LSB
#define PITCH_FACTOR (0.0109863F*90.0F/72.0F)       //  &#65533;/LSB
#define ROLL_FACTOR (0.0109863F*90.0F/72.0F)        //  &#65533;/LSB
#define YAW_RATE_FACTOR 0.0137329F    //  &#65533;/s/LSB
#define PITCH_RATE_FACTOR 0.0137329F  //  &#65533;/s/LSB
#define ROLL_RATE_FACTOR 0.0137329F   //  &#65533;/s/LSB
#define MAGX_FACTOR 0.061035F         //  mGauss/LSB
#define MAGY_FACTOR 0.061035F         //  mGauss/LSB
#define MAGZ_FACTOR 0.061035F         //  mGauss/LSB
#define GYROX_FACTOR 0.01812F         //  &#65533;/s/LSB
#define GYROY_FACTOR 0.01812F         //  &#65533;/s/LSB
#define GYROZ_FACTOR 0.01812F         //  &#65533;/s/LSB
#define ACCELX_FACTOR 0.106812F       //  mg/LSB
#define ACCELY_FACTOR 0.106812F       //  mg/LSB
#define ACCELZ_FACTOR 0.106812F       //  mg/LSB

/*************************************************************************
Function Headers
**************************************************************************/
void initIMU();
void IMUread();


/*************************************************************************
Function: initIMU()
Purpose:  initializes serial for the CH-Robotics ch6dm IMU
**************************************************************************/
void initIMU(){
    IMUSerial.baud(115200);
    
    // Zero all variables
    MyAttitude.Yaw_zero = 0.0;
    MyAttitude.Pitch_zero = 0.0;
    MyAttitude.Roll_zero = 0.0;
    MyAttitude.Yaw = 0.0;
    MyAttitude.Pitch = 0.0;
    MyAttitude.Roll = 0.0;
    MyAttitude.YawRate = 0.0;
    MyAttitude.PitchRate = 0.0;
    MyAttitude.RollRate = 0.0;    
}


/*************************************************************************
Function: readIMU()
Purpose:  reads a packet from the CH-Robotics ch6dm IMU
**************************************************************************/
void IMUread(){
    char temp;
    char packettype;
    char packetFound = 0;
    if(IMUSerial.rxBufferGetCount() == 0){
        return;
    }
    while(IMUSerial.rxBufferGetCount() > 20){
        if(IMUSerial.getc() == 's'){
            if(IMUSerial.getc() == 'n'){
                if(IMUSerial.getc() == 'p'){
                    packetFound = 1;
                    break;
                }
            }
        }
    }
         
    if(packetFound == 1){
        checksum +='s';
        checksum +='n';
        checksum +='p';
        packettype = IMUSerial.getc(); //Packet Type
        checksum += packettype;
        if(packettype != 0xB7){
            return;
        }
        temp = IMUSerial.getc(); //num. data bytes
        checksum += temp;
        temp = IMUSerial.getc(); //active channels
        checksum += temp;
        temp = IMUSerial.getc(); //active channels
        checksum += temp;
        high = IMUSerial.getc(); //yaw MSB
        checksum += high;
        low = IMUSerial.getc(); //yaw LSB
        checksum += low;
        yawnum = (short)high<<8;
        yawnum |= low;
        
        high = IMUSerial.getc(); //pitch MSB
        checksum += high;
        low = IMUSerial.getc(); //pitch LSB
        checksum += low;
        pitchnum = (short)high<<8;
        pitchnum |= low;
        
        high = IMUSerial.getc(); //roll MSB
        checksum += high;
        low = IMUSerial.getc(); //roll LSB
        checksum += low;
        rollnum = (short)high<<8;
        rollnum |= low;
        
        high = IMUSerial.getc(); //yaw_rate MSB
        checksum += high;
        low = IMUSerial.getc(); //yaw_rate LSB
        checksum += low;
        yawratenum = (short)high<<8;
        yawratenum |= low;
        
        high = IMUSerial.getc(); //pitch_rate MSB
        checksum += high;
        low = IMUSerial.getc(); //pitch_rate LSB
        checksum += low;
        pitchratenum = (short)high<<8;
        pitchratenum |= low;
        
        high = IMUSerial.getc(); //roll_rate MSB
        checksum += high;
        low = IMUSerial.getc(); //roll_rate LSB
        checksum += low;
        rollratenum = (short)high<<8;
        rollratenum |= low;
        

        checkhigh = IMUSerial.getc(); //checksum
        checklow = IMUSerial.getc(); //checksum
        if(((checkhigh<<8)|(checklow))==checksum){
            // Correct checksum
            MyAttitude.Yaw = yawnum*YAW_FACTOR + MyAttitude.Yaw_zero;
            MyAttitude.Pitch = pitchnum*PITCH_FACTOR + MyAttitude.Pitch_zero;
            MyAttitude.Roll = rollnum*ROLL_FACTOR + MyAttitude.Roll_zero;
            MyAttitude.YawRate = yawratenum*YAW_RATE_FACTOR;
            MyAttitude.PitchRate = pitchratenum*PITCH_RATE_FACTOR;
            MyAttitude.RollRate = rollratenum*ROLL_RATE_FACTOR;
        }
        packetFound = 0;
        checksum = 0;
     }
     return;
}