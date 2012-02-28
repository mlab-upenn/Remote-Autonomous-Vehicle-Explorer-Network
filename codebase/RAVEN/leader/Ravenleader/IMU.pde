/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.6
 March 27, 2011
 
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
//  - Get correct hex value for ZERO_RATE_GYROS and put into Calibrate function
//  - Do something if IMU didn't initialize successfully
//  - Check if more/less delay is required during startup


/*************************************************************************
//        IMU SERIAL SETUP
*************************************************************************/
// Baud Rate = 115,200
#define IMUSerialBaud  115200
#define IMUSerialAvailable  Serial2.available
#define IMUSerialWrite  Serial2.write
#define IMUSerialRead  Serial2.read
#define IMUSerialFlush  Serial2.flush
#define IMUSerialBegin  Serial2.begin

/*************************************************************************
//        PACKET STRUCTURE
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
//        CHR-6dm AHRS Value Conversion Factors
*************************************************************************/
#define YAW_FACTOR (0.0109863F*135.0F/100.0F)         //  °/LSB
#define PITCH_FACTOR (0.0109863F*90.0F/72.0F)       //  °/LSB
#define ROLL_FACTOR (0.0109863F*90.0F/72.0F)        //  °/LSB
#define YAW_RATE_FACTOR 0.0137329F    //  °/s/LSB
#define PITCH_RATE_FACTOR 0.0137329F  //  °/s/LSB
#define ROLL_RATE_FACTOR 0.0137329F   //  °/s/LSB
#define MAGX_FACTOR 0.061035F         //  mGauss/LSB
#define MAGY_FACTOR 0.061035F         //  mGauss/LSB
#define MAGZ_FACTOR 0.061035F         //  mGauss/LSB
#define GYROX_FACTOR 0.01812F         //  °/s/LSB
#define GYROY_FACTOR 0.01812F         //  °/s/LSB
#define GYROZ_FACTOR 0.01812F         //  °/s/LSB
#define ACCELX_FACTOR 0.106812F       //  mg/LSB
#define ACCELY_FACTOR 0.106812F       //  mg/LSB
#define ACCELZ_FACTOR 0.106812F       //  mg/LSB

/*************************************************************************
//        IMU RX PACKET DEFINITIONS (-> IMU)
*************************************************************************/
#define SET_ACTIVE_CHANNELS 0x80    // Specifies which channel data should be transmitted over the UART.
#define SET_SILENT_MODE 0x81        // Enables "Silent Mode." In Silent Mode, the AHRS only reports data when
                                    // a GET_DATA packet is received.
#define SET_BROADCAST_MODE 0x82     // Enables "Broadcast Mode." In Broadcast Mode, the AHRS automatically 
                                    // transmits sensor data every Ts milliseconds, where Ts is encoded in 
                                    // the data section of the SET_BROADCAST_MODE packet.
#define SET_GYRO_BIAS 0x83          // Manually sets the x-axis rate gyro bias term. The bias term can be 
                                    // automatically set for all gyro axes by sending a ZERO_RATE_GYROS packet.
#define SET_ACCEL_BIAS 0x84         // Manually sets the x-axis accelerometer bias term.
#define SET_ACCEL_REF_VECTOR 0x85   // Manually sets the accelerometer reference vector used by the EKF to determine 
                                    // "which way is down"
#define AUTO_SET_ACCEL_REF 0x86     // Causes the AHRS to set the current accelerometer measurements as the reference
                                    // vector (sets pitch and roll angles to zero for the given orientation).
#define ZERO_RATE_GYROS 0x87        // Starts internal self-calibration of all three rate gyro axes. By default, rate
                                    // gyros are zeroed on AHRS startup, but gyro startup calibration can be disabled
                                    // (or re-enabled) by sending a SET_START_CAL packet.
#define SELF_TEST 0x88              // Instructs the AHRS to perform a self-test of all sensor channels. A STATUS_REPORT 
                                    // packet is transmitted after the self-test is complete.
#define SET_START_CAL 0x89          // Enables or disables automatic startup calibration of rate gyro biases.
#define SET_PROCESS_COVARIANCE 0x8A // Sets the 3x3 matrix representing the covariance of the process noise used in the 
                                    // prediction step of the EKF.
#define SET_MAG_COVARIANCE 0X8B     // Sets the 3x3 matrix representing the covariance of the measurement noise used in 
                                    // the magnetometer update step of the EKF
#define SET_ACCEL_COVARIANCE 0x8C   // Sets the 3x3 matrix representing the covariance of the measurement noise used in 
                                    // the accelerometer update step of the EKF
#define SET_EKF_CONFIG 0x8D         // Sets the EKF_CONFIG register. Can be used to enable/disable the EKF, or to enable/disable
                                    // accelerometer and magnetometer updates.
#define SET_GYRO_ALIGNMENT 0x8E     // Sets the 3x3 matrix used to correct rate gyro crossaxis misalignment.
#define SET_ACCEL_ALIGNMENT 0x8F    // Sets the 3x3 matrix used to correct accelerometer cross-axis misalignment
#define SET_MAG_REF_VECTOR 0x90     // Sets the reference vector representing the expected output of the magnetometer when yaw,
                                    // pitch, and roll angles are zero.
#define AUTO_SET_MAG_REF 0x91       // Sets the current magnetometer output as the reference vector.
#define SET_MAG_CAL 0x92            // Sets the 3x3 magnetic field distortion correction matrix to compensate for soft-iron 
                                    // distortions, axis misalignment, and sensor scale inconsistencies.
#define SET_MAG_BIAS 0x93           // Sets the magnetic field measurement bias to compensate for hard iron distortions
#define SET_GYRO_SCALE 0x94         // Sets the scale factors used to convert raw ADC data to rates for angle estimation on the EKF.
#define EKF_RESET 0x95              // Sets all terms in the current state covariance matrix to zero and re-initializes angle estimates.
#define RESET_TO_FACTORY 0x96       // Resets all AHRS setting to factory default values
#define WRITE_TO_FLASH 0xA0         // Writes current AHRS configuration to on-board flash, so that the configuration persists 
                                    // when the power is cycled.
#define GET_DATA 0x01               // In Listen Mode, causes the AHRS to transmit data from all active channels in a SENSOR_DATA packet.
#define GET_ACTIVE_CHANNELS 0x02    // Reports which channels are "active" in an ACTIVE_CHANNELS_REPORT packet. Active channels are
                                    // sensor channels that are measured and transmitted in response to a GET_DATA packet, or periodically
                                    // in Broadcast Mode.
#define GET_BROADCAST_MODE 0x03     // Returns the BROADCAST_MODE_REPORT packet, which specifies whether the AHRS is in Broadcast Mode or Silent Mode.
#define GET_ACCEL_BIAS 0x04         // Return the bias values for all three accel axes in a ACCEL_BIAS_REPORT packet.
#define GET_ACCEL_REF_VECTOR 0x05   // Returns the accelerometer reference vector in an ACCEL_REF_VECTOR_REPORT packet
#define GET_GYRO_BIAS 0x06          // Returns the bias values for all three rate gyros in a GYRO_BIAS_REPORT packet.
#define GET_GYRO_SCALE 0x07         // Returns the rate gyro scale factors used internally to convert raw rate gyro data to actual rates in a
                                    // GYRO_SCALE_REPORT packet.
#define GET_START_CAL 0x08          // Reports whether the AHRS is configured to calibrate rate gyro biases automatically on startup in a
                                    // START_CAL_REPORT packet.
#define GET_EKF_CONFIG 0x09         // Returns the one byte EKF configuration register in an EKF_CONFIG_REPORT packet.
#define GET_ACCEL_COVARIANCE 0X0A   // Returns the variance of the accelerometer measurements used by the EKF update step
#define GET_MAG_COVARIANCE 0x0B     // Returns the variance of the magnetometer measurements used by the EKF update step
#define GET_PROCESS_COVARIANCE 0x0C // Returns the variance of the EKF prediction step
#define GET_STATE_COVARIANCE 0x0D   // Returns the 3x3 matrix representing the covariance of the current EKF state estimates in a
                                    // STATE_COVARIANCE_REPORT packet.
#define GET_GYRO_ALIGNMENT 0x0E     // Returns the 3x3 matrix used to correct rate gyro cross-axis misalignment
#define GET_ACCEL_ALIGNMENT 0x0F    // Returns the 3x3 matrix used to correct accelerometer cross-axis misalignment
#define GET_MAG_REF_VECTOR 0x10     // Returns the magnetometer reference vector in a MAG_REF_VECTOR_REPORT packet
#define GET_MAG_CAL Returns 0x11    // the 3x3 magnetometer correction matrix in an MAG_CAL_REPORT packet
#define GET_MAG_BIAS 0x12           // Returns the magnetometer bias correction used by the EKF. The bias is reported in a
                                    // MAG_BIAS_REPORT packet.

/*************************************************************************
//        IMU TX PACKET DEFINITIONS (IMU ->)
*************************************************************************/
#define COMMAND_COMPLETE 0xB0       // Transmitted by the AHRS upon successful completion of a command that does not require
                                    // data to be returned.
#define COMMAND_COMPLETE_SIZE 8     // 8 Bytes in COMMAND_COMPLETE Packet
#define COMMAND_FAILED 0xB1         // Transmitted by the AHRS when a command received over the UART could not be executed.
#define BAD_CHECKSUM 0xB2           // Transmitted by the AHRS when a received packet checksum does not match the sum of the
                                    // other bytes in the packet.
#define BAD_DATA_LENGTH 0xB3        // Transmitted by the AHRS when a received packet contained more or less data than
                                    // expected for a given packet type.
#define UNRECOGNIZED_PACKET 0xB4    // Transmitted if the AHRS receives an unrecognized packet.
#define BUFFER_OVERFLOW 0xB5        // Transmitted by the AHRS when the internal receive buffer overflows before a full packet is received.
#define STATUS_REPORT 0xB6          // Transmitted at the end of a self-test procedure, triggered by a SELF_TEST command.
#define STATUS_REPORT_SIZE 8        // 8 Bytes in STATUS_REPORT Packet
#define SENSOR_DATA 0xB7            // Sent in response to a GET_DATA packet, or sent automatically in Broadcast Mode.
#define GYRO_BIAS_REPORT 0xB8       // Sent in response to the GET_GYRO_BIAS command.
#define GYRO_SCALE_REPORT 0xB9      // Sent in response to a GET_GYRO_SCALE command.
#define START_CAL_REPORT 0xBA       // Sent in response to a GET_START_CAL command.
#define ACCEL_BIAS_REPORT 0xBB      // Sent in response to the GET_ACCEL_BIAS command.
#define ACCEL_REF_VECTOR_REPORT 0xBC // Sent in response to a GET_ACCEL_REF_VECTOR command, or sent after an AUTO_SET_ACCEL_REF command.
#define ACCEL_REF_VECTOR_REPORT_SIZE 13 // 13 Bytes in ACCEL_REF_VECTOR_REPORT Packet
#define ACTIVE_CHANNEL_REPORT 0xBD   // Sent in response to a GET_ACTIVE_CHANNELS command.
#define ACCEL_COVARIANCE_REPORT 0xBE // Sent in response to a GET_ACCEL_COVARIANCE command
#define MAG_COVARIANCE_REPORT 0xBF   // Sent in response to a GET_MAG_COVARIANCE command.
#define PROCESS_COVARIANCE_REPORT 0xC0 //Sent in response to a GET_PROCESS_COVARIANCE command
#define STATE_COVARIANCE_REPORT 0xC1 // Sent in response to a GET_STATE_COVARIANCE command
#define EKF_CONFIG_REPORT 0xC2       // Sent in response to a GET_EKF_CONFIG command.
#define GYRO_ALIGNMENT_REPORT 0xC3   // Sent in response to a GET_GYRO_ALIGNMENT command.
#define ACCEL_ALIGNMENT_REPORT 0xC4  // Sent in response to a GET_ACCEL_ALIGNMENT command.
#define MAG_REF_VECTOR_REPORT 0xC5   // Sent in response to a GET_MAG_REF_VECTOR command
#define MAG_CAL_REPORT 0xC6          // Sent in response to a GET_MAG_CAL command
#define MAG_BIAS_REPORT 0xC7         // Sent in response to a GET_MAG_BIAS command
#define BROADCAST_MODE_REPORT 0XC8   // Sent in response to a GET_BROADCAST_MODE command.

/*************************************************************************
Function: IMU_init()
Purpose:  Run self-test
          Set values to output
Returns:  boolean IMUinitialization = if IMU initialized successfully
**************************************************************************/
boolean IMU_init(void){
  // Variables
  boolean IMUinitialization = true;
  int sum = 0;
  
  // Open Serial Channel, 115,200 Baud Rate
  IMUSerialBegin(IMUSerialBaud);
  
  // ---------- Run Self-Test -----------------
  // clear incoming buffer
  IMUSerialFlush();
  sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(SELF_TEST); 
  sum+=SELF_TEST;
  // Number of data bytes
  IMUSerialWrite((byte)0); 
  sum+=(byte)0;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
  
  delay(200);
  
  // ---------- Set IMU Values Channels to Output-----------------
  // clear incoming buffer
  IMUSerialFlush();
  sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(SET_ACTIVE_CHANNELS); 
  sum+=SET_ACTIVE_CHANNELS;
  // Number of data bytes
  IMUSerialWrite((byte)2); 
  sum+=(byte)2;
  IMUSerialWrite(0xFC); // Activate Estimates and Rates for Yaw, Pitch, and Roll
  //IMUSerialWrite(0xE0); // Activate Estimates for Yaw, Pitch, and Roll
  sum+=0xFC;
  IMUSerialWrite((byte)0); // Don't Activate Rate Gyro data in X, Y, and Z
  sum+=0x00;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
  
  //boolean silent = IMU_silent();
  
  IMUSerialFlush();
  return IMUinitialization;
}

/*************************************************************************
Function: IMU_calibrate()
Purpose:  Calibrates IMU
            - Zero Rate Gyros
            - Zero Accelerometer Measurements
Returns:  boolean noerror = if IMU calibrated successfully
**************************************************************************/
boolean IMU_calibrate(void){
  boolean noerror = true;
  // ---------- Zero Rate Gyros -----------------
  // clear incoming buffer
  IMUSerialFlush();
  int sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(ZERO_RATE_GYROS); 
  sum+=ZERO_RATE_GYROS;
  // Number of data bytes
  IMUSerialWrite((byte)0); 
  sum+=(byte)0;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
  
  delay(200);
  
  // ---------- Zero Accelerometers -----------------
  // clear incoming buffer
  IMUSerialFlush();
  sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(AUTO_SET_ACCEL_REF); 
  sum+=AUTO_SET_ACCEL_REF;
  // Number of data bytes
  IMUSerialWrite((byte)0); 
  sum+=(byte)0;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
  
  IMUSerialFlush();
  return noerror;
}

/*************************************************************************
Function: IMU_data_request()
Purpose:  Sends data request to CHRobotics IMU
**************************************************************************/
void IMU_data_request(void){
  // clear incoming buffer
  IMUSerialFlush();
  int sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(GET_DATA); 
  sum+=GET_DATA;
  // Number of data bytes
  IMUSerialWrite((byte)0); 
  sum+=(byte)0;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
}
  
/*************************************************************************
Function: IMU_data_read()
Purpose:  Sends data request to CHRobotics IMU
**************************************************************************/
void IMU_data_read(void){
  
  // Wait for serial data
  while(IMUSerialAvailable()<20){
  }
  
  newIMUdata = true;
  
  // Read Data Packet
  byte data[12];
  int num;
  byte header;
  header = IMUSerialRead(); // byte 1 ('s')
  header = IMUSerialRead(); // byte 2 ('n')
  header = IMUSerialRead(); // byte 3 ('p')
  header = IMUSerialRead(); // byte 4 (Sensor Data)
  header = IMUSerialRead(); // byte 5 (Number of data bytes)
  header = IMUSerialRead(); // byte 6 (active channels)
  header = IMUSerialRead(); // byte 7 (active channels)
  // DATA BYTES
  data[0]=IMUSerialRead(); // Yaw data byte 1
  data[1]=IMUSerialRead(); // Yaw data byte 2
  num = (int)data[0]<<8; 
  num |= data[1];
  yaw = (num*YAW_FACTOR)+yaw_offset;
 
  data[2]=IMUSerialRead(); // Pitch data byte 1
  data[3]=IMUSerialRead(); // Pitch data byte 2
  num = (int)data[2]<<8; 
  num |= data[3];
  pitch = -1.0*(num*PITCH_FACTOR)+pitch_offset;
  //Serial.println(pitch);
  
  data[4]=IMUSerialRead(); // Roll data byte 1
  data[5]=IMUSerialRead(); // Roll data byte 2
  num = (int)data[4]<<8; 
  num |= data[5];
  roll = (num*ROLL_FACTOR)+roll_offset;

  data[6]=IMUSerialRead(); // Yaw Rate data byte 1
  data[7]=IMUSerialRead(); // Yaw Rate data byte 2
  num = (int)data[6]<<8; 
  num |= data[7];
  yawrate = (num*YAW_RATE_FACTOR)+yawrate_offset;
  
  data[8]=IMUSerialRead(); // Pitch Rate data byte 1
  data[9]=IMUSerialRead(); // Pitch Rate data byte 2
  num = (int)data[8]<<8; 
  num |= data[9];
  pitchrate = -1*(num*PITCH_RATE_FACTOR)+pitchrate_offset;
  
  data[10]=IMUSerialRead(); // Roll Rate data byte 1
  data[11]=IMUSerialRead(); // Roll Rate data byte 2
  num = (int)data[10]<<8; 
  num |= data[11];
  rollrate = (num*ROLL_RATE_FACTOR)+rollrate_offset;
  
  header = IMUSerialRead(); // Checksum
  header = IMUSerialRead(); // Checksum
  
  // clear incoming buffer
  IMUSerialFlush();
}
  
/*************************************************************************
Function: IMU_data()
Purpose:  Gets data from CHRobotics IMU
          - places result into IMU data variables
**************************************************************************/
void IMU_data(void){
  uint16_t check = 0;
  uint16_t checksum = 0;  
  byte header;
  while(IMUSerialAvailable()>20){
    if(IMUSerialRead() == 's'){
      check += 115;
      if(IMUSerialRead() =='n'){
        check += 110;
        if(IMUSerialRead() == 'p'){
          check += 112;
          packetfound = true;
        }
      }
    }
  }
  if(packetfound && IMUSerialAvailable()>17){
    // Found complete Packet Header
    // Read Data Packet    
    byte data[12];
    int num;
    header = IMUSerialRead(); // byte 4 (Sensor Data)
    check += header;
    header = IMUSerialRead(); // byte 5 (Number of data bytes)
    check += header;
    header = IMUSerialRead(); // byte 6 (active channels)
    check += header;
    header = IMUSerialRead(); // byte 7 (active channels)
    check += header;
    // DATA BYTES
    data[0]=IMUSerialRead(); // Yaw data byte 1
    check += data[0];
    data[1]=IMUSerialRead(); // Yaw data byte 2
    check += data[1];
    num = (int)data[0]<<8; 
    num |= data[1];
    yaw = (num*YAW_FACTOR)+yaw_offset;
   
    data[2]=IMUSerialRead(); // Pitch data byte 1
    check += data[2];
    data[3]=IMUSerialRead(); // Pitch data byte 2
    check += data[3];
    num = (int)data[2]<<8; 
    num |= data[3];
    pitch = -1.0*(num*PITCH_FACTOR)+pitch_offset;
    
    data[4]=IMUSerialRead(); // Roll data byte 1
    check += data[4];
    data[5]=IMUSerialRead(); // Roll data byte 2
    check += data[5];
    num = (int)data[4]<<8; 
    num |= data[5];
    roll = (num*ROLL_FACTOR)+roll_offset;
  
    data[6]=IMUSerialRead(); // Yaw Rate data byte 1
    check += data[6];
    data[7]=IMUSerialRead(); // Yaw Rate data byte 2
    check += data[7];
    num = (int)data[6]<<8; 
    num |= data[7];
    yawrate = (num*YAW_RATE_FACTOR)+yawrate_offset;
  
    data[8]=IMUSerialRead(); // Pitch Rate data byte 1
    check += data[8];
    data[9]=IMUSerialRead(); // Pitch Rate data byte 2
    check += data[9];
    num = (int)data[8]<<8; 
    num |= data[9];
    pitchrate = -1.0*(num*PITCH_RATE_FACTOR)+pitchrate_offset;
    
    data[10]=IMUSerialRead(); // Roll Rate data byte 1
    check += data[10];
    data[11]=IMUSerialRead(); // Roll Rate data byte 2
    check += data[11];
    num = (int)data[10]<<8; 
    num |= data[11];
    rollrate = (num*ROLL_RATE_FACTOR)+rollrate_offset;
    
    // Read final checksum
    checksum = (int)(IMUSerialRead()<<8); // Read final checksum
    checksum |= IMUSerialRead(); // Read final checksum
    if(check == checksum){
      newIMUdata = true;
    }
    packetfound = false;
  }
}
          
/*************************************************************************
Function: IMU_restart()
Purpose:  Restarts the CHRobotics IMU calculations
          - (if passes too closely to singularity)
Returns:  boolean noerror = if IMU restarted successfully
**************************************************************************/
boolean IMU_restart(void){
  boolean noerror = true;
  // ---------- IMU Restart (EKF Reset) -----------------
  // clear incoming buffer
  IMUSerialFlush();
  int sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(0x95); 
  sum+=0x95;
  // Number of data bytes
  IMUSerialWrite((byte)0); 
  sum+=(byte)0;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
  
  //  Read Command Complete Packet
  byte commandcomp[COMMAND_COMPLETE_SIZE];
  int i = 0;
  // Wait for serial data
  while(IMUSerialAvailable()<COMMAND_COMPLETE_SIZE);
  // Fill array with packet data
  while(IMUSerialAvailable()){
     commandcomp[i] = IMUSerialRead();
     i++;
  }
  // Check Packet Contents
  if(commandcomp[3] != COMMAND_COMPLETE){
    // Wrong packet type received
    noerror = false;
  }
  //SerPriln("IMU Reset");
  return noerror;
}

/*************************************************************************
Function: IMU_write()
Purpose:  Writes previous commands to flash
Returns:  boolean noerror = if IMU wrote to flash successfully
**************************************************************************/
boolean IMU_write(void){
  boolean noerror = true;
  // clear incoming buffer
  IMUSerialFlush();
  int sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(WRITE_TO_FLASH); 
  sum+=WRITE_TO_FLASH;
  // Number of data bytes
  IMUSerialWrite((byte)0); 
  sum+=(byte)0;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
  
  // Read Command Complete Packet
  byte commandcomp[COMMAND_COMPLETE_SIZE];
  int i = 0;
  // Wait for serial data
  while(IMUSerialAvailable()<COMMAND_COMPLETE_SIZE);
  // Fill array with packet data
  while(IMUSerialAvailable()){
     commandcomp[i] = IMUSerialRead();
     i++;
  }
  // Check Packet Contents
  if(commandcomp[3] != COMMAND_COMPLETE){
    // Wrong packet type received
    noerror = false;
  }
  return noerror;
}

/*************************************************************************
Function: IMU_silent()
Purpose:  Set IMU to Silent Mode
Returns:  boolean noerror = if IMU was set to silent mode successfully
**************************************************************************/
boolean IMU_silent(void){ 
  boolean noerror = true;
  // clear incoming buffer
  IMUSerialFlush();
  int sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(SET_SILENT_MODE); 
  sum+=SET_SILENT_MODE;
  // Number of data bytes
  IMUSerialWrite((byte)0); 
  sum+=(byte)0;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
  
  // Read Command Complete Packet
  int i = 0;
  byte commandcomp[COMMAND_COMPLETE_SIZE];
  // Wait for serial data
  while(IMUSerialAvailable()<COMMAND_COMPLETE_SIZE);
  // Fill array with packet data
  while(IMUSerialAvailable()){
     commandcomp[i] = IMUSerialRead();
     i++;
  }
  // Check Packet Contents
  if(commandcomp[3] != COMMAND_COMPLETE){
    // Wrong packet type received
    noerror = false;
  }  
  return noerror;
}

/*************************************************************************
Function: IMU_broadcast()
Purpose:  Set IMU to Broadcast Mode
Returns:  boolean noerror = if IMU was set to broadcast mode successfully
**************************************************************************/
boolean IMU_broadcast(void){ 
  boolean noerror = true;
  // clear incoming buffer
  IMUSerialFlush();
  int sum = 0;
  // Packet header
  IMUSerialWrite('s'); 
  sum+='s';
  IMUSerialWrite('n');
  sum+='n';
  IMUSerialWrite('p');
  sum+='p';
  // Packet type
  IMUSerialWrite(0x82); 
  sum+=0x82;
  // Number of data bytes
  IMUSerialWrite(0x01); 
  sum+=0x01;
  // Data byte - Set Broadcast to 300Hz
  IMUSerialWrite(0xFF); 
  sum+=0xFF;
  // Checksum = Sum of all preceding bytes in packet
  IMUSerialWrite((byte)((sum >> 8) & 0x0FF));
  IMUSerialWrite((byte)((sum) & 0x0FF));
  
  // Read Command Complete Packet
  int i = 0;
  byte commandcomp[COMMAND_COMPLETE_SIZE];
  // Wait for serial data
  while(IMUSerialAvailable()<COMMAND_COMPLETE_SIZE);
  // Fill array with packet data
  while(IMUSerialAvailable()){
     commandcomp[i] = IMUSerialRead();
     i++;
  }
  // Check Packet Contents
  if(commandcomp[3] != COMMAND_COMPLETE){
    // Wrong packet type received
    noerror = false;
  }  
  return noerror;
}

/*************************************************************************
Function: IMU_GyroFilter()
Purpose:  IMU Low Pass Filter on Gyro Data
**************************************************************************/
void IMU_RateFilter(void) {
  // Filter Gyro Values
  yawrate = (1.0-ratefilter_alpha)*yawrate_old + yawrate*ratefilter_alpha;
  pitchrate = (1.0-ratefilter_alpha)*pitchrate_old + pitchrate*ratefilter_alpha;
  rollrate = (1.0-ratefilter_alpha)*rollrate_old + rollrate*ratefilter_alpha;
  // Set current values to previous values
  yawrate_old = yawrate;
  pitchrate_old = pitchrate;
  rollrate_old = rollrate;
}
