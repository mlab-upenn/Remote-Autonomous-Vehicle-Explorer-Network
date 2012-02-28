/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.3
 February 1, 2011
 
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
// - Do something with aux input 2?
// After getting command_rx_yaw put in constraint to limit it to +/- X from current Yaw


/*************************************************************************
Function: get_RC()
Purpose:  Read RC Input Values
          Adjust values appropriately
**************************************************************************/
void get_RC(void){
  float aux_float;
  // Check if a new RC Input has been received
  if (APM_RC.GetState() == 1){
    // Read RC Commands and apply filter
    // RC Controller Stick position is desired yaw, pitch, and roll angles
    ch_roll = channel_filter(APM_RC.InputCh(roll_channel) * ch_roll_slope + ch_roll_offset, ch_roll);
    ch_pitch = channel_filter(APM_RC.InputCh(pitch_channel) * ch_pitch_slope + ch_pitch_offset, ch_pitch);
    ch_throttle = channel_filter(APM_RC.InputCh(throttle_channel) * ch_throttle_slope, ch_throttle);
    ch_yaw = channel_filter(APM_RC.InputCh(yaw_channel) * ch_yaw_slope + ch_yaw_offset, ch_yaw);
    ch_aux = APM_RC.InputCh(4) * ch_aux_slope + ch_aux_offset;
    ch_aux2 = APM_RC.InputCh(5) * ch_aux2_slope + ch_aux2_offset;
    
    command_rx_roll = (ch_roll-ROLL_MID) / roll_command_divider;
    command_rx_pitch = (ch_pitch-PITCH_MID) / pitch_command_divider;
    
    // Ignore small flucations in Yaw
    if(abs(ch_yaw-YAW_MID)>15){
      command_rx_yaw += (ch_yaw-YAW_MID) / yaw_command_divider;
      // Normalzie to -180, 180 degrees
      if(command_rx_yaw > 180.0){
        command_rx_yaw -= 360.0;
      }
      else if(command_rx_yaw< -180.0){
        command_rx_yaw += 360.0;
      }
    }

    // Check if RC transmitter is ON
    if(ch_aux > 1200){
      // If greater than 1200, transmitter is off and receiver is outputting 1500
      
      // Turn LED to WHITE
      statLEDwhiteOn();
      
      // Disarm motors (safety cut)
      motorArmed = 0;
    }
    else if(ch_aux <1200 && motorArmed == 0){
      statLEDgreenOn();
    }
    
    //if(ch_aux2 > 1800)
  }
}

/*************************************************************************
Function: channel_filter()
Purpose:  Maximun slope filter for radio inputs
          (limit max differences between readings)
Inputs:   ch = current channel input
          old_ch = old channel input
Returns:  Corrected channel input within set difference amount (currently 60)
**************************************************************************/
int channel_filter(int ch, int old_ch){
  // Difference between readings
  int diff;
  
  // Check if Old Channel is initialized yet
  if (old_ch==0)       
    return(ch);
  diff = ch - old_ch;
  if (diff < 0)
  {
    if (diff <- 60)
      return(old_ch - 60);
  }
  else
  {
    if (diff > 60)    
      return(old_ch + 60);
  }
  
  // Filtering
  return((ch + old_ch) >> 1);                 
}

/*************************************************************************
Function: armDisarm_RC()
Purpose:  Arm or Disarm Motors
**************************************************************************/
void armDisarm_RC(void){
  // Arm Motors - Throttle down and full yaw right
  if (ch_throttle < (MIN_THROTTLE + 100)) {    
    // Set current Yaw as commanded Yaw
    control_yaw = 0;
    command_rx_yaw = yaw;
    if (ch_yaw > 1800 && ch_aux < 1200) {
      if (Arming_counter > ARM_DELAY){
        // Check if Throttle is above a threshold
        if(ch_throttle > 800) {
          motorArmed = 1;
          // Set LED to RED
          statLEDredOn();
          // Start the motors spinning
          minThrottle = MOTOR_START;  
        }
      }
      else{
        Arming_counter++;
      }
    }
    else{
      Arming_counter=0;
    }
    
    // To Disarm motor output : Throttle down and full yaw left for more than 2 seconds
    if (ch_yaw < 1200) {
      if (Disarming_counter > DISARM_DELAY){
        motorArmed = 0;
        // Set LED to GREEN
        statLEDgreenOn();
        minThrottle = MOTOR_STOP;
      }
      else
        Disarming_counter++;
    }
    else
      Disarming_counter=0;
  }
  else{
    Arming_counter=0;
    Disarming_counter=0;
  }
}

/*************************************************************************
Function: send_RC()
Purpose:  Send RC Output Values
          Adjust values appropriately for Quadrotor X Flight Mode
**************************************************************************/
void send_RC(void){
  if (motorArmed == 1){
    // Throttle Multiplier
    ch_throttle_adj = (constrain(ch_throttle-1000,0,1000)) * throttle_gain + minThrottle;
    // Front Left Motor
    FrontLeftMotor = constrain(ch_throttle_adj + control_roll - control_pitch - control_yaw, minThrottle, MAX_MOTOR);
    
    // Front Right Motor
    FrontRightMotor = constrain(ch_throttle_adj - control_roll - control_pitch + control_yaw, minThrottle, MAX_MOTOR);
    
    // Rear Left Motor
    RearLeftMotor = constrain(ch_throttle_adj + control_roll + control_pitch + control_yaw, minThrottle, MAX_MOTOR);
    
    // Rear Right Motor
    RearRightMotor = constrain(ch_throttle_adj - control_roll + control_pitch - control_yaw, minThrottle, MAX_MOTOR);
  }
  if (motorArmed == 0){
    // Stop all motors if not armed
    FrontLeftMotor = MOTOR_STOP;
    FrontRightMotor = MOTOR_STOP;
    RearLeftMotor = MOTOR_STOP;
    RearRightMotor = MOTOR_STOP;
    // Reset Integral (I) terms in PID controls
    roll_I = 0;
    pitch_I = 0;
    yaw_I = 0;
    // Set desired yaw to actual yaw
    command_rx_yaw = yaw;
  }
  // Send commands to motors
  // Front Left Motor
  APM_RC.OutputCh(FLMotorCh, FrontLeftMotor);
  // Front Right Motor
  APM_RC.OutputCh(FRMotorCh, FrontRightMotor);
  // Rear Left Motor
  APM_RC.OutputCh(RLMotorCh, RearLeftMotor);
  // Rear Right Motor
  APM_RC.OutputCh(RRMotorCh, RearRightMotor);
}
