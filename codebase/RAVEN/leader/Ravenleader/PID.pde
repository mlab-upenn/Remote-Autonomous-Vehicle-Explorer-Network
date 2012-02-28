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
// TODO
//  - Edit Constraints
//  - Adjust Maximum Control Output



/*************************************************************************
Function: PID()
Purpose:  Runs the Proportional, Integral, and Derivative (PID) Loop
          for quadrotor flight stabilization
**************************************************************************/
void PID(void){
  // YAW CONTROL
  yaw_P = command_rx_yaw - yaw;
  // Normalzie to -180, 180 degrees
  if(yaw_P > 180.0){
    yaw_P -= 360.0;
  }
  else if(yaw_P< -180.0){
    yaw_P += 360.0;
  }
  yaw_P = constrain(yaw_P,-40,40);
  yaw_I += yaw_P*Dt;
  yaw_I = constrain(yaw_I,-20,20);
  
  yaw_D =yawrate;

  
  //if(yaw_D <0.2 && yaw_D >-0.2){
  //  yaw_D = 0.0;  // Throw out small values (noise)
  //}
  stable_yaw = KP_Yaw*yaw_P + KI_Yaw*yaw_I - KD_Yaw*yaw_D;
  control_yaw = PID_Yaw_Control_Gain*stable_yaw;
  control_yaw = constrain(control_yaw,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);
   
  // PITCH CONTROL
  pitch_P = command_rx_pitch - pitch;
  if(pitch_P <0.2 && pitch_P >-0.2){
    pitch_P = 0.0;  // Throw out small values (noise)
  }
  pitch_P = constrain(pitch_P,-30,30);
  pitch_I += pitch_P*Dt;
  pitch_I = constrain(pitch_I,-20,20);
  
  pitch_D = pitchrate;
  
  //if(pitch_D <0.2 && pitch_D >-0.2){
  //  pitch_D = 0.0;  // Throw out small values (noise)
  //}
  
  stable_pitch = KP_Pitch*pitch_P + KI_Pitch*pitch_I - KD_Pitch*pitch_D;
  control_pitch = PID_Pitch_Control_Gain*stable_pitch;
  control_pitch = constrain(control_pitch,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);
  
  // ROLL CONTROL
  roll_P = command_rx_roll - roll;
  if(roll_P <0.2 && roll_P >-0.2){
    roll_P = 0.0;  // Throw out small values (noise)
  }
  roll_P = constrain(roll_P,-25,25);
  roll_I += roll_P*Dt;
  roll_I = constrain(roll_I,-20,20);
  
  roll_D = rollrate;
  
  //if(roll_D <0.2 && roll_D >-0.2){
  //  roll_D = 0.0;  // Throw out small values (noise)
  //}
  stable_roll = KP_Roll*roll_P + KI_Roll*roll_I - KD_Roll*roll_D;
  control_roll = PID_Roll_Control_Gain*stable_roll;
  control_roll = constrain(control_roll,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);
  
  newIMUdata = false;
}
