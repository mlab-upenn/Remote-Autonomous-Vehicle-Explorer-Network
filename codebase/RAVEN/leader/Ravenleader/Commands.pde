/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.1
 January 23, 2010
 
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

/*************************************************************************
Function: Commands()
Purpose:  Completes the desired command as sent from the Base Station
          to the Quadrotor
**************************************************************************/
void BaseCommands(byte inputcommand){
  switch(inputcommand){
      case 0xD4:   // Follower Present
        followerpresent = true;
        break;
      default:  // Do nothing
        break;
     }
}
