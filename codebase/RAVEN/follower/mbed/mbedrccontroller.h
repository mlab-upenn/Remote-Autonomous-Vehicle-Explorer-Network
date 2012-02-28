/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.0
 March 28, 2011
 
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
Function Headers
**************************************************************************/
void initRC(void);
void sendRC(void);

/*************************************************************************
Function: initRC()
Purpose:  Initializes the PWM RC Controller Outputs
**************************************************************************/
void initRC(void){
    RCch1.period_ms(20);
    RCch2.period_ms(20);
    RCch3.period_ms(20);
    RCch4.period_ms(20);
    RCch5.period_ms(20);
    
    RCoutput.ch1 = 1000;
    RCoutput.ch2 = 1000;
    RCoutput.ch3 = 1000;
    RCoutput.ch4 = 1000;
    RCoutput.ch5 = 1000;
    
    RCch1.pulsewidth_us(RCoutput.ch1);
    RCch2.pulsewidth_us(RCoutput.ch2);
    RCch3.pulsewidth_us(RCoutput.ch3);
    RCch4.pulsewidth_us(RCoutput.ch4);
    RCch5.pulsewidth_us(RCoutput.ch5);
}

/*************************************************************************
Function: sendRC()
Purpose:  Updates the PWM RC Controller Outputs
**************************************************************************/
void sendRC(void){
    RCch1.pulsewidth_us(RCoutput.ch1);
    RCch2.pulsewidth_us(RCoutput.ch2);
    RCch3.pulsewidth_us(RCoutput.ch3);
    RCch4.pulsewidth_us(RCoutput.ch4);
    RCch5.pulsewidth_us(RCoutput.ch5);
}