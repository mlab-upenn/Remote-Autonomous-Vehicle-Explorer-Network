/********************************************************************
 R.A.V.E.N. Quadrotor - November 2010
 www.AirHacks.com
 Copyright (c) 2010.  All rights reserved.
 
 Version: 1.0
 February 12, 2011
 
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
void initThermal();
void readThermal();

/*************************************************************************
Function Varibles
**************************************************************************/



/*************************************************************************
Function: initThermal()
Purpose:  Initializes the Devantech TPA81 Thermopile Array
**************************************************************************/
void initThermal(void){
    // Reset ThermalData array
    
}

/*************************************************************************
Function: readThermal()
Purpose:  Reads data from the Devantech TPA81 Thermopile Array
**************************************************************************/
void readThermal(void){
    char thermal[1];
    // Ask for Ambient Temperature Data
    thermal[0]=0x01;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Ambient Temperature Data
    cam_i2c.read(thermal_address,thermal,1);
    ThermalData.ambient = thermal[0];
    // Ask for Pixel1 Temperature Data
    thermal[0]=0x02;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Pixel1 Temperature Data
    cam_i2c.read(thermal_address, thermal,1);
    ThermalData.pixel1 = thermal[0];
    // Ask for Pixel2 Temperature Data
    thermal[0]=0x03;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Pixel2 Temperature Data
    cam_i2c.read(thermal_address,thermal,1);
    ThermalData.pixel2 = thermal[0];
    // Ask for Pixel3 Temperature Data
    thermal[0]=0x04;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Pixel3 Temperature Data
    cam_i2c.read(thermal_address, thermal,1);
    ThermalData.pixel3 = thermal[0];
    // Ask for Pixel4 Temperature Data
    thermal[0]=0x05;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Pixel4 Temperature Data
    cam_i2c.read(thermal_address, thermal,1);
    ThermalData.pixel4 = thermal[0];
    // Ask for Pixel5 Temperature Data
    thermal[0]=0x06;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Pixel5 Temperature Data
    cam_i2c.read(thermal_address,thermal,1);
    ThermalData.pixel5 = thermal[0];
    // Ask for Pixel6 Temperature Data
    thermal[0]=0x07;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Pixel6 Temperature Data
    cam_i2c.read(thermal_address, thermal,1);
    ThermalData.pixel6 = thermal[0];
    // Ask for Pixel7 Temperature Data
    thermal[0]=0x08;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Pixel7 Temperature Data
    cam_i2c.read(thermal_address, thermal,1);
    ThermalData.pixel7 = thermal[0];
    // Ask for Pixel8 Temperature Data
    thermal[0]=0x09;
    cam_i2c.write(thermal_address, thermal,1);
    // Receive Pixel8 Temperature Data
    cam_i2c.read(thermal_address, thermal,1);
    ThermalData.pixel8 = thermal[0];
}