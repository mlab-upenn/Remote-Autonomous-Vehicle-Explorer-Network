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

/*************************************************************************
Function Headers
**************************************************************************/
void initIRcam(void);
void readIRcam(void);
void measureIRcam(void);
void servoIRcam();
void camWrite2(char a, char b);

/*************************************************************************
Function Varibles
**************************************************************************/
// Temporary position holders
int point1x = 0;
int point1y = 0;
int point1s = 0;
int point2x = 0;
int point2y = 0;
int point2s = 0;
int point3x = 0;
int point3y = 0;
int point3s = 0;
int point4x = 0;
int point4y = 0;
int point4s = 0;

char buf[16];           // I2C Data buffer
char ir_s;              // IR Sensor Shift Variable
char cmd[2];            // IR Sensor Command Buffer

/*************************************************************************
Function: initIRcam()
Purpose:  Initializes the Pixart InfraRed Camera
          and sets the servo to center (1500us)
**************************************************************************/
void initIRcam(void){
  // Initialize IR Camera Sensor
    camWrite2(0x30, 0x01);
    camWrite2(0x30, 0x08);    
    camWrite2(0x06, 0x90);
    camWrite2(0x08, 0xC0);
    camWrite2(0x1A, 0x40);
    camWrite2(0x33, 0x33);
    
    // Initialize IR Camera Servo
    IRservo.period_ms(20);
    IRservo.pulsewidth_us(1450);
}

/*************************************************************************
Function: readIRcam()
Purpose:  Obtains data from the Pixart InfraRed Camera
          - Scales Y points (forms square image)
          - Rotates points (compensates for Roll)
          - Filters points (reduces effects of 
          -
          
          // NEED TO THROW OUT LARGE JUMPS AND TAKE CARE OF SWITCHING VALUES
**************************************************************************/
void readIRcam(void){
  // request data
  cmd[0] = 0x36;
  cam_i2c.write(cam_address, cmd, 1);
  // receive data
  cam_i2c.read(cam_address, buf, 16);
  
  // IR Point 1
  point1x = buf[1];
  point1y = buf[2];
  ir_s   = buf[3];
  point1x += (ir_s & 0x30) <<4;
  point1x = 1024-point1x;
  point1y += (ir_s & 0xC0) <<2;
  point1y = 768-point1y;
  // Point Validity
  if(point1y<0){
    // Point is not present
    IRdata.valid[0] = false;
  }else{
    // Point is present
    IRdata.valid[0] = true;
  }
  // Scale Y value to 1024 resolution (square)
  point1y*=4/3;
  // Rotations
  point1x = point1x*cos(-1*MyAttitude.Roll*(PI/180))-point1y*sin(-1*MyAttitude.Roll*(PI/180));
  point1y = point1x*sin(-1*MyAttitude.Roll*(PI/180))+point1y*cos(-1*MyAttitude.Roll*(PI/180));
  // Point Size
  point1s = (ir_s & 0x0F);
  // Smoothing Filter
  IRdata.p1x = IRdata.p1x + cam_alpha*(point1x-IRdata.p1x);
  IRdata.p1y = IRdata.p1y + cam_alpha*(point1y-IRdata.p1y);
  IRdata.p1s = IRdata.p1s + cam_alpha*(point1s-IRdata.p1s);
  
  // IR Point 2
  point2x = buf[4];
  point2y = buf[5];
  ir_s   = buf[6];
  point2x += (ir_s & 0x30) <<4;
  point2x = 1024-point2x;
  point2y += (ir_s & 0xC0) <<2;
  point2y = 768-point2y;
  // Point Validity
  if(point2y<0){
    // Point is not present
    IRdata.valid[1] = false;
  }else{
    // Point is present
    IRdata.valid[1] = true;
  }
  // Scale Y value to 1024 resolution (square)
  point2y*=4/3;
  // Rotations
  point2x = point2x*cos(-1*MyAttitude.Roll*(PI/180))-point2y*sin(-1*MyAttitude.Roll*(PI/180));
  point2y = point2x*sin(-1*MyAttitude.Roll*(PI/180))+point2y*cos(-1*MyAttitude.Roll*(PI/180));
  // Point Size
  point2s = (ir_s & 0x0F);
  // Smoothing Filter
  IRdata.p2x = IRdata.p2x + cam_alpha*(point2x-IRdata.p2x);
  IRdata.p2y = IRdata.p2y + cam_alpha*(point2y-IRdata.p2y);
  IRdata.p2s = IRdata.p2s + cam_alpha*(point2s-IRdata.p2s);
  
  // IR Point 3
  point3x = buf[7];
  point3y = buf[8];
  ir_s   = buf[9];
  point3x += (ir_s & 0x30) <<4;
  point3x = 1024-point3x;
  point3y += (ir_s & 0xC0) <<2;
  point3y = 768-point3y;
  // Point Validity
  if(point3y<0){
    // Point is not present
    IRdata.valid[2] = false;
  }else{
    // Point is present
    IRdata.valid[2] = true;
  }
  // Scale Y value to 1024 resolution (square)
  point3y*=4/3;
  // Rotations
  point3x = point3x*cos(-1*MyAttitude.Roll*(PI/180))-point3y*sin(-1*MyAttitude.Roll*(PI/180));
  point3y = point3x*sin(-1*MyAttitude.Roll*(PI/180))+point3y*cos(-1*MyAttitude.Roll*(PI/180));
  // Point Size
  point3s = (ir_s & 0x0F);
  
  // Smoothing Filter
  IRdata.p3x = IRdata.p3x + cam_alpha*(point3x-IRdata.p3x);
  IRdata.p3y = IRdata.p3y + cam_alpha*(point3y-IRdata.p3y);
  IRdata.p3s = IRdata.p3s + cam_alpha*(point3s-IRdata.p3s);
  
  // IR Point 4
  point4x = buf[10];
  point4y = buf[11];
  ir_s   = buf[12];
  point4x += (ir_s & 0x30) <<4;
  point4x = 1024-point4x;
  point4y += (ir_s & 0xC0) <<2;
  point4y = 768-point4y;
  // Point Validity
  if(point4y<0){
    // Point is not present
    IRdata.valid[3] = false;
  }else{
    // Point is present
    IRdata.valid[3] = true;
  }
  // Scale Y value to 1024 resolution (square)
  point4y*=4/3;
  // Rotations
  point4x = point4x*cos(-1*MyAttitude.Roll*(PI/180))-point4y*sin(-1*MyAttitude.Roll*(PI/180));
  point4y = point4x*sin(-1*MyAttitude.Roll*(PI/180))+point4y*cos(-1*MyAttitude.Roll*(PI/180));
  // Point Size
  point4s = (ir_s & 0x0F); 
  // Smoothing Filter
  IRdata.p4x = IRdata.p4x + cam_alpha*(point4x-IRdata.p4x);
  IRdata.p4y = IRdata.p4y + cam_alpha*(point4y-IRdata.p4y);
  IRdata.p4s = IRdata.p4s + cam_alpha*(point4s-IRdata.p4s);
  
  // Centroids
  if(IRdata.valid[0] && IRdata.valid[1]){
    IRdata.centroidY = ((float)IRdata.p1x + (float)IRdata.p2x)/2.0;
    IRdata.centroidZ = ((float)IRdata.p1y + (float)IRdata.p2y)/2.0;
  }
}

/*************************************************************************
Function: measureIRcam()
Purpose:  Calculates the Distance from the IR beacons 
          -
          // NEED TO THROW OUT LARGE JUMPS AND TAKE CARE OF SWITCHING VALUES
**************************************************************************/
void measureIRcam(void){
  float absangles = LeaderData.Yaw-MyAttitude.Yaw-leaderyawoffset;
  if(absangles<0){
    absangles*=-1;
  }
  pc.printf("angle offset = %f \r\n",absangles);
  // Check if Points 1 and 2 are valid
  if(IRdata.valid[0] && IRdata.valid[1]){
    
    float xdif = ((float)IRdata.p1x-(float)IRdata.p2x)/100.0;
    xdif*=xdif;
    float ydif = ((float)IRdata.p1y-(float)IRdata.p2y)/100.0;
    ydif*=ydif;
    IRdata.dist = (float)sqrt(xdif+ydif); 
    
    
    // Corrected Distance between IR points (in pixels) (given Yaw offsets)
    float correcteddist = IRdata.dist/cos((PI/180)*(absangles));
    // Measured Distance from Quadrotor to IR Points
    float denom = tan((PI/180)*21.0*correcteddist/10.24);
    if(denom<0){
        denom*=-1;
    }
    float num = 0.5* beacon_width;
    float measuredDist = num/denom;
    
    pc.printf("Dist = %f \r\n",measuredDist);
    
    // Displacement Calculations
    
    // X-Displacement
    // Angle IR camera makes with horizontal axis
    float theta = MyAttitude.Pitch-IRservoAngle;
    // Angle IR beacon makes with horizontal axis
    float alpha = (IRdata.centroidZ-512)/512*15;
    // Total Angle
    float beta = theta + alpha;
    // Intermediate triagle hypotenuse
    float x = measuredDist/cos((PI/180)*beta);
    TrackingData.dist_x = x*cos((PI/180)*beta);
    //pc.printf("Distx = %f \r\n",x);
    
    // Y-Displacement
    TrackingData.dist_y = measuredDist*tan((PI/180)*21*(IRdata.centroidY-512)/512);
    
    // Z-Displacement
    TrackingData.dist_z = TrackingData.dist_x*tan((PI/180)*beta);    
  }
  else IRdata.dist = 0;
}

/*************************************************************************
Function: servoIRcam()
Purpose:  Tracks the IR beacons to help in times when pitch/leader altitude
          affect view
**************************************************************************/
void servoIRcam(void)
{
  if(IRdata.valid[0] && IRdata.valid[1]){
    // Determine corrected angle
      float diff = abs(IRdata.centroidZ-512);
      if(diff>50){
          float offset = 15*(IRdata.centroidZ-512)/512;
          IRservoAngle-=offset;
          if(IRservoAngle > IRservoMax){
            IRservoAngle = IRservoMax;
          }else if(IRservoAngle < IRservoMin){
            IRservoAngle = IRservoMin;
          }
          // Output to servo
          IRservo.pulsewidth_us((int)(IRservoAngle+50)*10 + 950);
      }
  }
  else{
    // Increment counter since IR not found
    noIRcounter++;
    if(noIRcounter>60){ // No IR for 3 seconds if IR Servo is running at 20Hz
        noIRcounter = 0;    // Reset Counter
        IRservoAngle = 0;   // Reset IR sero angle
        IRservo.pulsewidth_us((int)(IRservoAngle+50)*10 + 950); // Update servo;
    }
  }
}

/*************************************************************************
Function: camWrite2
Purpose:  Writes two bytes on the I2C bus to the Pixart IR Camera Sensor
Inputs: byte data1, byte data2 = data bytes to write to I2C bus
**************************************************************************/
void camWrite2(char data1, char data2){
    cmd[0] = data1;
    cmd[1] = data2;
    cam_i2c.write(cam_address, cmd, 2);
    wait(0.1); // delay 100ms    
}