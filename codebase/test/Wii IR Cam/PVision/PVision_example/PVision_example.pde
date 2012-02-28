// Example of using the PVision library for interaction with the Pixart sensor on a WiiMote
// This work was derived from Kako's excellent Japanese website
// http://www.kako.com/neta/2007-001/2007-001.html

// Steve Hobley 2009 - www.stephenhobley.com

#include <Wire.h>
#include <PVision.h>

PVision ircam;
byte result;

void setup()
{
  Serial.begin(115200);
  ircam.init();
  pinMode(2, OUTPUT);
}

void loop()
{
  digitalWrite(2, HIGH);
  digitalWrite(2, LOW);  
  result = ircam.read();
  digitalWrite(2, HIGH);
  
  /*
  if (result & BLOB1)
  {
    Serial.print("BLOB1 detected. X:");
    Serial.print(ircam.Blob1.X);
    Serial.print(" Y:");
    Serial.print(ircam.Blob1.Y);
    Serial.print(" Size:");
    Serial.println(ircam.Blob1.Size);
  }
  
  if (result & BLOB2)
  {
    Serial.print("BLOB2 detected. X:");
    Serial.print(ircam.Blob2.X);
    Serial.print(" Y:");
    Serial.print(ircam.Blob2.Y);
    Serial.print(" Size:");
    Serial.println(ircam.Blob2.Size);
  }
  if (result & BLOB3)
  {
    Serial.print("BLOB3 detected. X:");
    Serial.print(ircam.Blob3.X);
    Serial.print(" Y:");
    Serial.print(ircam.Blob3.Y);
    Serial.print(" Size:");
    Serial.println(ircam.Blob3.Size);
  }
  if (result & BLOB4)
  {
    Serial.print("BLOB4 detected. X:");
    Serial.print(ircam.Blob4.X);
    Serial.print(" Y:");
    Serial.print(ircam.Blob4.Y);
    Serial.print(" Size:");
    Serial.println(ircam.Blob4.Size);
  }
  */
  
  if (result & BLOB1){
    if (ircam.Blob1.X < 1000)
        Serial.print(" ");
      if (ircam.Blob1.X < 100)  
        Serial.print(" ");
      if (ircam.Blob1.X < 10)  
        Serial.print(" ");
      Serial.print(ircam.Blob1.X);
      Serial.print(",");
      if (ircam.Blob1.Y < 1000)
        Serial.print(" ");
      if (ircam.Blob1.Y < 100)  
        Serial.print(" ");
      if (ircam.Blob1.Y < 10)  
        Serial.print(" ");
      Serial.print(ircam.Blob1.Y);
  }
  else{
    Serial.print("   0,   0");
  }
  Serial.print(",");
  
  if (result & BLOB2){
    if (ircam.Blob2.X < 1000)
        Serial.print(" ");
      if (ircam.Blob2.X < 100)  
        Serial.print(" ");
      if (ircam.Blob2.X < 10)  
        Serial.print(" ");
      Serial.print(ircam.Blob2.X);
      Serial.print(",");
      if (ircam.Blob2.Y < 1000)
        Serial.print(" ");
      if (ircam.Blob2.Y < 100)  
        Serial.print(" ");
      if (ircam.Blob2.Y < 10)  
        Serial.print(" ");
      Serial.print(ircam.Blob2.Y);
  }
  else{
    Serial.print("   0,   0");
  }
  Serial.print(",");
  
  if (result & BLOB3){
    if (ircam.Blob3.X < 1000)
        Serial.print(" ");
      if (ircam.Blob3.X < 100)  
        Serial.print(" ");
      if (ircam.Blob3.X < 10)  
        Serial.print(" ");
      Serial.print(ircam.Blob3.X);
      Serial.print(",");
      if (ircam.Blob3.Y < 1000)
        Serial.print(" ");
      if (ircam.Blob3.Y < 100)  
        Serial.print(" ");
      if (ircam.Blob3.Y < 10)  
        Serial.print(" ");
      Serial.print(ircam.Blob3.Y);
  }
  else{
    Serial.print("   0,   0");
  }
  Serial.print(",");
  
  if (result & BLOB4){
    if (ircam.Blob4.X < 1000)
        Serial.print(" ");
      if (ircam.Blob4.X < 100)  
        Serial.print(" ");
      if (ircam.Blob4.X < 10)  
        Serial.print(" ");
      Serial.print(ircam.Blob4.X);
      Serial.print(",");
      if (ircam.Blob4.Y < 1000)
        Serial.print(" ");
      if (ircam.Blob4.Y < 100)  
        Serial.print(" ");
      if (ircam.Blob4.Y < 10)  
        Serial.print(" ");
      Serial.print(ircam.Blob4.Y);
  }
  else{
    Serial.print("   0,   0");
  }
  Serial.println("");
  
  
  
  
  /*
  if (result & BLOB2){
    Serial.print(ircam.Blob2.X);
    Serial.print(",");
    Serial.print(ircam.Blob2.Y);
  }
  else{
    Serial.print("0,0");
  }
  Serial.print(",");
  
  if (result & BLOB3){
    Serial.print(ircam.Blob3.X);
    Serial.print(",");
    Serial.print(ircam.Blob3.Y);
  }
  else{
    Serial.print("0,0");
  }
  Serial.print(",");
  
  if (result & BLOB4){
    Serial.print(ircam.Blob4.X);
    Serial.print(",");
    Serial.print(ircam.Blob4.Y);
  }
  else{
    Serial.print("0,0");
  }
  Serial.println();
  */

  // Short delay...
  delay(100);
  

}
