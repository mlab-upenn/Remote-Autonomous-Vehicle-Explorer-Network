


// Read floating point values from the serial port
//float readFloatSerial() {
//  byte index = 0;
//  byte timeout = 0;
//  char data[128] = "";
//
//  do {
//    if (SerAva() == 0) {
//      delay(10);
//      timeout++;
//    }
//    else {
//      data[index] = SerRea();
//      timeout = 0;
//      index++;
//    }
//  }  
//  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
//  return atof(data);
//}
