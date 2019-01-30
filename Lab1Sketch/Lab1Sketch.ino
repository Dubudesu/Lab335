#include <Wire.h>

void setup() {

  Serial.begin(38400);
  // Usually 9600 for BT mode, although it is sometimes 38400:
  Serial2.begin(9600);
  //38400 for command mode:
  //BTSerial.begin(38400);
}

String s = "";
String v = "";
byte c = 0;

void loop() {

  //check if a byte is ready
  if(Serial2.available()){
    
    //Store character in a string and increment a counter
    s += (char)Serial2.read();
    c++;
  }
  //once three characters have been read, a full signal has been sent and it can be decoded.
  if(c == 3){

    //check just the last character of the code, to see if a button was pressed or released
    switch(s.charAt(2)){
      case '!' :
        v = " was pressed!\n";
      break;
      case '$' :
        v = " was released!\n";
      break;
      default:
        v = " ERROR\n";
      break;
    }
    //build and print which button was press/released to the console.
    Serial.print("Button ");
    Serial.print((char)s.charAt(1));
    Serial.print(v);
    //clear all values to read in the next command
    c = 0;
    s = "";
    v = "";
  }
  
}