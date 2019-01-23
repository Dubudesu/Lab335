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


  if(Serial2.available()){
    
    s += (char)Serial2.read();
    c++;
  }
  if(c == 3){
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
    Serial.print("Button ");
    Serial.print((char)s.charAt(1));
    Serial.print(v);
    c = 0;
    s = "";
    v = "";
  }
  
}
