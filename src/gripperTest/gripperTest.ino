
#include <Gripper.h>


Gripper *datGripper;

void setup() {
  // put your setup code here, to run once:
  datGripper = new Gripper(10);
  Serial.begin(115200);
  //datGripper->close();
  Serial.println("Starting");
}

void loop() {
  // put your main code here, to run repeatedly:

   if (Serial.available() > 2) {
    int incomingByte = Serial.read();

    if(incomingByte == 'o'){
      Serial.println("open!");
      datGripper->open();
    }
    else if(incomingByte == 'c'){
      Serial.println("close!");
      datGripper->close();
    }
  }

}
