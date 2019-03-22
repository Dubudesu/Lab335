
#include <LineFollower.h>

unsigned int leftPin = 23;
unsigned int rightPin = 22;


LineSensor *leftsensor = new LineSensor(leftPin);
LineSensor *rightsensor = new LineSensor(rightPin);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);


}

int leftstat;
int rightstat;

void loop() {
  // put your main code here, to run repeatedly:


  String leftstat = leftsensor->getStatus() ? "OFF TAPE" : "ON TAPE";
  String rightstat = rightsensor->getStatus() ? "OFF TAPE" : "ON TAPE";

  Serial.print("Left sensor: ");
  Serial.println(leftstat);
  Serial.print("Right sensor: ");
  Serial.println(rightstat);
  
  delay(500);
}
