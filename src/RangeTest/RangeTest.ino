
#include <Arduino.h>
#include <RangeFinder.h>

unsigned int trigPin = 23;    // Trigger
unsigned int echoPin = 21;    // Echo

volatile bool MEASURE_FLAG1 = false;
volatile bool ISRflag = false;

RangeFinder  *disranger = new RangeFinder(echoPin, trigPin, &Serial);
 
void setup() {
  //Serial Port begin
  Serial.begin (115200);

  //Counter setup for range finding
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B = _BV(CS12);    // prescaler of 256
  TCNT1  = 0;
  
  //interrupt attached to the echoPin of the ultrasonic sensor
  attachInterrupt( digitalPinToInterrupt(echoPin), myISR, CHANGE );
  
  interrupts();
  Serial.println("setup done..");
}
 
void loop() {

  if (Serial.available() > 2) {
    int incomingByte = Serial.read();
    disranger->sendPing();
  }

  if(MEASURE_FLAG1 == true){
    MEASURE_FLAG1 = false;
    disranger->update( TCNT1 * 16 );
    Serial.print("distance: ");
    Serial.print( disranger->getDistance() );
    Serial.print("cm\n");
  }

}

void myISR(){

    if( digitalRead(echoPin) == HIGH ){
      TCNT1 = 0;
    }
    else if( digitalRead(echoPin) == LOW ){
      MEASURE_FLAG1 = true;
    }
}
