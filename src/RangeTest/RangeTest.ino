
#include <Arduino.h>
#include <RangeFinder.h>

unsigned int trig1Pin = 23;    // Trigger
unsigned int trig2Pin = 22;
unsigned int echo1Pin = 19;    // Echo
unsigned int echo2Pin = 18; 

volatile bool MEASURE_FLAG1 = false;
volatile bool MEASURE_FLAG2 = false;

volatile bool ISR1flag = false;
volatile bool ISR2flag = false;

RangeFinder  *frontRanger = new RangeFinder(echo1Pin, trig1Pin, &Serial);
RangeFinder  *leftRanger = new RangeFinder(echo2Pin, trig2Pin, &Serial);
 
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
  attachInterrupt( digitalPinToInterrupt(echo1Pin), frontSensISR, CHANGE );
  attachInterrupt( digitalPinToInterrupt(echo2Pin), leftSensISR, CHANGE );
  
  interrupts();
  Serial.println("setup done..");
}
 
void loop() {

  if (Serial.available() > 2) {
    int incomingByte = Serial.read();

    if(incomingByte == 'f'){
      Serial.println("Front!");
      frontRanger->sendPing();
    }
    else if(incomingByte == 'l'){
      Serial.println("Left!");
      leftRanger->sendPing();
    }
  }

  if(MEASURE_FLAG1 == true){
    MEASURE_FLAG1 = false;
    frontRanger->update( TCNT1 * 16 );
    Serial.print("distance: ");
    Serial.print( frontRanger->getDistance() );
    Serial.print("cm\n");
  }
    if(MEASURE_FLAG2 == true){
    MEASURE_FLAG2 = false;
    leftRanger->update( TCNT1 * 16 );
    Serial.print("distance: ");
    Serial.print( leftRanger->getDistance() );
    Serial.print("cm\n");
  }

}

void frontSensISR(){
    if( digitalRead(echo1Pin) == HIGH ){
      TCNT1 = 0;
    }
    else if( digitalRead(echo1Pin) == LOW ){
      MEASURE_FLAG1 = true;
    }
}

void leftSensISR(){
    if( digitalRead(echo2Pin) == HIGH ){
      TCNT1 = 0;
    }
    else if( digitalRead(echo2Pin) == LOW ){
      MEASURE_FLAG2 = true;
    }
}
