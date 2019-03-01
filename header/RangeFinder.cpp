#include "RangeFinder.h"
#include <Arduino.h>

RangeFinder::RangeFinder(unsigned int inPin, unsigned int outPin, HardwareSerial *port){
    _inPin = inPin;
    _outPin = outPin;
    _port   = port;

    pinMode(_inPin, INPUT);
    pinMode(_outPin, OUTPUT);
}


void RangeFinder::sendPing(){

	digitalWrite(_outPin, LOW);
    delayMicroseconds(5);
    digitalWrite(_outPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_outPin, LOW);

    _port->print("ping");
    
}

double RangeFinder::getDistance(){
    return _distance;
}

void RangeFinder::update(double duration ){
    _distance = (duration/2) * 0.0343;  //distance in CM

}