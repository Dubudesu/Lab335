#include "Encoder.h"
#include <Arduino.h>

#define COUNT_MAX   65535 // maximum value for the timer/counter register used.

Encoder::Encoder(HardwareSerial *port, unsigned int slots, unsigned int diameter, unsigned int pin) {
    _port              = port;
    _speed             = 0.0;
    _currentTime       = 0;
    _prevTime          = 0;
    _deltaTime         = 0;
    _slots             = slots;
    _diameter          = diameter;
    _pin			   = pin;
    _tick              = 0.000004;                          //seconds per tick at 64 prescaller
    _mmPerSlot         = (360/_slots)*((PI*_diameter)/360);  // (degrees per slot) * (mm per degree)
}

//call on interupt flag with time
double Encoder::updateTime(unsigned int time) {
        
    //previous and current time samples
    _prevTime = _currentTime;
    _currentTime = time;

    // Time difference between samples. used long to ensure no register overflows
    unsigned long timediff = 0; 
    timediff = _currentTime - _prevTime;
    //Check for overflow between samples, and compensate if needed
    if(_currentTime >= _prevTime){
        timediff = _currentTime - _prevTime;
    }
    else {
        timediff = (_currentTime + COUNT_MAX) - _prevTime;
    }

    //calculate linear speed in mm/s
    _speed = (_mmPerSlot)/(timediff*_tick);

    return(_speed);   // return the calculated speed
}

double Encoder::getSpeed() {    // return current calculated speed
    return(_speed);
}

void Encoder::zeroSpeed() {     // Encoder wheel stopped moving, zero speed and time values
    _speed = 0.0;
    _prevTime = 0;
    _currentTime = 0;
}

void Encoder::init() {
    pinMode(_pin,INPUT);
}
    
    
    