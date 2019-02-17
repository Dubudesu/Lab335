#include "Encoder.h"
#include <Arduino.h>

#define COUNT_MAX   65536

Encoder::Encoder(unsigned int slots, unsigned int diameter, unsigned int pin) {
	_speed             = 0.0;
	_currentTime       = 0;
	_prevTime          = 0;
	_deltaTime         = 0;
    _slots             = slots;
	_diameter          = diameter;
  	_pin			   = pin;
    _tick              = 0.000016; //seconds per tick at 256 prescaller
}

//call on interupt flag with time
void Encoder::updateTime(unsigned int time) {
	
    //previous and current time samples
    _prevTime = _currentTime;
	_currentTime = time;

	// Compute _speed and handle Timer Overflow
    unsigned int timediff = 0;
    
    //Check for overflow between samples, and compensate if needed
    if(_currentTime >= _prevTime){
        timediff = _currentTime - _prevTime;
    }else{
        timediff = (_currentTime + COUNT_MAX) - _prevTime;
    }

    //calculate linear speed in mm/s
    _speed = ((360/_slots)/(timediff*_tick))*((PI*_diameter)/360) ;
}

double Encoder::getSpeed() {
	return(_speed);
}

void Encoder::zeroSpeed() {
    _speed = 0.0;
    _prevTime = 0;
    _currentTime = 0;
}

void Encoder::init() {
	pinMode(_pin,INPUT);
}