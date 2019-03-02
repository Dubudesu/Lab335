#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor(int pin){
    _pin = pin;
    pinMode(_pin, INPUT);
}

int LineSensor::getStatus(){
	return digitalRead(_pin);
}