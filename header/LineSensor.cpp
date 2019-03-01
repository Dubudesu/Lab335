#include "LineSensor.h"
#include <Arduino.h>

LineSensor::LineSensor(unsigned int pin){
    pinMode(_pin, INPUT);
}

bool LineSensor::getStatus(){
	return digitalRead(_pin);
}