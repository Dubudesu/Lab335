#ifndef LineSensor_h
#define LineSensor_h

#include <Arduino.h>

class LineSensor {
    private:
        unsigned int _pin;

    public:
        LineSensor(unsigned int pin);
		bool getStatus();
};

#endif