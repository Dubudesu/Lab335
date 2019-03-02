#ifndef LineSensor_h
#define LineSensor_h

#include <Arduino.h>

class LineSensor {
    private:
        int _pin;

    public:
        LineSensor(int pin);
		int getStatus();
};

#endif