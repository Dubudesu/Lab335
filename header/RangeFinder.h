#ifndef RangeFinder_h
#define RangeFinder_h

#include <Arduino.h>

class RangeFinder {
    private:
        unsigned int    _inPin;
        unsigned int    _outPin;
        double          _distance;
        HardwareSerial *_port;

    public:
        RangeFinder(unsigned int inPin, unsigned int outPin, HardwareSerial *port);

		void sendPing();
        double getDistance();
        void update(double duration );
};

#endif