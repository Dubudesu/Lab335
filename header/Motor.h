#include "arduino.h"
#include <Adafruit_MotorShield.h>

#ifndef Motor_h
#define Motor_h

class Motor {
    private:
        byte                _motorNumber;
        bool                _polarity;
        unsigned int        _command;
        unsigned int        _dutyCycle;
        
        Adafruit_DCMotor    *_AFMotor;
        
    public: 
        Motor(Adafruit_MotorShield *AFMS, byte motorNumber, bool polarity);
        void driveCmd(unsigned int driveNum, unsigned int cmd);
        unsigned int getSpeed();
        void stop();
};

#endif

        
