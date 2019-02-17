#include "Motor.h"
#include <Adafruit_MotorShield.h>

Motor::Motor(Adafruit_MotorShield *AFMSptr, byte motorNumber, bool polarity) {
    _motorNumber   = motorNumber;
    _polarity      = polarity;
    _AFMotor       = AFMSptr->getMotor(motorNumber);
    _command       = 0;
    _dutyCycle     = 0;
}

// FORWARD 1, BACKWARD 2, BRAKE 3, RELEASE 4, #defined in Adafruit motor library

void Motor::driveCmd(unsigned int dutyCycle, unsigned int command) {
    
    _dutyCycle  = dutyCycle; // PWM duty cycle on a 0-255 scale
    _command    = command;
    
    if(command == RELEASE)
        _AFMotor->run(RELEASE);
    else if ((_polarity && _command == FORWARD) || (!_polarity && _command == BACKWARD))
        _AFMotor->run(BACKWARD);
    else
        _AFMotor->run(FORWARD);
    
    _AFMotor->setSpeed(_dutyCycle);
}

unsigned int Motor::getSpeed() {
    
    return(_dutyCycle / 255 * 100); // return motor speed in percent
}

void Motor::stop() {
    
    _AFMotor->setSpeed(0);

}
