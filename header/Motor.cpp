
#include "Motor.h"
#include <Adafruit_MotorShield.h>

Motor::Motor(Adafruit_MotorShield *AFMSptr, byte motorNumber, bool polarity) {

    _motorNumber   = motorNumber; // determined by the terminals the motor is wired to on the motor shield
    _polarity      = polarity;    // used to accomodate reversed motor wiring
    _AFMotor       = AFMSptr->getMotor(motorNumber); // Links the Adafruit DCMotor object to the correct motor shield terminals
    _command       = 0; // current motor direction command
    _dutyCycle     = 0; // current motor speed command (0-255) PWM duty cycle
}

// FORWARD 1, BACKWARD 2, BRAKE 3, RELEASE 4, declared in Adafruit motor library
void Motor::driveCmd(unsigned int dutyCycle, unsigned int command) {
    
    _dutyCycle  = dutyCycle; // PWM duty cycle on a 0-255 scale
    _command    = command;
    
    if(command == RELEASE)
        _AFMotor->run(RELEASE);
    else if ((_polarity && _command == FORWARD) || (!_polarity && _command == BACKWARD)) // accomodate reversed motor wiring
        _AFMotor->run(BACKWARD);
    else
        _AFMotor->run(FORWARD);         // if not commanded to release or run backward, run forward.
    
    _AFMotor->setSpeed(_dutyCycle);     // set the motor PWM to the specified duty cycle
}

unsigned int Motor::getSpeed() {
    return(_dutyCycle); // return motor speed PWM value
}

void Motor::stop() {        // stop the motor
    
    _AFMotor->setSpeed(0);
}



