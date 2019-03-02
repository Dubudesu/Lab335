#include "SpeedControl.h"
#include <Arduino.h>

SpeedControl::SpeedControl(HardwareSerial *port, Encoder *encoder, Motor *motor1, Motor *motor2, double sampleTime){
    _serialPort         = port;
    _encoder            = encoder;
    _motor1             = motor1;
    _motor2             = motor2;
    _sampleTime         = sampleTime;

    _enable             = 0;
    _setPoint           = 0.0;
    _error              = 0.0;
    _errorIntegral      = 0.0;
    _currentSpeed       = 0.0;
    _prevSpeed          = 0.0;

    //control variables
    _gainIntegral       = 3.3;
    _gainProportional   = 0.0167;
    _integratorLimit    = 0.5; 
}

void SpeedControl::setSpeed(double speed){
    _setPoint       = speed;
}

double SpeedControl::getspeed(){
    return _encoder->getSpeed();
}

void SpeedControl::enable(){
    _enable = 1;
}

void SpeedControl::disable(){
    _enable = 0;
}

void SpeedControl::update(){

    _prevSpeed = _currentSpeed;
    _currentSpeed = _encoder->getSpeed();

    //calculate error
    _error = _currentSpeed - _setPoint;
    _errorIntegral += _error*_sampleTime;

    //Limit the integral windup to avoid overshoot
    _errorIntegral = (_errorIntegral > _integratorLimit) ? _integratorLimit : _errorIntegral;
    _errorIntegral = (_errorIntegral < -_integratorLimit) ? -_integratorLimit : _errorIntegral;

    double pTerm = _gainProportional*_error;
    double iTerm = _gainIntegral*_errorIntegral;

    double newDrive = -(iTerm + pTerm);
    newDrive = 255.0*newDrive;

    newDrive = (newDrive < 0.0) ? 0.0 : newDrive;
    newDrive = (newDrive > 255.0) ? 255.0 : newDrive;

    if(_enable){
        if(newDrive == 0.0){
            _motor1->driveCmd( 0, RELEASE);
            _motor2->driveCmd( 0, RELEASE);
        }
        else{
            _motor1->driveCmd( (int)newDrive, FORWARD);
            _motor2->driveCmd( (int)newDrive, FORWARD);
        }
    }

}


