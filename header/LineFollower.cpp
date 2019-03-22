#include "LineFollower.h"
#include <Arduino.h>

#define TURNVAL     0.1

LineFollower::LineFollower(SpeedControl *lc, SpeedControl *rc, LineSensor *ls, LineSensor *rs, RangeFinder *ranger, HardwareSerial *port){

    _leftController     = lc;
    _rightController    = rc;
    _leftSensor         = ls;
    _rightSensor        = rs;
    _ranger             = ranger;
    _port               = port;

    _state              = S_IDLE;
    _nextState          = S_IDLE;
    _enable             = false;

    _cruiseSpeed        = 80.0;    //  base move speed in Motor PWM
    _stopDistance       = 35.0;
}
void LineFollower::enable(){
    _enable = true;
}
void LineFollower::disable(){
    _enable = false;
}
void LineFollower::update(double speed){
    _cruiseSpeed = speed;
    //read sensor data in
    bool l_sensor = !_leftSensor->getStatus();
    bool r_sensor = !_rightSensor->getStatus();
    double distance = _ranger->getDistance();
    //_port->println(distance);



    if(_enable){
        switch (_state)
        {
            case S_IDLE:
                    //_port->println(_state);
                    if( !l_sensor && !r_sensor ){
                        _nextState = S_DRIVE;
                    }
                    else{
                        //_port->println("Error! cannot start when not over tape!");
                        _nextState = S_IDLE;
                    }

                break;
            case S_DRIVE:
                //_port->println(_state);
                    if(distance < _stopDistance ){
                        _nextState = S_STOP;
                    }
                    else if( l_sensor && r_sensor ){  //if this happens, we goin too fast
                        _nextState = S_IDLE;
                    }
                    else if(l_sensor && !r_sensor){ //left sensor off tape -> turn right
                        _nextState = S_RIGHT;
                    }
                    else if(!l_sensor && r_sensor){ //right sensor off tape -> turn left
                        _nextState = S_LEFT;
                    }else{
                        _nextState = S_DRIVE;
                    }

                break;
            case S_LEFT:
                    if(distance < _stopDistance ){
                        _nextState = S_STOP;
                    }
                    else if( !l_sensor && !r_sensor ){ //back on course go straight
                        _nextState = S_DRIVE;
                    }
                    else if( l_sensor && !r_sensor ){ //back on course go straight
                        _nextState = S_RIGHT;
                    }
                    else{
                        _nextState = S_LEFT;
                    }

                break;
            case S_RIGHT:
                
                    if(distance < _stopDistance ){
                    _nextState = S_STOP;
                    }
                    else if( !l_sensor && !r_sensor ){ //back on course go straight
                        _nextState = S_DRIVE;
                    }
                    else if(!l_sensor && r_sensor){ //right sensor off tape -> turn left
                        _nextState = S_LEFT;
                    }
                    else{
                        _nextState = S_RIGHT;
                    }
                break;
            case S_STOP:
                    if(distance > _stopDistance ){
                        _nextState = S_DRIVE;
                    }
                    else{
                        _nextState = S_STOP;
                    }
                break;
        
            default:
                break;
        }
    }
    else{
        _nextState = S_IDLE;
    }

    _state = _nextState;
     switch (_state)
    {
        case S_IDLE:
                _leftController->setSpeed(0.0, BRAKE, false);
                _rightController->setSpeed(0.0, BRAKE, false);
            break;
        case S_DRIVE:
                _leftController->setSpeed(_cruiseSpeed, FORWARD, false);
                _rightController->setSpeed(_cruiseSpeed, FORWARD, false);
            break;
        case S_LEFT:
                _rightController->setSpeed(_cruiseSpeed, BACKWARD, false);  
                _leftController->setSpeed(_cruiseSpeed*1.2, FORWARD, false);
            break;
        case S_RIGHT:
                _rightController->setSpeed(_cruiseSpeed*1.2, FORWARD, false);
                _leftController->setSpeed(_cruiseSpeed, BACKWARD, false);
            break;
        case S_STOP:
                _leftController->setSpeed(0.0, BRAKE, false);
                _rightController->setSpeed(0.0, BRAKE, false);
            break;
        default:
            break;
    }

    _leftController->update();
    _rightController->update();
}