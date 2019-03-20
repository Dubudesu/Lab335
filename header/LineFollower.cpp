#include "LineFollower.h"
#include <Arduino.h>

LineFollower::LineFollower(SpeedControl *lc, SpeedControl *rc, LineSensor *ls, LineSensor *rs, RangeFinder *ranger, HardwareSerial *port){

    _leftController     = lc;
    _rightController    = rc;
    _leftSensor         = ls;
    _rightSensor        = rs;
    _ranger             = ranger;
    _port               = port;

    _state              = S_IDLE;
    _enable             = false;

    _cruiseSpeed        = 80.0;    //  base move speed in Motor PWM
}
void LineFollower::enable(){
    _enable = true;
}
void LineFollower::disable(){
    _enable = false;
}
void LineFollower::update(){

    //read sensor data in
    bool l_sensor = _leftSensor->getStatus();
    bool r_sensor = _rightSensor->getStatus();
    bool distance = _ranger->getDistance();

    switch (_state)
    {
        case S_IDLE:

            //wait for some signal to start
            if(_enable){
                
                if( !l_sensor && !r_sensor ){

                    _leftController->setSpeed(_cruiseSpeed, 1, false);
                    _rightController->setSpeed(_cruiseSpeed, 1, false);
                    _state = S_DRIVE;
                }
                else{
                    _state = S_IDLE;
                    _port->println("Error! cannot start when not over tape!");
                }
            }

            break;
        case S_DRIVE:

            if(_enable){
                
                if( !l_sensor && !r_sensor ){  //if this happens, we goin too fast
                    _leftController->setSpeed(0.0, 1, false);
                    _rightController->setSpeed(0.0, 1, false);
                    _state = S_IDLE;
                    _enable = false;
                }
                else if(!l_sensor){ //left sensor off tape -> turn right
                    _rightController->setSpeed(0.0, 1, false);
                    _state = S_LEFT;
                }
                else if(!r_sensor){ //right sensor off tape -> turn left
                    _leftController->setSpeed(0.0, 1, false);
                    _state = S_RIGHT;
                }
                
            }
            else{
                _leftController->setSpeed(0.0, 1, false);
                _rightController->setSpeed(0.0, 1, false);
                _state = S_IDLE;
                _enable = false;
            }

            break;
        case S_LEFT:

            if( !l_sensor && !r_sensor ){ //back on course go straight
                _leftController->setSpeed(_cruiseSpeed, 1, false);
                _rightController->setSpeed(_cruiseSpeed, 1, false);
                _state = S_DRIVE;
            }

            break;
        case S_RIGHT:


            if( !l_sensor && !r_sensor ){ //back on course go straight
                _leftController->setSpeed(_cruiseSpeed, 1, false);
                _rightController->setSpeed(_cruiseSpeed, 1, false);
                _state = S_DRIVE;
            }

            break;
        case S_STOP:

            break;
    
        default:
            break;
    }
    _port->println(_state);
    _leftController->update();
    _rightController->update();

}