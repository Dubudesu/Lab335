#include "Gripper.h"
#include <arduino.h>

#define OPEN_ANGLE  0
#define CLOSE_ANGLE 15

#define OPEN_POSN 1
#define CLOSE_POSN 0

Gripper::Gripper (unsigned int pin) {
    _gripper = new Servo;               // instantiate new MegaServo class instance
    _gripper->attach(pin,800,2200);         // 
    _gripper->write(OPEN_ANGLE);            // drive the gripper open on power up
    _posn = OPEN_POSN;                      // set gripper position to open
    
}

void Gripper::open() {                      // open the gripper
    if(_posn == OPEN_POSN) {                // gripper already open
        return;
    }
    else {
        _gripper->write(OPEN_ANGLE);       // open gripper to the defined OPEN_ANGLE
        _posn = 0;
    }
}

void Gripper::close() {                     // close the gripper
    if(_posn == CLOSE_ANGLE) {              // gripper already closed
        return;
    }
    else {
        _gripper->write(CLOSE_ANGLE);       // close gripper to the defined CLOSE_ANGLE
        _posn = CLOSE_POSN;
    }

}

bool Gripper::getPosn() {                   // return the gripper current position (CLOSED = 0, OPEN = 1)
    return(_posn);
}

