#ifndef Gripper_h
#define Gripper_h

#include <MegaServo.h>

class Gripper {
    private:
        bool _posn;                 // Current gripper position
        
        MegaServo _gripper;         // MegaServo object instance
        
    
    public:
        Gripper(unsigned int pin);  // Instantiation function
        void open();                // open gripper
        void close();               // close gripper
        bool getPosn();             // return gripper current position
        
};

#endif