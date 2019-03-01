#ifndef LineFollower_h
#define LineFollower_h

#include <Arduino.h>
#include <SpeedControl.h>
#include <LineSensor.h>
#include <RangeFinder.h>

enum state{S_IDLE, S_DRIVE, S_LEFT, S_RIGHT, S_STOP};

class LineFollower {
    private:

        SpeedControl        *_leftController;
        SpeedControl        *_rightController;
        LineSensor          *_leftSensor;
        LineSensor          *_rightSensor;
        RangeFinder         *_ranger;
        HardwareSerial      *_port;

        double              _cruiseSpeed;
        state               _state;

        bool                _enable;

    public:
        LineFollower(SpeedControl *lc, SpeedControl *rc, LineSensor *ls, LineSensor *rs, RangeFinder *ranger, HardwareSerial *port);

        void enable();
        void disable();
        void update();
        
};

#endif