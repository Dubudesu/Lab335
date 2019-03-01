#ifndef SpeedControl_h
#define SpeedControl_h

#include <Arduino.h>
#include <Motor.h>
#include <Encoder.h>

class SpeedControl {
    private:
        
        HardwareSerial  *_serialPort;

        Encoder     *_encoder;
        Motor       *_motor1;
        Motor       *_motor2;

        double      _setPoint;
        double      _sampleTime;
        bool        _enable;

        double      _currentSpeed;
        double      _prevSpeed;
        double      _error;
        double      _errorIntegral;

        double      _gainProportional;
        double      _gainIntegral;
        double      _integratorLimit;

    public:
        SpeedControl(HardwareSerial *port, Encoder *encoder, Motor *motor1, Motor *motor2, double sampleTime);
		
        void        setSpeed(double speed);
        double      getspeed();

        void        enable();
        void        disable();
        void        update();
};

#endif