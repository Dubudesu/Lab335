#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>

class Encoder {
private:
  double            _speed;
  double            _tick;
  double            _degreePerSlot;
  double            _mmPerDegree;
  unsigned int      _currentTime;
  unsigned int      _prevTime;
  unsigned int      _deltaTime;
  unsigned int      _slots;
  unsigned int      _diameter;
  unsigned int      _pin;

public:
  Encoder(unsigned int slots, unsigned int diameter, unsigned int pin);
  void init();
  double updateTime(unsigned int time);
  double getSpeed();
  void zeroSpeed();
};

#endif
