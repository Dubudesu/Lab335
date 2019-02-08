#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>

class Encoder {
private:
  double m_speed;
  unsigned int m_currentTime;
  unsigned int m_prevTime;
  unsigned int m_deltaTime;
  unsigned int m_slots;
  unsigned int m_diameter;
  unsigned int m_pin;
  

public:
  Encoder(unsigned int slots, unsigned int diameter, unsigned int pin);
  void init();
  void updateTime(unsigned int time);
  double getSpeed();
  void zeroSpeed();
};

#endif
