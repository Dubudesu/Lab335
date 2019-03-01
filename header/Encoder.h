

#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>

class Encoder {
private:
  double            _speed;             // current calculated speed
  double            _tick;              // time between each timer tick based on the prescaler used
  double            _mmPerSlot;         // mm between each slot on the encoder (calculated)
  unsigned int      _currentTime;       // current updateTime() function call time input    
  unsigned int      _prevTime;          // time input from the previous call of the updateTime() function
  unsigned int      _deltaTime;         // time difference between the currentTime and prevTime (calculated)
  unsigned int      _slots;             // number of slots in the encoder wheel used
  unsigned int      _diameter;          // diameter of the robot wheel assoicated with this encoder
  unsigned int      _pin;               // timer interrupt pin used for this encoder

public:
  Encoder(unsigned int slots, unsigned int diameter, unsigned int pin);
  void init();
  double updateTime(unsigned int time); // calculate and return current wheen speed
  double getSpeed(); // return current wheel speed
  void zeroSpeed();  // set speed to zero when the encoder stops moving
};

#endif

