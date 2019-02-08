#include "Encoder.h"
#include <Arduino.h>

#define COUNT_MAX   65536

Encoder::Encoder(unsigned int slots, unsigned int diameter, unsigned int pin) {
	m_speed             = 0.0;
	m_currentTime       = 0;
	m_prevTime          = 0;
	m_deltaTime         = 0;
    m_slots             = slots;
	m_diameter          = diameter;
  	m_pin				= pin;
}

//call on interupt flag with time
void Encoder::updateTime(unsigned int time) {
	
    m_prevTime = m_currentTime;
	m_currentTime = time;
	// Compute m_speed and handle Timer Overflow

    unsigned int timediff = 0;
    
    //Check for overflow between samples, and compensate if needed
    if(m_currentTime >= m_prevTime){
        timediff = m_currentTime - m_prevTime;
    }else{
        timediff = (m_currentTime + COUNT_MAX) - m_prevTime;
    }

    m_speed = ((360/m_slots)/(timediff*0.000016))*((PI*m_diameter)/360) ;
}

double Encoder::getSpeed() {
	return(m_speed);
}

void Encoder::zeroSpeed() {

    m_speed = 0.0;
    m_prevTime = 0;
    m_currentTime = 0;
}

void Encoder::init() {
	pinMode(m_pin,INPUT);
}