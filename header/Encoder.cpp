#include "Encoder.h"
#include <Arduino.h>

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
	ICR4 = 0;

	return ( (m_diameter*PI)/360.0  ) * ( (360.0/m_slots)/(m_currentTime - m_prevTime)  );

}

double Encoder::getSpeed() {
	return(m_speed);
}

void Encoder::zeroSpeed() {
	m_speed = 0.0;
}

void Encoder::init() {
	pinMode(m_pin,INPUT);
}