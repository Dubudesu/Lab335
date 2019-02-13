#include <Adafruit_MotorShield.h>

#define ENCODER_SLOTS       20
#define WHEEL_DIAMETER      65  //mm
#define LEFT_ENCODER_PIN    49  //ICP4  
#define RIGHT_ENCODER_PIN   48  //ICP5

volatile bool timer4_capt_flag = false;
volatile bool timer4_over_flag = false;
volatile bool timer5_capt_flag = false;
volatile bool timer5_over_flag = false;


#include <Wire.h>
#include <Encoder.h>

Encoder *leftEncoder    = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, LEFT_ENCODER_PIN);
Encoder *rightEncoder   = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, RIGHT_ENCODER_PIN);

void setup(){

    Serial.begin(115200);
    Serial.print("Encoder initializing..");
    noInterrupts();
    
    TCCR4A =  0B00000000;
    TCCR4B =  0B01000100; // Enable Input Capture Rising Edge - 256 prescaler = 16us per tick
    TIMSK4 =  0B00100001; // Interrupt on Timer 4 overflow and input capture
    TCNT4 =   0;          // Set counter to zero

    interrupts();
    leftEncoder->init();
    rightEncoder->init();

};

void loop(){

    if(timer4_capt_flag){
        
        leftEncoder->updateTime(ICR4);
        timer4_capt_flag = false;
        timer4_over_flag = false;
        Serial.print("Current speed: !");
        Serial.print(leftEncoder->getSpeed());
        Serial.print("\n");
        
    }
    if(timer5_capt_flag){
        rightEncoder->updateTime(ICR4);
        timer5_capt_flag = false;
        timer5_over_flag = false;
        Serial.print("Current speed: !");
        Serial.print(rightEncoder->getSpeed());
        Serial.print("\n");
    }

};

//ISR's here

ISR(TIMER4_CAPT_vect){
    timer4_capt_flag = true;
    timer4_over_flag = false;
}
ISR(TIMER4_OVF_vect){
  if(timer4_over_flag){
    leftEncoder->zeroSpeed();
  }
  timer4_over_flag = 1;
}
ISR(TIMER5_CAPT_vect){
    timer5_capt_flag = true;
    timer5_over_flag = false;
}
ISR(TIMER5_OVF_vect){
  if(timer5_over_flag){
    rightEncoder->zeroSpeed();
  }
  timer5_over_flag = 1;
}
