#define ENCODER_SLOTS       20
#define WHEEL_DIAMETER      65  //mm
#define LEFT_ENCODER_PIN    49    
#define RIGHT_ENCODER_PIN   48

volatile bool timer4_capt_flag = false;
volatile bool timer4_over_flag = false;
volatile bool timer5_capt_flag = false;
volatile bool timer5_over_flag = false;


#include <Wire.h>
#include <Encoder.h>

Encoder *leftEncoder    = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, LEFT_ENCODER_PIN);
Encoder *rightEncoder   = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, RIGHT_ENCODER_PIN);

void setup(){

    Serial.begin(9600);
    noInterrupts();
    
    TCCR4A =  0B00000000;
    TCCR4B =  0B01000100; // Enable Input Capture Rising Edge - NO prescaler

    TIMSK4 =  0B00100001; // Interrupt on Timer 4 overflow and input capture
    
    TCNT4 =   0;          // Set counter to zero

    interrupts();
    
    leftEncoder->init();
    rightEncoder->init();

};

void loop(){

    if(timer4_capt_flag){
       // int speed4 = leftEncoder->updateTime(ICR4);
        timer4_capt_flag = false;
        timer4_over_flag = false;
        Serial.print("Encoder4!");
    }
    if(timer5_capt_flag){
        timer5_capt_flag = false;
        timer5_over_flag = false;
        Serial.print("Encoder5!");
    }

Serial.print("loop running");
//delay(1000);

};

//ISR's here

ISR(TIMER4_CAPT_vect){
    timer4_capt_flag = true;
    timer4_over_flag = false;
    Serial.print("a43e2f");
}
ISR(TIMER5_CAPT_vect){
    timer5_capt_flag = true;
    timer5_over_flag = false;
    Serial.print("a43e2f");
}
