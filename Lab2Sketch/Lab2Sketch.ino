
#define ENCODER_SLOTS       20
#define WHEEL_DIAMETER      65  //mm
#define LEFT_ENCODER_PIN    49  //ICP4
#define RIGHT_ENCODER_PIN   48  //ICP5
#define LEFT_LINE_PIN       20
#define RIGHT_LINE_PIN      21

#define POLARITY_LF         0
#define POLARITY_RF         0
#define POLARITY_LR         0
#define POLARITY_RR         0

//Encoder Flags
volatile bool timer4_capt_flag = false;
volatile bool timer4_over_flag = false;
volatile bool timer5_capt_flag = false;
volatile bool timer5_over_flag = false;

//Line Follower Flags
volatile bool left_line_flag   = false;
volatile bool right_line_flag  = false;

#include <Wire.h>
#include <Encoder.h>
#include <Follower.h>
#include <Motor.h>
#include <Adafruit_MotorShield.h>
#include <Arduino.h>

void left_line_isr(){
    left_line_flag = true;
}

void right_line_isr(){
    left_line_flag = true;
}

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Instantiate Encoder Objects
Encoder *leftEncoder        = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, LEFT_ENCODER_PIN);
Encoder *rightEncoder       = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, RIGHT_ENCODER_PIN);

// Motor instantiation syntax: Motor(AFMS, motorNumber, polarity)
Motor *lfMotor              = new Motor(&AFMS, 1, POLARITY_LF);
Motor *lrMotor              = new Motor(&AFMS, 2, POLARITY_LR);
Motor *rfMotor              = new Motor(&AFMS, 3, POLARITY_RF);
Motor *rrMotor              = new Motor(&AFMS, 4, POLARITY_RR);

// Instantiate Follower Objects
Follower *leftFollower      = new Follower(LEFT_ENCODER_PIN, lfMotor, lrMotor);
Follower *rightFollower     = new Follower(LEFT_ENCODER_PIN, rfMotor, rrMotor);

void setup(){
    AFMS.begin();
    Serial.begin(115200);
    Serial.print("Encoder initializing..");
    noInterrupts();
    
    //PIN 49 - Left Encoder counter/interrupts
    TCCR4A =  0B00000000;
    TCCR4B =  0B01000100; // Enable Input Capture Rising Edge - 256 prescaler = 16us per tick
    TIMSK4 =  0B00100001; // Interrupt on Timer 4 overflow and input capture
    TCNT4 =   0;          // Set counter to zero

    //PIN 48 - Right Encoder counter/interrupts
    TCCR5A =  0B00000000;
    TCCR5B =  0B01000100; // Enable Input Capture Rising Edge - 256 prescaler = 16us per tick
    TIMSK5 =  0B00100001; // Interrupt on Timer 5 overflow and input capture
    TCNT5 =   0;          // Set counter to zero

    //Follower interrupts
    attachInterrupt(digitalPinToInterrupt(LEFT_LINE_PIN), left_line_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_LINE_PIN), right_line_isr, CHANGE);
    
    interrupts();

    leftEncoder->init();
    rightEncoder->init();

}

void loop(){

    // send drive commands to each motor
    // variables to set speed and direction for easy change between tests
    // FORWARD 1, BACKWARD 2, BRAKE 3, RELEASE 4, #defined in Adafruit motor library
    static unsigned int dir = 1;
    static unsigned int spd = 100;
    lfMotor->driveCmd(spd, dir);
    lrMotor->driveCmd(spd, dir);
    rfMotor->driveCmd(spd, dir);
    rrMotor->driveCmd(spd, dir);

    if(timer4_capt_flag){

        leftEncoder->updateTime(ICR4);
        timer4_capt_flag = false;
        timer4_over_flag = false;
        Serial.print("Current speed: !");
        Serial.print(leftEncoder->getSpeed());
        Serial.print("\n");

    }
    if(timer5_capt_flag){
        rightEncoder->updateTime(ICR5);
        timer5_capt_flag = false;
        timer5_over_flag = false;
        Serial.print("Current speed: !");
        Serial.print(rightEncoder->getSpeed());
        Serial.print("\n");
    }

    if(left_line_flag){
        left_line_flag = false;
        if(LEFT_LINE_PIN == LOW){            //went from ON tape to OFF tape, begin turning.
            leftFollower->correct();
        }else if(LEFT_LINE_PIN == HIGH){     //went from OFF tape to ON tape, resume basic speed
            leftFollower->resume();
        }
    }
    if(right_line_flag){
        right_line_flag = false;
        if(RIGHT_LINE_PIN == LOW){            //went from ON tape to OFF tape, begin turning.
            rightFollower->correct();
        }else if(RIGHT_LINE_PIN == HIGH){     //went from OFF tape to ON tape, resume basic speed
            rightFollower->resume();
        }
    }

}

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
