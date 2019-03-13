#define ENCODER_SLOTS       20
#define WHEEL_DIAMETER      65  //mm
#define LEFT_ENCODER_PIN    49  //ICP4  
#define RIGHT_ENCODER_PIN   48  //ICP5
#define POLARITY_LF         0
#define POLARITY_RF         0
#define POLARITY_LR         1
#define POLARITY_RR         0

#define UPDATE_INTERVAL     0.02


#include <Wire.h>
#include <Encoder.h>
#include <Motor.h>
#include <SpeedControl.h>

#include <Adafruit_MotorShield.h>

volatile bool timer4_capt_flag = false;
volatile bool timer4_over_flag = false;
volatile bool timer5_capt_flag = false;
volatile bool timer5_over_flag = false;

volatile bool pid_update_flag  = false;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Encoder *leftEncoder    = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, LEFT_ENCODER_PIN);
Encoder *rightEncoder   = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, RIGHT_ENCODER_PIN);

Motor *lfMotor;
Motor *lrMotor;
Motor *rfMotor;
Motor *rrMotor;

SpeedControl *leftControler;
SpeedControl *rightControler;

void setup(){
  AFMS.begin();
  Serial.begin(115200);
  Serial.println("Encoder initializing...");
  noInterrupts();
    
    //PIN 49 - ICP4
    TCCR4A = 0;
    TCCR4B = B11000010; // Enable Input Capture Rising Edge - 256 prescaler = 16us per tick
    TCCR4C = 0;
    TIMSK4 = B00100001; // Interrupt on Timer 4 overflow and input capture
    TCNT4 =  0;          // Set counter to zero

    //PIN 48 - ICP5
    TCCR5A = 0;
    TCCR5B = B11000010; // Enable Input Capture Rising Edge - 64 prescaler = 4us per tick
    TCCR5C = 0;
    TIMSK5 = B00100001; // Interrupt on Timer 4 overflow and input capture
    TCNT5 =  0;          // Set counter to zero

    TCCR3A = 0; 
    TCCR3B = B00001100; // CTC Mode - 256 prescaler = 16us per tick
    TIMSK3 = B00000010; // Compare match vector A
    OCR3A  = 1250;      // 6250 * 16µs = 100ms
    TCNT3  = 0;

    interrupts();
    leftEncoder->init();
    rightEncoder->init();

  // Motor instantiation syntax: Motor(AFMS, motorNumber, polarity)
  lfMotor         = new Motor(&AFMS, 3, POLARITY_LF);
  lrMotor         = new Motor(&AFMS, 4, POLARITY_LR);
  rfMotor         = new Motor(&AFMS, 2, POLARITY_RF);
  rrMotor         = new Motor(&AFMS, 1, POLARITY_RR);
  leftControler   = new SpeedControl(&Serial, leftEncoder, lfMotor, lrMotor, UPDATE_INTERVAL);
  rightControler  = new SpeedControl(&Serial, rightEncoder, rfMotor, rrMotor, UPDATE_INTERVAL);

  leftControler->enable();
  rightControler->enable();

  leftControler->setSpeed(100, FORWARD);
  rightControler->setSpeed(100, FORWARD);
}

void loop(){
// send drive commands to each motor for simple motor run and speed feedback test
// FORWARD 1, BACKWARD 2, BRAKE 3, RELEASE 4, #defined in Adafruit motor library
//  static bool runningMotor = false;
//  if(!runningMotor) {
//    static unsigned int dir = 1;
//    leftControler->setSpeed(100, FORWARD);
//    rightControler->setSpeed(100, FORWARD);
//  
//    runningMotor = true;
//  }
  if(timer4_capt_flag){
      timer4_capt_flag = false;
      timer4_over_flag = false;    
      leftEncoder->updateTime(ICR4);  
  }
  if(timer5_capt_flag){
      rightEncoder->updateTime(ICR5);
      timer5_capt_flag = false;
      timer5_over_flag = false;
  }
  if(timer5_over_flag){

  }

  if(pid_update_flag){
    pid_update_flag = false;
    leftControler->update();
  }
}

ISR(TIMER4_CAPT_vect){
    timer4_capt_flag = true;
    timer4_over_flag = false;
}
ISR(TIMER4_OVF_vect){
  if(timer4_over_flag){
    leftEncoder->zeroSpeed();
    timer4_over_flag = false;
  }
  timer4_over_flag = true;
}

ISR(TIMER5_CAPT_vect){
    timer5_capt_flag = true;
    timer5_over_flag = false;
}
ISR(TIMER5_OVF_vect){
  if(timer5_over_flag){
    rightEncoder->zeroSpeed();
    timer5_over_flag = true;
  }
  timer5_over_flag = true;
}

ISR(TIMER3_COMPA_vect){
  pid_update_flag = true;
}
