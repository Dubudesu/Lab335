#include <Adafruit_MotorShield.h>

#define ENCODER_SLOTS       20
#define WHEEL_DIAMETER      65  //mm
#define LEFT_ENCODER_PIN    49  //ICP4  
#define RIGHT_ENCODER_PIN   48  //ICP5
#define POLARITY_LF         0
#define POLARITY_RF         0
#define POLARITY_LR         0
#define POLARITY_RR         0

volatile bool timer4_capt_flag = false;
volatile bool timer4_over_flag = false;
volatile bool timer5_capt_flag = false;
volatile bool timer5_over_flag = false;


#include <Wire.h>
#include <Encoder.h>
#include <Motor.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Encoder *leftEncoder    = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, LEFT_ENCODER_PIN);
Encoder *rightEncoder   = new Encoder(ENCODER_SLOTS, WHEEL_DIAMETER, RIGHT_ENCODER_PIN);

Motor *lfMotor;
Motor *lrMotor;
Motor *rfMotor;
Motor *rrMotor;

void setup(){
  AFMS.begin();
  Serial.begin(115200);
  Serial.println("Encoder initializing..");
  noInterrupts();
    
    //PIN 49
    TCCR4A =  0B00000000;
    TCCR4B =  0B01000100; // Enable Input Capture Rising Edge - 256 prescaler = 16us per tick
    TIMSK4 =  0B00100001; // Interrupt on Timer 4 overflow and input capture
    TCNT4 =   0;          // Set counter to zero

    //PIN 48
    TCCR5A =  0B00000000;
    TCCR5B =  0B01000100; // Enable Input Capture Rising Edge - 256 prescaler = 16us per tick
    TIMSK5 =  0B00100001; // Interrupt on Timer 4 overflow and input capture
    TCNT5 =   0;          // Set counter to zero

    interrupts();
    leftEncoder->init();
    rightEncoder->init();

  // Motor instantiation syntax: Motor(AFMS, motorNumber, polarity)
  lfMotor = new Motor(&AFMS, 1, POLARITY_LF);
  lrMotor = new Motor(&AFMS, 2, POLARITY_LR);
  rfMotor = new Motor(&AFMS, 3, POLARITY_RF);
  rrMotor = new Motor(&AFMS, 4, POLARITY_RR);

}

void loop(){

// send drive commands to each motor
// variables to set speed and direction for easy change between tests
// FORWARD 1, BACKWARD 2, BRAKE 3, RELEASE 4, #defined in Adafruit motor library
  static unsigned int dir = 2;
  static unsigned int spd = 100;
  lfMotor->driveCmd(spd, dir);
  lrMotor->driveCmd(spd, dir);
  rfMotor->driveCmd(spd, dir);
  rrMotor->driveCmd(spd, dir);

  if(timer4_capt_flag){
      
      leftEncoder->updateTime(ICR4);
      timer4_capt_flag = false;
      timer4_over_flag = false;
      Serial.print("Current speed left: ");
      Serial.print(leftEncoder->getSpeed());
      Serial.print("\n");
      
  }
  if(timer5_capt_flag){
      rightEncoder->updateTime(ICR4);
      timer5_capt_flag = false;
      timer5_over_flag = false;
      Serial.print("Current speed right: !");
      Serial.print(rightEncoder->getSpeed());
      Serial.print("\n");
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
