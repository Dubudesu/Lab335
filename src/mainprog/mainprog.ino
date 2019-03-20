#include <Arduino.h>

#include <Wire.h>
#include <Encoder.h>
#include <Motor.h>
#include <SpeedControl.h>
#include <Gripper.h>
#include <RangeFinder.h>
#include <LineFollower.h>

#include <Adafruit_MotorShield.h>


#define ENCODER_SLOTS       20
#define WHEEL_DIAMETER      65  //mm
#define LEFT_ENCODER_PIN    49  //ICP4  
#define RIGHT_ENCODER_PIN   48  //ICP5
#define POLARITY_LF         0
#define POLARITY_RF         0
#define POLARITY_LR         1
#define POLARITY_RR         0

#define UPDATE_INTERVAL     0.02

volatile bool timer4_capt_flag = false;
volatile bool timer4_over_flag = false;
volatile bool timer5_capt_flag = false;
volatile bool timer5_over_flag = false;

volatile bool pid_update_flag  = false;

volatile int datFlag = 0;

volatile bool MAZE_FLAG = false;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Encoder *leftEncoder;
Encoder *rightEncoder;

Gripper *datGripper;

Motor *lfMotor;
Motor *lrMotor;
Motor *rfMotor;
Motor *rrMotor;

SpeedControl *leftControler;
SpeedControl *rightControler;

/*------------RANGE FINDER-------------- */
unsigned int trig1Pin = 23;    // Trigger
unsigned int trig2Pin = 22;
unsigned int echo1Pin = 19;    // Echo
unsigned int echo2Pin = 18; 

volatile bool MEASURE_FLAG1 = false;
volatile bool MEASURE_FLAG2 = false;

volatile bool ISR1flag = false;
volatile bool ISR2flag = false;

RangeFinder  *frontRanger = new RangeFinder(echo1Pin, trig1Pin, &Serial);
RangeFinder  *leftRanger = new RangeFinder(echo2Pin, trig2Pin, &Serial);

/*------------RANGE FINDER-------------- */

LineFollower *daMaster;

void setup(){
  AFMS.begin();
  Serial.begin(115200);
  Serial.println("Encoder initializing...");

  
  noInterrupts();
    
    //PIN 49 - ICP4
    TCCR4A = 0;
    TCCR4B = B11000011; // Enable Input Capture Rising Edge - 64 prescaler = 4us per tick
    TCCR4C = 0;
    TIMSK4 = B00100001; // Interrupt on Timer 4 overflow and input capture
    TCNT4 =  0;          // Set counter to zero

    //PIN 48 - ICP5
    TCCR5A = 0;
    TCCR5B = B11000011; // Enable Input Capture Rising Edge - 64 prescaler = 4us per tick
    TCCR5C = 0;
    TIMSK5 = B00100001; // Interrupt on Timer 4 overflow and input capture
    TCNT5 =  0;          // Set counter to zero

    TCCR3A = 0; 
    TCCR3B = B00001100; // CTC Mode - 256 prescaler = 16us per tick
    TIMSK3 = B00000100; // Compare match vector B
    OCR3A  = 6250;      // 6250 * 16µs = 100ms, 1250 = 20ms
    TCNT3  = 0;

    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B = _BV(CS12);    // prescaler of 256
    TCNT1  = 0;

    attachInterrupt( digitalPinToInterrupt(echo1Pin), frontSensISR, CHANGE );
    attachInterrupt( digitalPinToInterrupt(echo2Pin), leftSensISR, CHANGE );

    interrupts();
    
    rightEncoder   = new Encoder(&Serial, ENCODER_SLOTS, WHEEL_DIAMETER, RIGHT_ENCODER_PIN);
    leftEncoder    = new Encoder(&Serial, ENCODER_SLOTS, WHEEL_DIAMETER, LEFT_ENCODER_PIN);
    
    leftEncoder->init();
    rightEncoder->init();
    datGripper = new Gripper(10);
    // Motor instantiation syntax: Motor(AFMS, motorNumber, polarity)
    lfMotor         = new Motor(&AFMS, 3, POLARITY_LF);
    lrMotor         = new Motor(&AFMS, 4, POLARITY_LR);
    rfMotor         = new Motor(&AFMS, 2, POLARITY_RF);
    rrMotor         = new Motor(&AFMS, 1, POLARITY_RR);

    leftControler   = new SpeedControl(&Serial, leftEncoder, lfMotor, lrMotor, UPDATE_INTERVAL);
    rightControler  = new SpeedControl(&Serial, rightEncoder, rfMotor, rrMotor, UPDATE_INTERVAL);
  
    leftControler->disable();
    rightControler->disable();
  
    leftControler->setSpeed(400.0, BACKWARD, true);
    rightControler->setSpeed(400.0, FORWARD, true);
    leftControler->update();
    rightControler->update();
}

void loop(){

     if (Serial.available() > 2) {
      int incomingByte = Serial.read();
  
      if(incomingByte == 'o'){
        Serial.println("open!");
        datGripper->open();
      }
      else if(incomingByte == 'c'){
        Serial.println("close!");
        datGripper->close();
      }
      else if(incomingByte == 'f'){
        Serial.println("Front!");
        frontRanger->sendPing();
      }
      else if(incomingByte == 'l'){
        Serial.println("Left!");
        leftRanger->sendPing();
      }
      else if(incomingByte == 'm'){
        
        MAZE_FLAG = !MAZE_FLAG;
        if(MAZE_FLAG){
          Serial.println("MazeGO!");
          mazeStart();
        }else{
          Serial.println("MazeNO!");
          mazeStop();
        }
      }
      else if(incomingByte == 'b'){
        Serial.println("tankLeft!");
        tankLeft();
      }
      else if(incomingByte == 'n'){
        Serial.println("tankRight!");
        tankRight();
      }
    }
  
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
      timer5_capt_flag = false;
      timer5_over_flag = false;
      rightEncoder->updateTime(ICR5);

  }
  if(MEASURE_FLAG1 == true){
    MEASURE_FLAG1 = false;

    frontRanger->update( TCNT1 * 16 );
    if(MAZE_FLAG){
      if( frontRanger->getDistance() < 10.0){
        Serial.println("STOP");
        mazeStop();
      }else{
        Serial.println("GO");
        mazeStart();
      }
    }
  }
  if(MEASURE_FLAG2 == true){
    MEASURE_FLAG2 = false;
    
    leftRanger->update( TCNT1 * 16 );
    if(MAZE_FLAG){
      if( leftRanger->getDistance() < 50.0){
        //Cant turn left, better turn right
        Serial.print("distance: ");
        Serial.print( leftRanger->getDistance() );
        Serial.print("cm\n");
      }
    }
  }


  if(pid_update_flag){
    pid_update_flag = false;

    if(MAZE_FLAG){

      frontRanger->sendPing();
    }else{
      leftControler->update();
    }

  }
  
}

ISR(TIMER4_CAPT_vect){
    timer4_capt_flag = true;
    timer4_over_flag = false;
    datFlag++;
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

void frontSensISR(){
    if( digitalRead(echo1Pin) == HIGH ){
      TCNT1 = 0;
    }
    else if( digitalRead(echo1Pin) == LOW ){
      MEASURE_FLAG1 = true;
    }
}

void leftSensISR(){
    if( digitalRead(echo2Pin) == HIGH ){
      TCNT1 = 0;
    }
    else if( digitalRead(echo2Pin) == LOW ){
      MEASURE_FLAG2 = true;
    }
}


ISR(TIMER3_COMPB_vect){
  pid_update_flag = true;
}

void mazeStart(){
    OCR3A  = 6250;    //100ms delay on timer3, used for sending pings on maze
    leftControler->enable();
    rightControler->enable();
  
    leftControler->setSpeed(100.0, FORWARD, false);
    rightControler->setSpeed(100.0, FORWARD, false);
    leftControler->update();
    rightControler->update();
}

void mazeStop(){
    leftControler->disable();
    rightControler->disable();
  
    leftControler->setSpeed(0.0, FORWARD, false);
    rightControler->setSpeed(0.0, FORWARD, false);
    leftControler->update();
    rightControler->update(); 
}

void tankLeft(){

    leftControler->enable();
    rightControler->enable();
  
    leftControler->setSpeed(100.0, BACKWARD, false);
    rightControler->setSpeed(100.0, FORWARD, false);
    leftControler->update();
    rightControler->update();

    delay(900);
    leftControler->disable();
    rightControler->disable();
    leftControler->setSpeed(0.0, BRAKE, false);
    rightControler->setSpeed(0.0, BRAKE, false);
    leftControler->update();
    rightControler->update();
}

void tankRight(){

    leftControler->enable();
    rightControler->enable();
  
    leftControler->setSpeed(100.0, FORWARD, false);
    rightControler->setSpeed(100.0, BACKWARD, false);
    leftControler->update();
    rightControler->update();

    delay(900);
    leftControler->disable();
    rightControler->disable();
    leftControler->setSpeed(0.0, BRAKE, false);
    rightControler->setSpeed(0.0, BRAKE, false);
    leftControler->update();
    rightControler->update();
}
