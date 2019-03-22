#include <Arduino.h>

#include <Wire.h>
#include <Encoder.h>
#include <Motor.h>
#include <SpeedControl.h>
#include <Gripper.h>
#include <RangeFinder.h>
#include <LineFollower.h>
#include <LineSensor.h>

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



volatile int datFlag = 0;

volatile unsigned int t2OVFcnt = 0;
volatile bool distance_ping = false;
volatile bool pid_update_flag  = false;

//Control Flags
volatile bool MAZE_FLAG = false;
volatile bool LINE_FLAG = true;
volatile bool PID_FLAG = false;
volatile bool MAN_FLAG = false;

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

unsigned int leftLSpin = 24;
unsigned int rightLSpin = 25;

LineSensor  *leftLS;
LineSensor  *rightLS;

/*------------RANGE FINDER-------------- */
unsigned int trig1Pin = 23;    // Trigger
unsigned int trig2Pin = 22;
unsigned int echo1Pin = 19;    // Echo
unsigned int echo2Pin = 18; 

volatile bool MEASURE_FLAG1 = false;
volatile bool UPDATE_FLAG1 = false;
volatile bool MEASURE_FLAG2 = false;
volatile bool UPDATE_FLAG2 = false;

volatile int rangeSanityCheck = 0;

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

    //PID counter
    TCCR2A = 0; 
    TCCR2B = B00000100; // 256 prescaler = 16us per tick
    TIMSK2 = B00000001; // Overflow vector enable
    TCNT2  = 0;

    //Front sensor counter
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B = _BV(CS12);    // prescaler of 256
    TCNT1  = 0;

    //Left sensor counter
    TCCR3A = 0;
    TCCR3B = 0;
    TCCR3B = _BV(CS32);    // prescaler of 256
    TCNT3  = 0;

    attachInterrupt( digitalPinToInterrupt(echo1Pin), frontSensISR, CHANGE );
    attachInterrupt( digitalPinToInterrupt(echo2Pin), leftSensISR, CHANGE );

    interrupts();
    
    rightEncoder   = new Encoder(&Serial, ENCODER_SLOTS, WHEEL_DIAMETER, RIGHT_ENCODER_PIN);
    leftEncoder    = new Encoder(&Serial, ENCODER_SLOTS, WHEEL_DIAMETER, LEFT_ENCODER_PIN);

    leftLS         = new LineSensor(leftLSpin);    
    rightLS        = new LineSensor(rightLSpin);
    
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
  
    leftControler->setSpeed(0.0, FORWARD, false);
    rightControler->setSpeed(0.0, FORWARD, false);
    leftControler->update();
    rightControler->update();


    daMaster = new LineFollower(leftControler, rightControler, leftLS, rightLS, frontRanger, &Serial);
    
    //if(MAZE_FLAG) mazeStart();
    //if(LINE_FLAG) daMaster->enable();


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
      else if(incomingByte == 't'){
        LINE_FLAG = !LINE_FLAG;
        if(LINE_FLAG){
          Serial.println("LineGO!");
          daMaster->enable();
        }else{
          Serial.println("LineNO!");
            leftControler->setSpeed(0.0, BACKWARD, false);
            rightControler->setSpeed(0.0, FORWARD, false);
            leftControler->update();
            rightControler->update();
            daMaster->disable();
        }
      }
    }

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
  //Front range finder update
  if(MEASURE_FLAG1 == true){
    MEASURE_FLAG1 = false;
    UPDATE_FLAG1  = true;
    frontRanger->update( TCNT1 * 16 );
    Serial.println(frontRanger->getDistance());
  }
  //Left range finder update
  if(MEASURE_FLAG2 == true){
    MEASURE_FLAG2 = false;
    UPDATE_FLAG2  = true;
    leftRanger->update( TCNT3 * 16 );
  }


  if(distance_ping){
    distance_ping = false;
    frontRanger->sendPing();
    //leftRanger->sendPing();
    //Serial.println("ping");
  }
  if(pid_update_flag){
      pid_update_flag = false;
    if(MAZE_FLAG){
      mazeUpdate();
    }
    if(LINE_FLAG){
      daMaster->enable();
      daMaster->update(65.0);
    }
  }


}

ISR(TIMER2_OVF_vect){
  if(t2OVFcnt%500 == 0){   //ping for distance 500* 4µs times per second
    distance_ping = true;
  }
  if(t2OVFcnt%80 == 0 ){   //update pid/linefollower 50 times per second
    pid_update_flag = true; 
  }
  if(t2OVFcnt == 1600){
    t2OVFcnt = 0;
  }
  t2OVFcnt++;
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
      TCNT3 = 0;
    }
    else if( digitalRead(echo2Pin) == LOW ){
      MEASURE_FLAG2 = true;

    }
}

void mazeStart(){
    
    if(UPDATE_FLAG1){
      UPDATE_FLAG1 = false;
      leftControler->enable();
      rightControler->enable();
    
      leftControler->setSpeed(120.0, FORWARD, false);
      rightControler->setSpeed(120.0, FORWARD, false);
      leftControler->update();
      rightControler->update();
    }
}

void mazeStop(){
    leftControler->disable();
    rightControler->disable();
  
    leftControler->setSpeed(0.0, FORWARD, false);
    rightControler->setSpeed(0.0, FORWARD, false);
    leftControler->update();
    rightControler->update(); 
}

void mazeUpdate(){
  
//    Serial.print("Left: ");
//    Serial.print( leftRanger->getDistance() );
//    Serial.print("  Front: ");
//    Serial.println( frontRanger->getDistance() );

    double fr = frontRanger->getDistance();
    double lr = leftRanger->getDistance();

    if(fr > 30.0){
      mazeStart();
    }else if(fr < 30.0){
      mazeStop();
      if(lr < 55.0){
        tankRight();
        mazeStop();
      }else if(lr > 55.0){
        tankLeft();
        mazeStop();
      }
    }
}

void tankLeft(){

    if(UPDATE_FLAG1){
      UPDATE_FLAG1 = false;
      Serial.println("tankLeft");
      leftControler->enable();
      rightControler->enable();
    
      leftControler->setSpeed(100.0, BACKWARD, false);
      rightControler->setSpeed(100.0, FORWARD, false);
      leftControler->update();
      rightControler->update();
  
      delay(825);
      
    }
    //mazeStop();
}

void tankRight(){

    if(UPDATE_FLAG1){
      UPDATE_FLAG1 = false;
      Serial.println("tankRight");
      leftControler->enable();
      rightControler->enable();
    
      leftControler->setSpeed(100.0, FORWARD, false);
      rightControler->setSpeed(100.0, BACKWARD, false);
      leftControler->update();
      rightControler->update();
  
      delay(750);

    }
    //mazeStop();
}
