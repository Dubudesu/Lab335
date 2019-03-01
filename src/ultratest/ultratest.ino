/*
 * created by Rui Santos, https://randomnerdtutorials.com
 * 
 * Complete Guide for Ultrasonic Sensor HC-SR04
 *
    Ultrasonic sensor Pins:
        VCC: +5VDC
        Trig : Trigger (INPUT) - Pin11
        Echo: Echo (OUTPUT) - Pin 12
        GND: GND
 */
 
int trigPin = 11;    // Trigger
int echo1 = 12;    // Echo
int echo2 = 13;    // Echo
long duration1, duration2, cm1, cm2;

volatile bool echo_flag = false;
volatile unsigned int t2cnt = 0;
 
void setup() {
  //Serial Port begin
  Serial.begin (115200);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(echo2, INPUT);
  
  noInterrupts();

  attachInterrupt(echo1, junk, CHANGE);
  attachInterrupt(echo2, junk, CHANGE);
  // stop timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B = _BV(WGM12) |  // CTC
           _BV(CS12);    // prescaler of 256
  OCR1A = 2000;
  TIMSK1 = _BV(OCIE1A);  // interrupt on compare A

  interrupts();
}

unsigned int count = 0;
void loop() {

  if(echo_flag){
  
      pulseIn(echo1, HIGH);
      pulseIn(echo2, HIGH);
      duration1 = pulseIn(echo1, HIGH);
      duration2 = pulseIn(echo2, HIGH);
      //Convert the time into a distance
      cm1 = (duration1/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
      cm2 = (duration2/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
      

      Serial.print(cm1);
      Serial.print("   ");
      Serial.print(cm2);
      Serial.println();
      echo_flag = false;
  }
  count ++;
  if(count == 30000){
    Serial.print("WHAT");
  }
}

ISR (TIMER1_COMPA_vect)
{
  PINB = bit (3); // toggle pin 13 every 200ms
  echo_flag = true;
}

void junk(){
  if(digitalRead(echo1)==HIGH ){
     
  }
}
