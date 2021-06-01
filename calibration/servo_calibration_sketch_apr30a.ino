#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_MIN 105
#define SERVO_MAX 415

// our servo # counter
uint8_t servonum = 0;

#define SDA_PIN 16
#define SCL_PIN 4
#define BUTTON 13

int value = 105;
unsigned long long t;
boolean bounce=false;
boolean first=true;
boolean test=true;
boolean flipflop=false;
boolean measure_time=false;
int n=0;
int count=0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pinMode(BUTTON, INPUT_PULLUP);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  //position to MIN
  #ifdef SERVO_MIN 

    for(int n=0;n<16;n++)pwm.setPWM(n,0,SERVO_MIN);
  #endif
  
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

int pulseWidth(int angle) {
  
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  }

void loop() {

  if(test){

    if(digitalRead(BUTTON)==0 && !bounce){

      bounce=true;
      measure_time=true;

      t=millis();

      int angle=0;
      if(flipflop)angle=0;
      if(!flipflop)angle=90;

      count++;
      if(count>4){count=0;n++;}
      if(n>15)n=0;
      
      Serial.print("REQUEST ANGLE ");Serial.println(angle);
      pwm.setPWM(n, 0, pulseWidth(angle));
      
      flipflop=!flipflop;      
      }
/*
    if(digitalRead(BUTTON)==0 && !bounce && measure_time){

      bounce=true;
      Serial.print("TIME TO POSITION ");Serial.println(millis()-t);
      measure_time=false;
      }
*/    
  }else{
    
    if(digitalRead(BUTTON)==0 && !bounce){
  
      bounce=true;
      value=value+5;
      Serial.print("VALUE=");Serial.println(value);
  
      first=false;
      }
  
    if(!first)pwm.setPWM(0, 0, value);
    }

  if(digitalRead(BUTTON)==1)bounce=false;
}
