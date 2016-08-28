#include <LedControl.h>

#include "digitalWriteFast.h"

IntervalTimer stepTimer;

// Pretends to be a motor + encoder but also
// actually sends out the step and dir data for testing.
const int stepOutPin = 2;
const int dirOutPin =  3;

const int encoderAPin = 4;
const int encoderBPin = 5;

const int motorInAPin = A5;
const int motorInBPin = A6;

int32_t motorPosition = 0;

boolean forward = true; 


unsigned long now = 0; 
unsigned long lastUpdate = 0; 

LedControl lc=LedControl(10,12,11,1);

void setup() {

  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,3);
  /* and clear the display */
  lc.clearDisplay(0); 


  pinMode(stepOutPin, OUTPUT);
  pinMode(dirOutPin, OUTPUT);
  pinMode(encoderAPin, OUTPUT);
  pinMode(encoderBPin, OUTPUT);
  digitalWrite(encoderAPin, LOW);
  digitalWrite(encoderBPin, LOW);
  digitalWrite(stepOutPin, LOW);
  digitalWrite(dirOutPin, LOW);

  //pinMode(motorInAPin, INPUT); 
  //pinMode(motorInBPin, INPUT); 
  Serial.begin(115200); 

 
  
  delay(3000);
  
   stepTimer.begin(stepMotor, 3);

}


void loop() { 


 
  updateLed();
  delay(10);
 
}


void stepMotor() { 
  digitalWriteFast(dirOutPin, forward);
  digitalWriteFast(stepOutPin, HIGH); 
  
   // pulse needs to be 1us, hopefully these instructions will be enough! 
  if(forward) motorPosition++;
  else motorPosition --; 

    //delayMicroseconds(1); 
  digitalWriteFast(stepOutPin, LOW); 
  

  digitalWriteFast(encoderAPin, ((motorPosition + 1) >> 1) & 0b1 ); 
  digitalWriteFast(encoderBPin, ((motorPosition) >> 1) & 0b1);   
  //delayMicroseconds(1); 

//  if(motorPosition>=300000) {
//    digitalWrite(13,LOW); 
//    forward = false;
//    updateLed(); 
//    delay(400); 
//    digitalWrite(13,HIGH); 
//  } 
//  else if (motorPosition<=0) {
//    digitalWrite(13,LOW); 
//    forward = true;  
//    updateLed(); 
//    delay(400); 
//    digitalWrite(13,HIGH); 
//  }

  if(motorPosition%300000 == 0) {
    digitalWrite(13,LOW); 
    //forward = false;
    updateLed(); 
    delay(400); 
    //digitalWrite(13,HIGH); 
  } 
  
  
}

inline void updateLed() {
 showNumber(motorPosition/1000); 
}



long power10[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000};

inline void showNumber(int32_t number) {

  boolean negative = false;


  if (number < 0) {
    number = abs(number);
    negative = true;

  }
  boolean minusShown = false; 
  
  for (int digit = 0; digit <8; digit++) {
    long powerval = power10[digit]; //pow(10,digit);
    if ((digit > 0) && (number < powerval)) {
      if (negative && !minusShown) {
        lc.setChar(0, digit, '-', false);
        minusShown = true;
      }
      else {
        lc.setChar(0, digit, ' ', false);
      }
      
    } else {
      lc.setDigit(0, digit, (number / powerval) % 10, false);
    }
  }

}

