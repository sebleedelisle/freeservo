#include <LedControl.h>

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

long pow10[] = {1,10,100,1000,10000,100000,1000000,10000000};


unsigned long now = 0; 
unsigned long lastUpdate = 0; 

LedControl lc=LedControl(10,8,9,1);

void setup() {
  pinMode(11, OUTPUT); 
  digitalWrite(11, LOW); 
  pinMode(12, OUTPUT); 
  digitalWrite(12, HIGH);
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
  
  pinMode(motorInAPin, INPUT); 
  pinMode(motorInBPin, INPUT); 
  Serial.begin(115200); 

}


void loop() { 
 // now = micros(); 
 // if(true) { //if((unsigned long) (now-lastUpdate) > 1) {
    digitalWrite(dirOutPin, forward);
    
    digitalWrite(stepOutPin, HIGH); 
    delayMicroseconds(1); 
    digitalWrite(stepOutPin, LOW); 
//    
    if(forward) motorPosition++;
    else motorPosition --; 
   
    if(motorPosition>=50000) {
      digitalWrite(13,LOW); 
      forward = false;
      updateLed(); 
      delay(400); 
      digitalWrite(13,HIGH); 
    } else if (motorPosition<=0) {
      digitalWrite(13,LOW); 
      forward = true;  
      updateLed(); 
      delay(400); 
      digitalWrite(13,HIGH); 
    }
//    lastUpdate = now; 
//  }
  
  //delayMicroseconds(200); 
  
  digitalWrite(encoderAPin, ((motorPosition + 1) >> 1) & 0b1 ); 
  digitalWrite(encoderBPin, ((motorPosition) >> 1) & 0b1);   
  delayMicroseconds(1); 
  //if(forward) digitalWrite(13,HIGH); 
  //else 
  if(motorPosition%128 == 0) updateLed();
  //if(motorPosition%100==0) updateLed(); 
  //digitalWrite(encoderAPin, LOW ); 
  //digitalWrite(encoderBPin, LOW);   
  
  //Serial.print(motorPosition >> 1 &0b1); 
  //Serial.print(" "); 
  //Serial.println((motorPosition+1) >> 1 &0b1); 
}

void updateLed() {
  lc.clearDisplay(0); 
  
  boolean negative = false; 
  
  int32_t pos = motorPosition; 
  if(pos<0) { 
    pos = abs(pos); 
    negative = true; 
    
  }
  
  for(int digit = 0; digit<8; digit++) { 
    unsigned long power= pow10[digit]; //pow(10,digit); 
    if((digit>0) && (pos < power)) {
      if(negative) lc.setChar(0,digit,'-',false);
      break;
    }
   // Serial.println(((unsigned long)motorPosition/power)%10); 
    lc.setDigit(0,digit,(pos/power)%10,false);
  }
  //Serial.println(motorPosition);
  //Serial.println("------------");
}
