#include <PID_v1.h>

// Arduino Motor Shield control pins for motor A
const int dirPin = 12; 
const int pwmPin = 3; 

// encoder pins 
const int aPin = 20; // interrupt 3
const int bPin = 21; // interrupt 2

volatile boolean aState, bState; 

double position, targetPosition, motorPower; 

long counter = 0; 

//PID myPID(&position, &motorPower, &targetPosition,0.02,0.4,0.02,DIRECT);
PID myPID(&position, &motorPower, &targetPosition,5,10,0.1,DIRECT);


void setup() { 
  
  TCCR3B &= (0xff & 0x1); // change pwm frequency to 40k or something
  
  position = targetPosition = motorPower = 0;   

  attachInterrupt(3, aChange, CHANGE);
  attachInterrupt(2, bChange, CHANGE);
  //PCattachInterrupt(aPin, aChange, CHANGE);
  //PCattachInterrupt(bPin, bChange, CHANGE);


  pinMode(aPin, INPUT); 
  pinMode(bPin, INPUT); 

  aState = digitalRead(aPin); 
  bState = digitalRead(bPin); 



  pinMode(dirPin, OUTPUT); 
  pinMode(pwmPin, OUTPUT); 
  digitalWrite(pwmPin, 0); 
  digitalWrite(dirPin, 0); 

  myPID.SetOutputLimits(-255,255); 
 myPID.SetMode(AUTOMATIC);
 myPID.SetSampleTime(1);
 
  Serial.begin(38400); 
}


void loop() { 


  myPID.Compute(); 

  targetPosition = round(sin(millis()*0.001f) * 10000.0f); 

  // motorPower automatically updated by the PID algo
  analogWrite(pwmPin, abs(round(motorPower))); 
  digitalWrite(dirPin, motorPower<0 ? HIGH : LOW);  

  
  
}



void aChange() { 

  

  //aState = PINC & (0b100) ;//(PINC & B100 > 0);//
  aState = digitalRead(aPin);      
  if(aState == HIGH) {
    // rising
    if(bState == LOW)  
      position++; 
    else 
      position--; 
  } 
  else { 
    // falling
    if(bState == HIGH) 
      position++; 
    else 
      position --; 

  }

}

void bChange() { 


  bState = digitalRead(bPin);  

  
  if(bState == HIGH) {
    // rising
    if(aState == HIGH)  
      position++; 
    else 
      position--; 
  } 
  else { 
    // falling
    if(aState == LOW) 
      position++; 
    else 
      position --; 

  }

}

