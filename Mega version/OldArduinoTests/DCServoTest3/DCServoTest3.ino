#include <PID_v1.h>



const int dirPin = 12; 
const int pwmPin = 3; 

const int aPin = 20; 
const int bPin = 21; 

float counter = 0; 

boolean aState = HIGH; 
boolean bState = HIGH; 

double position = 0; 
double targetPosition = 0; 
double motorPower = 0; 

PID myPID(&position, &motorPower, &targetPosition,2,5,1, DIRECT);


void setup() { 

  pinMode(dirPin, OUTPUT); 
  pinMode(pwmPin, OUTPUT); 
  digitalWrite(pwmPin, 0); 
  digitalWrite(dirPin, 0); 
  
  
  pinMode(aPin, INPUT); 
  pinMode(bPin, INPUT); 
  
  aState = digitalRead(aPin); 
  bState = digitalRead(bPin); 

  //attachInterrupt(0, changeA, CHANGE); 
  //attachInterrupt(0, fallA, FALLING); 
  //attachInterrupt(1, changeB, CHANGE); 
  //attachInterrupt(1, fallB, FALLING);
   
   myPID.SetMode(AUTOMATIC);
 
  Serial.begin(38400); 

}


void loop() { 

  checkEncoder(); 
  
  targetPosition = round(sin(millis()*0.001f) * 2000.0f); 
  
  myPID.Compute(); 
 
  analogWrite(pwmPin, min(abs(round(smoothedMotorPower)), 255)); 
  digitalWrite(dirPin, motorPower<0 ? HIGH : LOW);  
 
  counter++; 
 
}



void checkEncoder() { 
  
    boolean newstate = digitalRead(aPin); 
  
  if(newstate!=aState) { 
    if(newstate==HIGH) riseA(); 
    else fallA(); 
  }
  
  newstate = digitalRead(bPin); 
  
  if(newstate!=bState) { 
   if(newstate == HIGH) riseB();
   else fallB(); 
  } 
  
  
}



void riseA() { 
    aState = HIGH; 
    if(bState == LOW) position ++ ; 
    else position --; 
    //Serial.println("RISE A"); 
  
}

void fallA() { 
   aState = LOW; 

    if(bState == HIGH) position ++ ; 
    else position --; 
  
    //Serial.println("FALL A");   
}




void changeA() { 
  boolean newState = digitalRead(2); 
  //Serial.println(newState); 

  if(newState == HIGH) {
    // rising; 
    aState = HIGH; 
    if(bState == LOW) position ++ ; 
    else position --; 
  } 
  else { 
    // falling 
    aState = LOW; 

    if(bState == HIGH) position ++ ; 
    else position --; 
  }

  // else fallA();  


}









void riseB() { 
    bState = HIGH; 
    if(aState == HIGH) position ++ ; 
    else position --; 
  
  
}


void fallB() {
    bState = LOW; 

    if(aState == LOW) position ++ ; 
    else position --; 
  
  
} 
  

void changeB() { 
  boolean newState = digitalRead(3); 
  //Serial.println(newState); 

  if(newState == HIGH) {
    // rising; 
    bState = HIGH; 
    if(aState == HIGH) position ++ ; 
    else position --; 
  } 
  else { 
    // falling 
    bState = LOW; 

    if(aState == LOW) position ++ ; 
    else position --; 
  }

  // else fallA();  


}
