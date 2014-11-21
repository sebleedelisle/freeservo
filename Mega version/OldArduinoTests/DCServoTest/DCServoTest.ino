

const int dirPin = 12; 
const int pwmPin = 3; 

const int aPin = 20; 
const int bPin = 21; 

float motorPower = 0;
float motorPowerVel = 0; 
float smoothedMotorPower = 0; 
float counter = 0; 

boolean aState = HIGH; 
boolean bState = HIGH; 

long position = 0; 
long targetPosition = 0; 

void setup() { 

  pinMode(dirPin, OUTPUT); 
  pinMode(pwmPin, OUTPUT); 
  digitalWrite(pwmPin, 0); 
  digitalWrite(dirPin, 0); 
  
  
  pinMode(aPin, INPUT); 
  pinMode(bPin, INPUT); 

  //attachInterrupt(0, changeA, CHANGE); 
  //attachInterrupt(0, fallA, FALLING); 
  //attachInterrupt(1, changeB, CHANGE); 
  //attachInterrupt(1, fallB, FALLING);

  Serial.begin(38400); 

}


void loop() { 

  checkEncoder(); 
  
  
  targetPosition = round(sin(millis()*0.001f) * 2000.0f); 
  checkEncoder();
  
  //motorPower*=0.0;
  //motorPowerVel = 0;  
  motorPower = map(targetPosition-position, -500,500,-500,500); //sin(counter*0.001f) * 100.0f; 
  //motorPower+=motorPowerVel; 
  
  
  if(motorPower>255) motorPower = 255;
   else if (motorPower<-255) motorPower = -255;  
  
  smoothedMotorPower += ((motorPower - smoothedMotorPower) *0.1); 
  
  checkEncoder(); 

  analogWrite(pwmPin, min(abs(round(smoothedMotorPower)), 255)); 
  checkEncoder(); 

  digitalWrite(dirPin, motorPower<0 ? HIGH : LOW);  
  //Serial.println(motorSpeed); 
  checkEncoder(); 

  // delay(1); 
  
//  if((round(counter) % 100) == 0) { 
//    Serial.print(targetPosition); 
//    Serial.print(" "); 
//    Serial.println(position); 
//  }




  counter++; 
  checkEncoder(); 

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
