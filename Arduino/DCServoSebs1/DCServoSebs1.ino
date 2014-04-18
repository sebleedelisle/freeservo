#include <PID_v1.h>
#include<stdlib.h>

// Arduino Motor Shield control pins for motor A
const int dirPin = 12; 
const int pwmPin = 3; 

// encoder pins 
const int aPin = 20; // interrupt 3
const int bPin = 21; // interrupt 2

volatile boolean aState, bState;

double position, targetPosition, motorPower; 

long counter = 0; 

String command = "";
boolean commandComplete = false;

double Kp=5, Ki=10, Kd=0.1;

//PID myPID(&position, &motorPower, &targetPosition,0.02,0.4,0.02,DIRECT);
PID myPID(&position, &motorPower, &targetPosition,5,10,0.1,DIRECT);


void setup() { 
  targetPosition = 10000;
  command.reserve(256);
  Serial.begin(9600); 

  
  
  TCCR3B &= (0xff & 0x2); // change pwm frequency to 40k or something
  
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

 myPID.SetOutputLimits(-205,205); 
 myPID.SetMode(AUTOMATIC);
 myPID.SetSampleTime(5);
 
  
}


void loop() { 

  myPID.Compute();
  
  // motorPower automatically updated by the PID algo
  analogWrite(pwmPin, abs(round(motorPower))); 
  digitalWrite(dirPin, motorPower<0 ? HIGH : LOW);  
  
  if( commandComplete ){
    char commandStr[command.length()];
    char *ptr;
    
    command.toCharArray( commandStr, command.length() );
    
    Kp=strtod(commandStr, &ptr);
    ptr++;
    Ki=strtod(ptr, &ptr);
    ptr++;
    Serial.println( ptr );

    Kd=strtod(ptr, &ptr);
    
    myPID.SetTunings(Kp,Ki,Kd);
  
  
    Serial.print( Kp );
    Serial.print( "," );
    Serial.print( Ki );
    Serial.print( "," );
    Serial.println( Kd );
  
    targetPosition = random(0,8000);
    
    commandComplete = false;
    command = "";
   

  }
  
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

void serialEvent() {
  if (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      commandComplete = true;
    } else {
      command += inChar;
    }
  }
}

