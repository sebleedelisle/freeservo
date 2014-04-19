//#define USE_MOTOR_SHIELD
#define USE_PAUL_MOTOR_DRIVE



#include <PID_v1.h>
#include<stdlib.h>
#include "MotorDrives.h"

// encoder pins
const int aPin = 20; // interrupt 3 Port PD1 (vars not actually used any more)
const int bPin = 21; // interrupt 2 Port PD0

#define aPort PIND 
#define aPortPin PD1 
#define bPort PIND 
#define bPortPin PD0 


volatile boolean aState, bState;

volatile double position, targetPosition, motorPower;

int errorMargin = 256; // the number of ticks out of place before the servo goes
                       // into error.
                       
bool servoError = false; 

long counter = 0;

String command = "";
boolean commandComplete = false;

double Kp = 0.44, Ki = 0.06, Kd = 0.0019;

PID myPID(&position, &motorPower, &targetPosition, Kp, Ki, Kd, DIRECT);


void setup() {
  targetPosition = 10000;
  command.reserve(256);
  Serial.begin(9600);

  //TCCR3B &= (0xff & 0x1); // change pwm frequency to 40k or something
  //TCCR4B &= (0xff & 0x1); // change pwm frequency to 40k or something

  position = targetPosition = motorPower = 0;

  attachInterrupt(3, aChange, CHANGE);
  attachInterrupt(2, bChange, CHANGE);
  //PCattachInterrupt(aPin, aChange, CHANGE);
  //PCattachInterrupt(bPin, bChange, CHANGE);


  pinMode(aPin, INPUT);
  pinMode(bPin, INPUT);

  aState = digitalRead(aPin);
  bState = digitalRead(bPin);

 // initMotorPins(); 

  myPID.SetOutputLimits(-205, 205);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(5);
  myPID.SetTunings(Kp, Ki, Kd);


}


void loop() {

  myPID.Compute();

  if(!servoError) setMotorPower(motorPower);
  else setMotorPower(0); 
  
  targetPosition = round(sin(millis() * 0.001f) * 10000.0f);
  
  if(abs(position-targetPosition)>errorMargin) servoError = true; 
  
  checkSerial(); 
  
  

}

void checkSerial() { 
  
  if ( commandComplete ) {
    char commandStr[command.length() + 1];
    char *ptr;

    command.toCharArray( commandStr, command.length() + 1 );

    Kp = strtod(commandStr, &ptr);
    ptr++;
    Ki = strtod(ptr, &ptr);
    ptr++;
    Kd = strtod(ptr, &ptr);

    myPID.SetTunings(Kp, Ki, Kd);


    Serial.print( Kp );
    Serial.print( "," );
    Serial.print( Ki );
    Serial.print( "," );
    Serial.println( Kd );

    //targetPosition = random(0,8000);

    commandComplete = false;
    command = "";


  }
}

void aChange() {

  //aState = PINC & (0b100) ;//(PINC & B100 > 0);//
  //aState = digitalRead(aPin);
  aState = (aPort & (1<<aPortPin)) >> aPortPin; 
  if (aState == HIGH) {
    // rising
    if (bState == LOW)
      position++;
    else
      position--;
  }
  else {
    // falling
    if (bState == HIGH)
      position++;
    else
      position --;

  }

}

void bChange() {

   bState = (bPort & (1<<bPortPin)) >> bPortPin; 

  if (bState == HIGH) {
    // rising
    if (aState == HIGH)
      position++;
    else
      position--;
  }
  else {
    // falling
    if (aState == LOW)
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

