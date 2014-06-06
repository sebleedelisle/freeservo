//#define USE_MOTOR_SHIELD
//#define USE_PAUL_MOTOR_DRIVE
#define USE_RC_SERVO
#define USE_ENCODER_LIBRARY



#include <PID_v1.h>

volatile double position, targetPosition, motorPower;

#include<stdlib.h>
#include "constants.h"
#include "Encoder.h"
#ifdef USE_RC_SERVO
#include <Servo.h> 
#endif

#include "MotorDrives.h"

int errorMargin = 6000; // the number of ticks out of place before the servo goes
// into error.

bool servoError = false;
volatile int stepState = 0; 
volatile int dirState = 0; 

long startOffset = 0;

String command = "";
boolean commandComplete = false;

//double Kp = 0.14, Ki = 0.03, Kd = 0.0013;
//double Kp = 0.88, Ki = 0.02, Kd = 0.0007;
double Kp = 0.77, Ki = 0.37, Kd = 0.0005;

PID myPID(&position, &motorPower, &targetPosition, Kp, Ki, Kd, DIRECT);


void setup() {

  command.reserve(256);
  Serial.begin(9600);

  //TCCR3B &= (0xff & 0x1); // change pwm frequency to 40k or something
  //TCCR4B &= (0xff & 0x1); // change pwm frequency to 40k or something

  initEncoder(); 

  position = targetPosition = motorPower = 0;

  pinMode(errorLightPin, OUTPUT); 
  pinMode(okLightPin, OUTPUT); 
  pinMode(resetPin, INPUT_PULLUP); 
  pinMode(stepPin, INPUT_PULLUP); 
  pinMode(dirPin, INPUT_PULLUP);
  attachInterrupt(stepPin, stepInterruptFired, CHANGE);  
  attachInterrupt(dirPin, dirInterruptFired, CHANGE);  

  initMotor();

  initialisePID();   
    //---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
  
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz



}


void loop() {
  
  updateEncoder(); 
 
  myPID.Compute();

  if (!servoError) {
    setMotorPower(motorPower);
    //Serial.println(motorPower); 
  } //else setMotorPower(0);
  
  //targetPosition = round(((cos((millis()-startOffset) * 0.0008f)) -1) * 1000.0f);

  if (abs(position - targetPosition) > errorMargin) servoError = true;

  if (servoError) { // && ((millis() % 500) < 250)) || (abs(targetPosition - position)>256)) {
    digitalWrite(errorLightPin, HIGH);
  } else {
    digitalWrite(errorLightPin, LOW);
  }
  
  
  if(abs(targetPosition - position)<=10) {
     digitalWrite(okLightPin, HIGH);
  } else{ 
     digitalWrite(okLightPin, LOW);   
  }


  checkSerial();
  if((servoError) && (digitalRead(resetPin) == LOW)) {
     reset();  
    
  }
  
  Serial.println(digitalRead(dirPin)); 
  
  digitalWrite(13, digitalRead(resetPin)); 
}

void reset() { 
   
    servoError = false; 
    startOffset = millis(); 
    encoder.write(0); 
    position = targetPosition = motorPower = 0; 
  
}

void initialisePID() { 
  //encoder.write(0); 
  position = targetPosition = motorPower = 0; 
  myPID.SetOutputLimits(-100, 100); // 80% max power
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetTunings(Kp, Ki, Kd);
  

 
  
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


void stepInterruptFired() { 
  stepState = digitalRead(stepPin); 
  if(!stepState) { 
    if(dirState) targetPosition++; 
    else targetPosition--; 
  } 
  
} 

void dirInterruptFired() { 
  dirState = digitalRead(dirPin); 
  
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

