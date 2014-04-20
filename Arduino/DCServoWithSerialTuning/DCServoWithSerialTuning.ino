//#define USE_MOTOR_SHIELD
#define USE_PAUL_MOTOR_DRIVE
#define USE_ENCODER_LIBRARY



#include <PID_v1.h>

volatile double position, targetPosition, motorPower;

#include<stdlib.h>
#include "Encoder.h"
#include "MotorDrives.h"

const int errorLightPin = 13; 


int errorMargin = 2000; // the number of ticks out of place before the servo goes
// into error.

bool servoError = false;

long counter = 0;

String command = "";
boolean commandComplete = false;

double Kp = 0.14, Ki = 0.03, Kd = 0.0013;

PID myPID(&position, &motorPower, &targetPosition, Kp, Ki, Kd, DIRECT);


void setup() {

  command.reserve(256);
  Serial.begin(9600);

  //TCCR3B &= (0xff & 0x1); // change pwm frequency to 40k or something
  //TCCR4B &= (0xff & 0x1); // change pwm frequency to 40k or something

  initEncoder(); 

  position = targetPosition = motorPower = 0;

  pinMode(errorLightPin, OUTPUT); 


  initMotorPins();

  myPID.SetOutputLimits(-205, 205);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetTunings(Kp, Ki, Kd);


}


void loop() {
  
  updateEncoder(); 
  
  
  myPID.Compute();

  
  if (!servoError) setMotorPower(motorPower);
  else setMotorPower(0);

  targetPosition = round((cos(millis() * 0.001f)-1) * 20000.0f);

  if (abs(position - targetPosition) > errorMargin) servoError = true;

  if ((servoError && ((millis() % 500) < 250)) || (abs(targetPosition - position)>256)) {
    digitalWrite(errorLightPin, HIGH);
  } else {
    digitalWrite(errorLightPin, LOW);
  }


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

