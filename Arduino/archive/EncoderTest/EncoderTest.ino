#define ENCODER_USE_INTERRUPTS
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PID_v1.h>
#include "readWriteFast.h"


//---------- ENCODER ----------------------
const int aPin = 2; // interrupt 3 Port PD1 (vars not actually used any more)
const int bPin = 8; // interrupt 2 Port PD0

const int errorLightPin = 13; // LED when in error 
const int warnLightPin = 12;
const int okLightPin = 11;    // LED when within 256 of where it should be 

// --------- PID ---------------------------
volatile int32_t pos = 0;

Encoder encoder(aPin, bPin);

double Kp = 15.5, Ki = 5.71, Kd = 0.0184;
volatile double posDouble, targetPositionDouble, motorPower, errorValue;
volatile long targetPositionLong = 0;

PID myPID(&posDouble, &motorPower, &targetPositionDouble, Kp, Ki, Kd, DIRECT);

// ------------STEP / DIR --------------

const int stepPin = 3; 
const int stepPinInterrupt = 1;
const int dirPin = 5; 

void setup() { 
  
  // PID
  pinMode(errorLightPin, OUTPUT); 
  pinMode(warnLightPin, OUTPUT); 
  pinMode(okLightPin, OUTPUT); 
  
  myPID.SetOutputLimits(-100, 100); 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetTunings(Kp, Ki, Kd);
  
  // STEP/DIR
  // set interrupt
  pinMode(dirPin, INPUT); 
  pinMode(stepPin, INPUT); 
  attachInterrupt(stepPinInterrupt, doStepInterrupt, RISING); 
  
  Serial.begin(115200);
  
} 


void loop() { 
   pos = encoder.read(); 
   
   myPID.Compute();
   
   const int32_t error = abs(pos%1000); 
   
   digitalWriteFast(okLightPin, error==0); 
   
   const int32_t error2 = abs((int32_t)(targetPositionDouble)%1000); 
    digitalWriteFast(warnLightPin, error2<10);
  
   digitalWrite(errorLightPin, error2==0);
   
   
}

void doStepInterrupt() { 
  const int dir = digitalReadFast(dirPin); 
  if(dir) targetPositionDouble++; 
  else targetPositionDouble--; 
 // targetPositionDouble++; 
}
