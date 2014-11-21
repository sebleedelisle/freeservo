/* Arduino sketch to take Step & Direction signals from a gcode interpreter (such as a tinyg) and drives
   a dc servo motor with encoder feedback using a Sparkfun Ardumoto shield. This sketch is customized for Two
   motors called X and Z and needs a mega2650 - a single motor version should work on a Uno.
   
   pins are - encoder inputs:
   XPinA 18 // interrupt 5
   XPinB 22
   ZPinA 20  // interrupt 3
   ZPinB 24
   
   Step & direction inputs:
   stepXpin 19
   stepZpin 21
   dirXpin 23
   dirZpin 25
   
   */
   

// Timer2 does not interfere with pwm2 on the mega2650
#include <FlexiTimer2.h>
// excellent PID library with all the professional touches
#include <PID_v1.h>
// use the adafruit motor control library for managing the DC motor
// Depending - a version of this has been hacked to work with the beefier Ardumoto shield.
#include <AFMotor.h>
// quadrature encoder library
#include <Encoder.h>

// create the motor driver instances
AF_DCMotor motorX(2, MOTOR12_8KHZ);

//set up pins for the quadrature encoder - only mega2560 has these, uno only has pins 2,3 and 3 is the 2nd motor pwm output
#define encoderXPinA 2  // interrupt 0
#define encoderXPinB 3 // interrupt 1

#define stepXpin 19  // put these on interrupts
#define dirXpin 23

// For the PIDLib, double 'PositionX' is the Input - these used to be a 'volatile int' as they were updated in ISR but this works
// and eleminates some duplicate variables (space) and copying one to another (time). 
double PositionX;
// For sending output thru serial only when it has changed
double oldPositionX = 0;

//PID controller constants
double KP_X = 8; //position multiplier (gain)
double KI_X = 4.0; // Intergral multiplier (gain) - was 0.05 - use 0 for testing, simple P controller
double KD_X = .03; // derivative multiplier (gain)

// The Output variable motor speed to the motor driver
double ms_X;
// Setpoint 
double target_X = 0;
double old_target_X = 0;


// PID Lib 
PID myPID_X(&PositionX, &ms_X, &target_X,KP_X,KI_X,KD_X, DIRECT);

// encoders
Encoder XEnc(encoderXPinA, encoderXPinB);

void setup() {

  // needed only for debugging
  Serial.begin(115200);

  pinMode(stepXpin, INPUT);
  pinMode(dirXpin, INPUT);

  // the stepper simulator
  //attachInterrupt(4, doXstep, RISING);  // pin 19 interrupt X step 
  
  // we want direction also, change from default 0 to 255 to -255 to 255
  myPID_X.SetOutputLimits(-255,255);
  // compute output every 2mSec - 200mSec way too slow. 
  myPID_X.SetSampleTime(2);
  // PID on
  myPID_X.SetMode(AUTOMATIC);

  // this was tuned to be responsive to rapid movement and yet not step on the encoder interrupts too much  
  // every 2 1/1000ths of a second, or 2mSec
  FlexiTimer2::set(2, 1.0/1000, doPID);
  // all the PID calculation are done in an interrupt so the loop() just takes care of positioning
  FlexiTimer2::start();
  
}

void loop() {

  PositionX = XEnc.read();
  //PositionZ = ZEnc.read();
  // debugging position encoders if needed
 if (oldPositionX != PositionX) {
    Serial.print("X:");
    Serial.println(PositionX);
    oldPositionX = PositionX;
  } 

  // debugging stepping input if needed
  if ( target_X != old_target_X ) {
    Serial.print("target_X:");
    Serial.println(target_X);
    old_target_X = target_X;
  } 


}


void doXstep() {
  if ( digitalRead(dirXpin) == HIGH ) target_X--;
  else target_X++;
}


void doPID() {
  interrupts();   // reading encoders gets off position if this is not interrupted.
  
  myPID_X.Compute();

  int directionX; //determine the direction to go in since the adafruit controller expects positive values
  
  if(ms_X > 0){
    directionX = FORWARD;
  }
  if(ms_X < 0){
    directionX = BACKWARD;
  }
  
  
  motorX.setSpeed(abs(int(ms_X)));
  motorX.run(directionX);
}

