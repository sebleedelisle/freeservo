#pragma once

const int errorLightPin = 13; // LED when in error 
const int warnLightPin = 12;
const int okLightPin = 11;    // LED when within 256 of where it should be 
const int serialRX = 11;
const int serialTX = 12;
const int ampErrorPin = 7; 

// optical encoder
const int aPin = 2; // interrupt 3 Port PD1 (vars not actually used any more)
const int bPin = 3; // interrupt 2 Port PD0


#ifdef USE_4_PWM
//motor drive
// 0 and 3 make it go forward
// 1 and 2 make it go backward
const int pwm0 = 5; 
const int pwm1 = 9; 
const int pwm2 = 6;
const int pwm3 = 10; 
#endif


#ifdef USE_2_PWM

const int pwmForward = 9;
const int pwmBackward = 10;

#endif


#ifdef USE_ARDUMOTO

const int motorPwm = 9;
const int motorDir = 10;

#endif

const int stepPin = 14; //PJ1
const int dirPin = 15; //PJ0
#define DBIT_STEP  1    
#define DBIT_DIR   0

//const int buzzerPin = 12; 


