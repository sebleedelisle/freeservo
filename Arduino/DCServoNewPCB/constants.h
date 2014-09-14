#pragma once

const int errorLightPin = 13; // LED when in error 
const int warnLightPin = 16;
const int okLightPin = 17;    // LED when within 256 of where it should be 

const int ampErrorPin = 7; 

//const int resetPin = A2;

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

const int stepPin = 4; 
const int dirPin = 5; 
#define DBIT_STEP  4
#define DBIT_DIR   5

//const int buzzerPin = 12; 

#ifdef USE_RC_SERVO

// Arduino Motor Shield control pins for motor A
const int servoPin = 7;
int zeroPoint = 93; 

#endif
