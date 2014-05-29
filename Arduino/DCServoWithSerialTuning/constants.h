#pragma once

const int errorLightPin = A0; // LED when in error 
const int okLightPin = A1;    // LED when within 256 of where it should be 

const int resetPin = A2;

// optical encoder
const int aPin = 2; // interrupt 3 Port PD1 (vars not actually used any more)
const int bPin = 3; // interrupt 2 Port PD0

//motor drive
// 0 and 3 make it go forward
// 1 and 2 make it go backward
const int pwm0 = 5; 
const int pwm1 = 9; 
const int pwm2 = 6;
const int pwm3 = 10; 


const int stepPin = 4; 
const int dirPin = 5; 

