#pragma once


const int errorLightPin = 13; // LED when in error 
const int warnLightPin = 12;
const int okLightPin = 11;    // LED when within 256 of where it should be 


const int stepPin = 4; 
const int dirPin = 5; 
#define DBIT_STEP  4
#define DBIT_DIR   5


const int serialRX = 11;
const int serialTX = 12;
const int ampErrorPin = 7; 
const int motorEnablePin = 6; 

// optical encoder
const int aPin = 2; // interrupt 3 Port PD1 (vars not actually used any more)
const int bPin = 3; // interrupt 2 Port PD0

const int pwmForward = 9;
const int pwmBackward = 10;

const int buzzerPin = 45; 


