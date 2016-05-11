#include "Arduino.h"
///////////////////////////////////////////////////////////////////////////////////////////////
//         Servo drive for teensy
//
//
//        0 -
//        1 -
//        2 -
//        3  - ENC A
//        4  - ENC B
//        5  - MOTOR A ENABLE (PWM)
//        6  - MOTOR B ENABLE (PWM)
//        7  - MOTOR A SIGNAL (HIGH or LOW);
//        8  - MOTOR B SIGNAL (HIGH or LOW);
//        9  - MOTOR C SIGNAL (HIGH or LOW);
//        10 - MOTOR C ENABLE (PWM)
//        11 - STEP
//        12 - DIR
//        13 -
//   A0   14 - PID - D ADJUSTMENT
//   A1   15 - 7 SEG DIN
//   A2   16 - 7 SEG CLK
//   A3   17 - 7 SEG CS
//   A4   18 - RESET SERVO
//   A5   19 - HALL SENSOR A
//   A6   20 - HALL SENSOR B
//   A7   21 - HALL SENSOR C
//   A8   22 - PID - P ADJUSTMENT
//   A9   23 - PID - I ADJUSTMENT
//
////////////////////////////////////////////////////////////////////////////////////////////////


const int stepPin = 11;
const int dirPin = 12;
const int resetSwitch = 18;
const int ledCounterPins[] = {15, 16, 17}; // DIN, CLCK, CS
const int pidTrimPotPins[] = {A8, A9, A0};

double Kp = 15,
       Ki = 5.71 * 0.0002,      //0.001142
       Kd = 0.0184 / 0.0002 ;   //92

#ifdef USE_BRUSHLESS

//        5  - MOTOR A SIGNAL (PWM HIGH or LOW)
//        6  - MOTOR B SIGNAL (PWM HIGH or LOW)
//        7  - MOTOR A ENABLE
//        8  - MOTOR B ENABLE
//        9  - MOTOR C ENABLE
//        10 - MOTOR C SIGNAL (PWM HIGH or LOW)

const int motorPins[]  =   {5, 6, 10};     // these are PWM
const int enablePins[] =   {7, 8, 9};      // these are digital pins
const int hallSensorPins[] = {19, 20, 21}; // any digital pin

#else

const int forwardEnable = 7;  //5;
const int backwardEnable = 8; //6;
const int pwmForwardSignal = 5; // 7;
const int pwmBackwardSignal = 6; //8;

#endif
