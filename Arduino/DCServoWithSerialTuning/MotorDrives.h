
#pragma once
#include "Arduino.h"

#ifdef USE_MOTOR_SHIELD

// Arduino Motor Shield control pins for motor A
const int dirPin = 12;
const int pwmPin = 3;

void initMotorPins() {

  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  digitalWrite(pwmPin, 0);
  digitalWrite(dirPin, 0);

}

void setMotorPower(volatile double& power) {
  // motorPower automatically updated by the PID algo
  analogWrite(pwmPin, abs(round(power)));
  digitalWrite(dirPin, power < 0 ? HIGH : LOW);

}


#endif

#ifdef USE_PAUL_MOTOR_DRIVE

// Paul's motor shield
// Wiring is easy - wire pins in order straight into pins 2 to 6
// PWM 1     PWM 0     GND     PWM 3    PWM 2
//   \/        \/       \/       \/       \/
//   6          5       4         3       2

const int pwm0 = 5; // 0 and 3 make it go forward
const int pwm1 = 6; // 1 and 2 make it go backward
const int pwm2 = 9;
const int pwm3 = 10; //
//const int grdPin = 4;




void initMotorPins() {
  //pinMode(grdPin, OUTPUT);
  //digitalWrite(grdPin, LOW);
  pinMode(pwm0, OUTPUT);
  digitalWrite(pwm0, LOW);
  pinMode(pwm1, OUTPUT);
  digitalWrite(pwm1, LOW);
  pinMode(pwm2, OUTPUT);
  digitalWrite(pwm2, LOW);
  pinMode(pwm3, OUTPUT);
  digitalWrite(pwm3, LOW);
}

void setMotorPower(volatile double power) {
  int speed = abs(round(power));

  if (power < 0) {

    digitalWrite(pwm1, LOW);
    digitalWrite(pwm2, LOW);

    analogWrite(pwm0, speed);
    analogWrite(pwm3, speed);



  } else {

    digitalWrite(pwm0, LOW);
    digitalWrite(pwm3, LOW);

    analogWrite(pwm1, speed);
    analogWrite(pwm2, speed);



  }
}

#endif
