
#pragma once
#include "Arduino.h"
#include "pwm.h"

double motorDeadZone = 0; 

void initMotor() {

  pinMode(pwmForward, OUTPUT);
  digitalWrite(pwmForward, LOW);
  pinMode(pwmBackward, OUTPUT);
  digitalWrite(pwmBackward, LOW);


  //enable pin
  pinMode(motorEnablePin, OUTPUT);
  digitalWrite(motorEnablePin, HIGH);

}

void setMotorPower(volatile double power) {
  // power goes from -100 to 100

  int speed = map(abs(round(power)),0,100,motorDeadZone*255,254);

  if (power < 0) {

    analogWrite(pwmForward, 255-speed);
    digitalWrite(pwmBackward, HIGH);

  } 
  else if(power>0) {

    analogWrite(pwmBackward, 255-speed);
    digitalWrite(pwmForward, HIGH);

  } 
  else if(power == 0) { 
    digitalWrite(pwmBackward, LOW);
    digitalWrite(pwmForward, LOW);
  }
}

