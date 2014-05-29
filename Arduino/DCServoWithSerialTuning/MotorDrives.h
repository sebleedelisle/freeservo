
#pragma once
#include "Arduino.h"
#include "pwm.h"


#ifdef USE_PAUL_MOTOR_DRIVE

// Paul's motor shield
// Wiring is easy - wire pins in order straight into pins 2 to 6
// PWM 1     PWM 0     GND     PWM 3    PWM 2
//   \/        \/       \/       \/       \/
//   6          5       4         3       2


//const int grdPin = 4;




void initMotor() {

  pinMode(pwm0, OUTPUT);
  digitalWrite(pwm0, LOW);
  pinMode(pwm1, OUTPUT);
  digitalWrite(pwm1, LOW);
  pinMode(pwm2, OUTPUT);
  digitalWrite(pwm2, LOW);
  pinMode(pwm3, OUTPUT);
  digitalWrite(pwm3, LOW);
  
  //initPWM();

  

}

void setMotorPower(volatile double power) {
 
  //setDuty(power);

  
  int speed = map(abs(round(power)),0,100,5,255);

  if (power < 0) {

    digitalWrite(pwm1, LOW);
    digitalWrite(pwm2, LOW);

    //analogWrite(pwm0, speed);
    digitalWrite(pwm0, HIGH);
    analogWrite(pwm3, speed);



  } else {

    digitalWrite(pwm0, LOW);
    digitalWrite(pwm3, LOW);

     analogWrite(pwm1, speed);
    //analogWrite(pwm2, speed);
    digitalWrite(pwm2, HIGH);



  }
}

#endif


#ifdef USE_MOTOR_SHIELD

// Arduino Motor Shield control pins for motor A
const int dirPin = 12;
const int pwmPin = 3;

void initMotor() {

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

