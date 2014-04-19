#include <PID_v1.h>

// Arduino Motor Shield control pins for motor A
//const int dirPin = 12;
//const int pwmPin = 3;

// Paul's motor shield
const int pwm0 = 5; // 0 and 3 make it go forward
const int pwm1 = 6; // 1 and 2 make it go backward
const int pwm2 = 2;
const int pwm3 = 3; //
const int gnd = 4;

double motorPower = 0;


void setup() {

//  TCCR3B &= (0xff & 0x2); // change pwm frequency to 40k or something
//  TCCR4B &= (0xff & 0x2); // change pwm frequency to 40k or something


  pinMode(gnd, OUTPUT); 
  digitalWrite(gnd, LOW); 
  
  pinMode(pwm0, OUTPUT);
  digitalWrite(pwm0, LOW);
  pinMode(pwm1, OUTPUT);
  digitalWrite(pwm1, LOW);
  pinMode(pwm2, OUTPUT);
  digitalWrite(pwm2, LOW);
  pinMode(pwm3, OUTPUT);
  digitalWrite(pwm3, LOW);

  Serial.begin(9600);
}


void loop() {


  motorPower = 0;//sin(millis() / 500.0f / PI) * 120.0f;

  int speed = map(abs(round(motorPower)), 0, 255, 0, 255);

  if (motorPower < 0) {

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

  Serial.println(motorPower);
  
}



