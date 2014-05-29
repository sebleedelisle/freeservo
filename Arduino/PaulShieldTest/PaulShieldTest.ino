#include <PID_v1.h>

// Arduino Motor Shield control pins for motor A
//const int dirPin = 12;
//const int pwmPin = 3;

// Paul's motor shield
const int pwm0 = 5; // 0 and 3 make it go forward
const int pwm1 = 6; // 1 and 2 make it go backward
const int pwm2 = 9;
const int pwm3 = 10; //
//const int gnd = 4;

double motorPower = 0;


void setup() {

  //---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
  
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
 bitSet(TCCR1B, WGM12); // set Timer 1 to fast mode so as to match 0. 
  
//  TCCR3B &= (0xff & 0x2); // change pwm frequency to 40k or something
//  TCCR4B &= (0xff & 0x2); // change pwm frequency to 40k or something


  //pinMode(gnd, OUTPUT); 
  //digitalWrite(gnd, LOW); 
  
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


  motorPower = -20; //sin(millis() / 1000.0f / PI) * 150.0f;

  int speed = map(abs(round(motorPower)), 0, 255, 0, 255);

  if (motorPower < 0) {

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

  Serial.println(motorPower);
  
}



