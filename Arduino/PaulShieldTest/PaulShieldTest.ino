#include <PID_v1.h>

// Arduino Motor Shield control pins for motor A
//const int dirPin = 12;
//const int pwmPin = 3;

// Paul's motor shield

const int pwmForward = 9;
const int pwmBackward = 10; //
//const int gnd = 4;

double motorPower = 0;
double motorDeadZone = 0; 

const int motorEnablePin = 6; 


void setup() {

  //---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
  
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  
TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
// bitSet(TCCR2B, WGM12); // set Timer 1 to fast mode so as to match 0. 
  
//  TCCR3B &= (0xff & 0x2); // change pwm frequency to 40k or something
//  TCCR4B &= (0xff & 0x2); // change pwm frequency to 40k or something


  initMotor(); 

  Serial.begin(115200);
}


void loop() {


  motorPower = 10;//sin(millis() / 1000.0f / PI) * 100.0f;
  setMotorPower(motorPower); 
//  int speed = map(abs(round(motorPower)), 0, 255, 0, 255);
//
//  if (motorPower < 0) {
//
//    digitalWrite(pwm1, LOW);
//    digitalWrite(pwm2, LOW);
//
//    //analogWrite(pwm0, speed);
//    digitalWrite(pwm0, HIGH);
//    analogWrite(pwm3, speed);
//
//  } else {
//
//    digitalWrite(pwm0, LOW);
//    digitalWrite(pwm3, LOW);
//
//    analogWrite(pwm1, speed);
//    //analogWrite(pwm2, speed);
//    digitalWrite(pwm2, HIGH);
//
//
//
//  }

  Serial.println(motorPower);
  
}



void initMotor() {

  
  //setPwmFrequency(pwmForward, 1024);
  //setPwmFrequency(pwmBackward, 1024);
  
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

  } else if(power>0) {

    analogWrite(pwmBackward, 255-speed);
    digitalWrite(pwmForward, HIGH);

  } else if(power == 0) { 
  digitalWrite(pwmBackward, LOW);
     digitalWrite(pwmForward, LOW);
  }
}


