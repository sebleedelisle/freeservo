#include <PID_v1.h>

// Arduino Motor Shield control pins for motor A
//const int dirPin = 12;
//const int pwmPin = 3;

// Paul's motor shield

const int pwmForward = 9;
const int pwmBackward = 10; //
//const int gnd = 4;

//-----------------------------------------------
double motorPower = -100; // MOTOR POWER PERCENTAGE
//-----------------------------------------------

double motorDeadZone = 0; 

const int motorEnablePin = 6; 


void setup() {

   // set PWM frequency to 31250
  setPwmFrequency(9, 1);
  setPwmFrequency(10, 1);

  initMotor(); 

  Serial.begin(115200);
}


void loop() {


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


//
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}



