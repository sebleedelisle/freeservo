#include <PID_v1.h>

// Arduino Motor Shield control pins for motor A
//const int dirPin = 12;
//const int pwmPin = 3;

// Paul's motor shield
const int pwm0 = 2; // 0 and 3 make it go forward
const int pwm1 = 3; // 1 and 2 make it go backward
const int pwm2 = 4;
const int pwm3 = 5; //



// encoder pins
const int aPin = 20; // interrupt 3
const int bPin = 21; // interrupt 2

volatile boolean aState, bState;

double position, targetPosition, motorPower;

long counter = 0;

//PID myPID(&position, &motorPower, &targetPosition,0.02,0.4,0.02,DIRECT);
PID myPID(&position, &motorPower, &targetPosition, 5, 10, 0.1, DIRECT);


void setup() {

  TCCR3B &= (0xff & 0x1); // change pwm frequency to 40k or something

  position = targetPosition = motorPower = 0;

  attachInterrupt(3, aChange, CHANGE);
  attachInterrupt(2, bChange, CHANGE);
  //PCattachInterrupt(aPin, aChange, CHANGE);
  //PCattachInterrupt(bPin, bChange, CHANGE);


  pinMode(aPin, INPUT);
  pinMode(bPin, INPUT);

  aState = digitalRead(aPin);
  bState = digitalRead(bPin);



  pinMode(pwm0, OUTPUT);
  digitalWrite(pwm0, LOW);
  pinMode(pwm1, OUTPUT);
  digitalWrite(pwm1, LOW);
  pinMode(pwm2, OUTPUT);
  digitalWrite(pwm2, LOW);
  pinMode(pwm3, OUTPUT);
  digitalWrite(pwm3, LOW);

  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);

  Serial.begin(9600);
}


void loop() {


  myPID.Compute();

  targetPosition = 0;//round(sin(millis() * 0.001f) * 10000.0f);

  // motorPower automatically updated by the PID algo
  //analogWrite(pwmPin, abs(round(motorPower)));
  //digitalWrite(dirPin, motorPower<0 ? HIGH : LOW);

  int speed = abs(round(motorPower));

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

  //Serial.println(speed); 

}



void aChange() {

  //aState = PINC & (0b100) ;//(PINC & B100 > 0);//
  aState = digitalRead(aPin);
  if (aState == HIGH) {
    // rising
    if (bState == LOW)
      position++;
    else
      position--;
  }
  else {
    // falling
    if (bState == HIGH)
      position++;
    else
      position --;

  }

}

void bChange() {


  bState = digitalRead(bPin);


  if (bState == HIGH) {
    // rising
    if (aState == HIGH)
      position++;
    else
      position--;
  }
  else {
    // falling
    if (aState == LOW)
      position++;
    else
      position --;

  }

}

