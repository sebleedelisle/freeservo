
const int pwmForward = 9; 
const int pwmBackward = 10; 
float motorDeadZone = 0; 

void initMotor() {

  pinMode(pwmForward, OUTPUT);
  digitalWrite(pwmForward, LOW);
  pinMode(pwmBackward, OUTPUT);
  digitalWrite(pwmBackward, LOW);


  //enable pin
  //pinMode(motorEnablePin, OUTPUT);
  //digitalWrite(motorEnablePin, HIGH);

}

void setMotorPower(volatile double power) {
  // power goes from -100 to 100

  int speed = map(abs(round(power)),0,100,motorDeadZone*255,254);

  if (power < 0) {

    analogWrite(pwmForward, 0);
    analogWrite(pwmBackward, speed);

  } 
  else if(power>0) {

    analogWrite(pwmBackward, 0);
    analogWrite(pwmForward, speed);

  } 
  else if(power == 0) { 
    digitalWrite(pwmBackward, LOW);
    digitalWrite(pwmForward, LOW);
  }
}

