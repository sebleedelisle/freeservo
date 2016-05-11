

float motorDeadZone = 0.0f;

volatile int binaryHallSensors = 0;

int pwmFreq =   15000;// 23437;

#ifdef USE_BRUSHLESS


///////////////////////////////////////////////////////////////////////////////////////////////
//  BRUSHLESS MOTORS //////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


//const int commutations[6][3] = { { 0, -1, 1}, {1, -1, 0}, {1, 0, -1}, { 0, 1, -1} , { -1, 1, 0}, { -1, 0, 1}};
//const int commutations[6][3] = { { 0, -1, 1}, {1, -1, 0}, {1, 0, -1}, { 0, 1, -1} , { -1, 1, 0}, { -1, 0, 1}};
const int commutations[6][3] = { { 1, -1, 0}, {0, -1, 1}, { -1, 0, 1}, { -1, 1, 0} , {0, 1, -1}, { 1, 0, -1}};


// DEBUG STUFF
int currentMotorOutput = 0;
int stepOffset = 0;
const int motorOutputs[2][3] = { {0, 1, 2},  {2, 1, 0} }; //}, {1,0,2}, { 1,2,0} , {2,0,1}, {2,1,0}};
//String phaseString = "";
int motorOutputPhases[3] = {0, 0, 0};
//const int virtualHallSensorPins[3] = {17, 16,15};


// PHASES :
//      A     B     C     H1   H2   H3   binary (least significant bit = H1)
// 0    +     -     o     1    0    0      1
// 1    o     -     +     1    1    0      3
// 2    -     o     +     0    1    0      2
// 3    -     +     o     0    1    1      6
// 4    o     +     -     0    0    1      4
// 5    +     o     -     1    0    1      5

const int stepForSensors[6] = {0, 2, 1, 4, 5, 3};
const int sensorsForStep[6] = {1, 3, 2, 6, 4, 5};


// OLD VERSIONS :
// PHASES :
//      A     B     C     H1   H2   H3   binary
// 0    +     -     o     0    1    0      2
// 1    o     -     +     0    1    1      6
// 2    -     o     +     0    0    1      4
// 3    -     +     o     1    0    1      5
// 4    o     +     -     1    0    0      1
// 5    +     o     -     1    1    0      3
// PHASES :
//      A     B     C     H1   H2   H3   binary
// 0    o     -     +     1    0    1      5
// 1    +     -     o     1    0    0      1
// 2    +     o     -     1    1    0      3
// 3    o     +     -     0    1    0      2
// 4    -     +     o     0    1    1      6
// 5    -     o     +     0    0    1      4


volatile int currentStep = 0;
volatile int lastStep = -1;
volatile int encoderOffset = 0;

IntervalTimer phaseCheckTimer;

void updateMotor() {

  // motorPower = map(analogRead(A8), 0, 1023, -100, 100);

  int motorSpeed = map(abs(round(motorPower)), 0, 100, motorDeadZone * 1500, 1500); // should be 2047

  int currentstep;// = (currentStep+1)%6;

  if (motorPower < 0) currentstep = (currentStep + stepOffset + 5) % 6;
  else {
    currentstep = (currentStep + stepOffset + 2) % 6; //currentstep = (currentStep+stepOffset-2)%6;

  }
  // if(currentstep<0) currentstep+=6;
  //  Serial.println(currentstep);


  //  for(int i  =0; i<3; i++) {
  //    digitalWrite(enablePins[i], LOW);
  //
  //  }
  for (int i = 0; i < 3; i++) {
    //set enable pin
    int enablepin = enablePins[i];
    //int motorpin = motorPins[i];
    //int motorpin = motorPins[motorOutputs[currentMotorOutput][i]];

    // if the commutation for this phase is :
    //     -1 then it needs to go to ground
    //      0 no connection so enable pin false (otherwise enable pin is set to the pwm power)
    //      1 then connected to voltage.

    int commutation = commutations[currentstep][i];
    digitalWriteFast(enablepin, commutation == 0 ? LOW : HIGH);

    motorOutputPhases[i] = commutations[currentstep][i] ;

  }

  for (int i = 0; i < 3; i++) {
    //set output signal
    // if(motorPower>0)
    int commutation = motorOutputPhases[i];
    int motorpin = motorPins[i];
    if (commutation == 0)
      digitalWriteFast(motorpin, LOW);
    else // if(motorPower>0)
      analogWrite(motorpin, commutation > 0 ? motorSpeed : 0);
    //else  if(motorPower<0)
    //   analogWrite(motorpin, commutations[currentstep][i] < 0 ? motorSpeed : 0);
    // else
    //   analogWrite(motorpin, commutations[currentstep][i] < 0 ? motorSpeed : 0);

  }

}


void updateHallSensors() {

  binaryHallSensors = (digitalReadFast(hallSensorPins[2])  << 2) | (digitalReadFast(hallSensorPins[1]) << 1) | (digitalReadFast(hallSensorPins[0]));
  binaryHallSensors = constrain(binaryHallSensors, 1, 6);

  //currentStep = (stepForSensors[((binaryHallSensors - 1)) % 6])%6;

  //Serial.println(currentStep);
  //if(((currentStep ==1 ) && (lastStep == 0)) ||((currentStep ==0 ) && (lastStep == 1))) {
  //  long pos = xPosn.calcPosn();
  //  if(pos<0) {
  //    pos = abs(pos)%2000;
  //    pos = 2000-pos;
  //  } else {
  //    pos = pos%2000;
  //  }
  //  pos = fmod(pos, 2000.0f/7.0f);
  //  Serial.print(currentStep);
  //  Serial.print(" ");
  //  Serial.println(pos) ;
  ////  Serial.println(fmod(xPosn.calcPosn()%2000, 2000.0f/7.0f)) ;
  //}

  // updateMotor();
  // lastStep = currentStep;

}


const float stepsPerPhase = 2000.0f / 7.0f;

inline void checkPhase() {

  //map(analogRead(A9), 0, 1023, 100, -100);
  //position = xPosn.calcPosn();

  int currentPos = ((int)position - encoderOffset + coilOffset ) % 2000;
  currentPos = currentPos % round(stepsPerPhase);
  currentPos = floor((float)currentPos / stepsPerPhase * 6.0f);
  currentStep = (int)currentPos % 6; //(stepForSensors[((binaryHallSensors - 1)) % 6])%6;
  if (currentStep < 0) currentStep += 6;

  //  if(currentStep!=lastStep) {
  //  // update virtual hall sensors
  //    int  virtualHallSensorState = sensorsForStep[currentStep];
  //    for (int i = 0; i<3; i++) {
  //       digitalWrite(virtualHallSensorPins[i], ((virtualHallSensorState>>i) & 1));
  //    }
  //  }
  if (lastStep != currentStep) {
    updateMotor();
    lastStep = currentStep;
    
  }


}


void findCoilPosition() {

  xPosn.zeroFTM();
  Serial.println("START");
  delay(500);
  int samplePos = 3;

  //motorPower = 50;

  int i = 0;
  for (i = 0; i < 6 && abs(xPosn.calcPosn()) < 80; i++) {
    currentStep = i + samplePos;

    for (int j = 0; j < 100; j += 1) {
      motorPower = j;
      updateMotor();
      delay(2);
    }
    delay(10);
    if (i == 0)  xPosn.zeroFTM();
    Serial.print(i);
    Serial.print(" ");
    Serial.println(xPosn.calcPosn());
  }

  xPosn.zeroFTM();

  samplePos = i;

  //int offsets[20];
  int total = 0;
  int samplecount = 2;
  motorPower = 100;
  for (int i = 0; i < samplecount * 2; i++) {
    currentStep = (i % 2) + samplePos;
    for (int j = 0; j < 100; j += 1) {
      motorPower = j;
      updateMotor();
      delay (2);
    }
    delay(20);
    // delay((currentStep==1) ? 500 : 100);

    //offsets[i] = position;
    //if(currentStep == 1) {
    total += xPosn.calcPosn();
    //}
    Serial.print(currentStep); Serial.print(" ");
    Serial.println(xPosn.calcPosn());
  }

 // delay(1000);


  encoderOffset = total / (samplecount * 2);
  encoderOffset += (2000.0f / 7.0f / 6.0f) * ( 3.4f - samplePos);
  Serial.println(encoderOffset);
  
}


void initMotor() {

  for (int i = 0; i < 3; i++) {

    pinMode(enablePins[i], OUTPUT);
    pinMode(motorPins[i], OUTPUT);

    // pinMode(virtualHallSensorPins[i], OUTPUT);

    // if you wanna chnage the PWM frequency, do it here :
    //  analogWriteFrequency(enablePins[i], 3000);
    analogWriteFrequency(motorPins[i],  pwmFreq);

  }

  analogWriteResolution(11);


  findCoilPosition(); 

  for (int i = 0; i < 3; i++) {
    pinMode(hallSensorPins[i], INPUT);

    attachInterrupt(hallSensorPins[i], updateHallSensors, CHANGE);
  }
  //phaseCheckTimer.begin(checkPhase, 1000000 / 1000); // 1,000 times per second.
  //phaseCheckTimer.priority(250);// lower priority

}



#else



void initMotor() {

  pinMode(forwardEnable, OUTPUT);
  digitalWrite(forwardEnable, HIGH);
  pinMode(backwardEnable, OUTPUT);
  digitalWrite(backwardEnable, HIGH);
  pinMode(pwmForwardSignal, OUTPUT);
  digitalWrite(pwmForwardSignal, LOW);
  pinMode(pwmBackwardSignal, OUTPUT);
  digitalWrite(pwmBackwardSignal, LOW);


  analogWriteFrequency(pwmForwardSignal,  pwmFreq);
  analogWriteFrequency(pwmBackwardSignal, pwmFreq);
  analogWriteResolution(11);

  //enable pin
  //pinMode(motorEnablePin, OUTPUT);
  //digitalWrite(motorEnablePin, HIGH);

}

inline void updateMotor() {
  // motorPower = map(analogRead(A0), 0, 1023, -100, 100);
  //motorPower = -60;
  // motorPower goes from -100 to 100
  int speed = map(abs(round(motorPower)), 0, 100, motorDeadZone * 2047, 2047);

  if (motorPower < 0) {

    //digitalWrite(forwardEnable, HIGH);
    //analogWrite(backwardEnable, speed);
    digitalWriteFast(pwmForwardSignal, LOW);
    analogWrite(pwmBackwardSignal, speed);


  }
  else if (motorPower > 0) {

    //digitalWrite(backwardEnable, HIGH);
    //analogWrite(forwardEnable, speed);
    analogWrite(pwmForwardSignal, speed);
    digitalWriteFast(pwmBackwardSignal, LOW);

  }
  else if (motorPower == 0) {
    //digitalWrite(backwardEnable, HIGH);
    //digitalWrite(forwardEnable, HIGH);
    digitalWriteFast(pwmForwardSignal, LOW);
    digitalWriteFast(pwmBackwardSignal, LOW);
  }
}

#endif

