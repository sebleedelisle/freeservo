
const int outMax = 100;
const int outMin = -100;
double lastInput = 0;
double ITerm = 0;

IntervalTimer pidTimer;

long counter = 0;


inline void computePid() {
//  counter++;
//  targetPositionLong = -round(((cos(((float)counter * 0.0001f)) - 1) * 20000.0f));
//
//  if (counter % 1 == 0) {
//    if (forwards) {
//      targetPositionLong += 1;
//      if (targetPositionLong > 10000) forwards = false;
//    }
//    else {
//      targetPositionLong -= 1;
//      if (targetPositionLong < -10000) forwards = true;
//    }
//  }
  int targetchange; 
  cli(); 
  targetchange = targetPositionChange; 
  targetPositionChange = 0;
  sei(); 
  targetPositionLong += targetchange;
  position = xPosn.calcPosn();
  
checkPhase();
  
  /*Compute all the working error variables*/
  float error = targetPositionLong - position;
  

  ITerm += (Ki * error);
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
  float dInput = (position - lastInput);


 
  /*Compute PID Output*/
  float output = Kp * error + ITerm - Kd * dInput;

  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;
  motorPower = -output;


  /*Remember some variables for next time*/
  lastInput = position;

  errorValue = targetPositionLong - position;
  if (errorValue < 0) errorValue *= -1;
 // motorPower = 40; 

  updateMotor();

}


void initPID() {
  //encoder.write(0);
  position = targetPositionLong = xPosn.calcPosn(); 
  motorPower = 0;
  
  
  //sendPIDOverSerial();

  // PID updated 5000 times a second
  pidTimer.begin(computePid, 1000000 / 5000);
  // priority number tested to prove that it can maintain 5k plus also 
  // can receive step and quad pulses. Lower number is higher priority. 
  pidTimer.priority(145);
 //pidTimer.priority(160);

}

