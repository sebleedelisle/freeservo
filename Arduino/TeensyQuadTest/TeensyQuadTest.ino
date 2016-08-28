

#include "LedControl.h"
#include "QuadDecode_def.h"

#include <PID_v1.h>
#include "Motors.h"

LedControl lc = LedControl(12, 11, 8, 1);
QuadDecode<1> xPosn;  // Template using FTM1

const int stepPin = 5; 
const int dirPin = 6;
const int resetSwitch = 14;
 
//int32_t motorPosition = 0;
//volatile int32_t stepPosition = 0;
//int32_t targetMotorPosition = 0; 

volatile double position, targetPositionDouble, motorPower, errorValue;
volatile long targetPositionLong = 0;

double Kp = 15.5, Ki = 5.71, Kd = 0.0184;

PID pid(&position, &motorPower, &targetPositionDouble, Kp, Ki, Kd, DIRECT);


void setup()
{

  pinMode(stepPin, INPUT); 
  pinMode(dirPin, INPUT); 
  pinMode(resetSwitch, INPUT_PULLUP); 

  xPosn.setup();      // Start Quad Decode position count
  xPosn.start();      // Start Quad Decode position count
  attachInterrupt(stepPin, readStep, RISING); 
  initMotor();
  init7SegDisplay();

}

void loop() {
   position = xPosn.calcPosn();
 
  updatePID(); 
  //targetPositionDouble = targetPositionLong; 

  //showNumber(targetPositionDouble - position);
  //showNumber(motorPower*1000000);
    
  if(digitalReadFast(resetSwitch)==LOW) { 
    targetPositionLong = 0; 
    position = 0; 
    targetPositionDouble =  0; 
  }
  
  setMotorPower(map(analogRead(A9), 0, 1023, -100,100)); 
 showNumber(map(analogRead(A9), 0, 1023, -100,100));
 
}

void readStep() { 
  if(digitalReadFast(dirPin)) { 
    targetPositionLong++; 
  } else { 
    targetPositionLong--; 
  }
}



void init7SegDisplay() {

  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0, 2);
  /* and clear the display */
  lc.clearDisplay(0);

}

long power10[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000};

inline void showNumber(int32_t number) {


  boolean negative = false;


  if (number < 0) {
    number = abs(number);
    negative = true;

  }
  boolean minusShown = false; 
  
  for (int digit = 0; digit <8; digit++) {
    long powerval = power10[digit]; //pow(10,digit);
    if ((digit > 0) && (number < powerval)) {
      if (negative && !minusShown) {
        lc.setChar(0, digit, '-', false);
        minusShown = true;
      }
      else {
        lc.setChar(0, digit, ' ', false);
      }
      
    } else {
      lc.setDigit(0, digit, (number / powerval) % 10, false);
    }
  }

}


void initPID() { 
  //encoder.write(0); 
  position = targetPositionDouble = motorPower = 0; 
  targetPositionLong = 0;
  pid.SetOutputLimits(-100, 100); 
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1);
  pid.SetTunings(Kp, Ki, Kd);
  targetPositionLong = targetPositionDouble;
  //sendPIDOverSerial();

}

inline void updatePID() { 

  // reset if the numbers get too high!
  //  if(targetPositionLong>maxValue) {
  //    
  //    targetPositionLong-=maxValue; 
  //    targetPositionDouble-=maxValue; 
  //    position-=maxValue; 
  //  }
  //  
  //  if(targetPositionLong<-maxValue) {
  //    targetPositionLong+=maxValue; 
  //    targetPositionDouble+=maxValue; 
  //    position+=maxValue; 
  //  }

  targetPositionDouble=targetPositionLong; // Copy the integer value updated by the ISR into the float value used by PID

  if(pid.Compute()){
//    if(counter>33) { // update 30 times a second 
//      // send position over serial
//      if(sendPos) {
//        Serial.print("POS:"); 
//        Serial.print(targetPositionDouble); 
//        Serial.print(","); 
//        Serial.println(position); 
//      }
//      counter = 0; 
//    }
//    counter ++;
  }

  errorValue = targetPositionDouble - position; 
  if(errorValue<0) errorValue*=-1; 

}

